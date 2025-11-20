#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <Preferences.h>

#include "soc/rtc.h"
#include "driver/gpio.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <Preferences.h>

// Pin definitions (adjust per your hardware)
#define USING_SX1276  // Or SX1278, SX1262, LR1121 if matching your board

#define RADIO_CS_PIN    18
#define RADIO_DIO0_PIN  26
#define RADIO_DIO1_PIN  33
#define RADIO_RST_PIN   23
#define RADIO_BUSY_PIN  32
#define RADIO_SCLK_PIN  5
#define RADIO_MISO_PIN  19
#define RADIO_MOSI_PIN  27

#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#define GPS_BAUD_RATE   9600

#define PMU_IRQ         35

#define BOARD_LED       2
#define LED_ON          LOW

#define HAS_PMU         // enable this if using PMU code
#define PMU_WIRE_PORT Wire

#ifdef HAS_PMU
#include "XPowersLibInterface.hpp"
#include "XPowersAXP2101.tpp"
#include "XPowersAXP192.tpp"

// --- Preferences ---
Preferences store;

// --- GPS ---
#define SerialGPS Serial1    // GPS module serial port
TinyGPSPlus gps;
static bool find_gps = false;
String gps_model = "None";

// --- Radio and LoRaWAN ---
#if defined(USING_SX1276)
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#elif defined(USING_SX1262)
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#elif defined(USING_SX1278)
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#elif defined(USING_LR1121)
LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

#define RADIOLIB_LORAWAN_JOIN_EUI  0x7E207DCE9297F26F
#ifndef RADIOLIB_LORAWAN_DEV_EUI
#define RADIOLIB_LORAWAN_DEV_EUI   0x7E07F85817CACA51
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY
#define RADIOLIB_LORAWAN_APP_KEY   0xDE, 0x2E, 0x4A, 0xC2, 0x9C, 0x98, 0xDC, 0x5A, 0x71, 0xF1, 0xF1, 0xCD, 0xFD, 0x4E, 0x0E, 0xD0
#endif
#ifndef RADIOLIB_LORAWAN_NWK_KEY
#define RADIOLIB_LORAWAN_NWK_KEY   0x94, 0xE9, 0x1F, 0xAE, 0x47, 0xEF, 0x69, 0x02, 0x72, 0x37, 0xDB, 0xE8, 0xF8, 0x68, 0x7B, 0x75
#endif

const uint32_t uplinkIntervalSeconds = 2UL * 60UL;    // 2 minutes

const LoRaWANBand_t Region = EU868;
const uint8_t subBand = 0;

uint64_t joinEUI = { RADIOLIB_LORAWAN_JOIN_EUI };
uint64_t devEUI  = { RADIOLIB_LORAWAN_DEV_EUI };
uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

LoRaWANNode node(&radio, &Region, subBand);

// Session buffer for RTC persistence
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

XPowersLibInterface *PMU = NULL;
bool pmuInterrupt;

static void IRAM_ATTR setPmuFlag() {
    pmuInterrupt = true;
}

bool beginPower() {
    if (!PMU) {
        PMU = new XPowersAXP2101(PMU_WIRE_PORT);
        if (!PMU->init()) {
            delete PMU; PMU = NULL;
            PMU = new XPowersAXP192(PMU_WIRE_PORT);
            if (!PMU->init()) {
                delete PMU; PMU = NULL;
                return false;
            }
        }
    }

    // Power rails and IRQ configuration here
    // (Same detailed configuration from original LoRaBoards.cpp omitted for brevity)

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, setPmuFlag, FALLING);

    PMU->enableSystemVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableBattVoltageMeasure();

    Serial.println("Power management initialized");
    return true;
}

void disablePeripherals() {
    if (!PMU) return;
    PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
    PMU->disableSystemVoltageMeasure();
    PMU->disableVbusVoltageMeasure();
    PMU->disableBattVoltageMeasure();
    PMU->disableBattDetection();

    PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    PMU->clearIrqStatus();

    // Power off unused channels as in original
}
#endif

// --- GPS functions ---
bool l76kProbe()
{
    bool result = false;
    uint32_t startTimeout;

    SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(5);

    startTimeout = millis() + 3000;
    while (SerialGPS.available()) {
        if (millis() > startTimeout) {
            Serial.println("Wait L76K stop NMEA timeout!");
            return false;
        }
        SerialGPS.read();
    };

    SerialGPS.flush();
    delay(200);

    SerialGPS.write("$PCAS06,0*1B\r\n");
    startTimeout = millis() + 500;
    while (!SerialGPS.available()) {
        if (millis() > startTimeout) {
            Serial.println("Get L76K timeout!");
            return false;
        }
    }
    SerialGPS.setTimeout(10);
    String ver = SerialGPS.readStringUntil('\n');
    if (ver.startsWith("$GPTXT,01,01,02")) {
        Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
        result = true;
    }
    delay(500);

    SerialGPS.write("$PCAS04,5*1C\r\n");
    delay(250);

    SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);

    SerialGPS.write("$PCAS11,3*1E\r\n");
    return result;
}

bool beginGPS()
{
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    bool result = false;
    for (int i = 0; i < 3; ++i) {
        result = l76kProbe();
        if (result) {
            return result;
        }
    }
    return result;
}

static int getAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID) {
    uint16_t ubxFrameCounter = 0;
    uint32_t startTime = millis();
    uint16_t needRead;

    while (millis() - startTime < 800) {
        while (SerialGPS.available()) {
            int c = SerialGPS.read();
            switch (ubxFrameCounter) {
                case 0: if (c == 0xB5) ubxFrameCounter++; break;
                case 1: if (c == 0x62) ubxFrameCounter++; else ubxFrameCounter = 0; break;
                case 2: if (c == requestedClass) ubxFrameCounter++; else ubxFrameCounter = 0; break;
                case 3: if (c == requestedID) ubxFrameCounter++; else ubxFrameCounter = 0; break;
                case 4: needRead = c; ubxFrameCounter++; break;
                case 5: needRead |=  (c << 8); ubxFrameCounter++; break;
                case 6:
                    if (needRead >= size) {
                        ubxFrameCounter = 0;
                        break;
                    }
                    if (SerialGPS.readBytes(buffer, needRead) != needRead) {
                        ubxFrameCounter = 0;
                    } else {
                        return needRead;
                    }
                    break;
                default: break;
            }
        }
    }
    return 0;
}

bool recoveryGPS()
{
    uint8_t buffer[256];
    uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x02, 0x1C, 0xA2};
    uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x01, 0x1B, 0xA1};
    uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
                           0x03, 0x1D, 0xB3};
    SerialGPS.write(cfg_clear1, sizeof(cfg_clear1));
    if (getAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear2, sizeof(cfg_clear2));
    if (getAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear3, sizeof(cfg_clear3));
    if (getAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    SerialGPS.write(cfg_rate, sizeof(cfg_rate));
    if (getAck(buffer, 256, 0x06, 0x08)) {
        Serial.println("Get ack successes!");
    } else {
        return false;
    }
    return true;
}

// --- Helper functions for error decoding and debugging ---

String stateDecode(const int16_t result) {
    switch (result) {
        case RADIOLIB_ERR_NONE: return "ERR_NONE";
        case RADIOLIB_ERR_CHIP_NOT_FOUND: return "ERR_CHIP_NOT_FOUND";
        case RADIOLIB_ERR_PACKET_TOO_LONG: return "ERR_PACKET_TOO_LONG";
        case RADIOLIB_ERR_RX_TIMEOUT: return "ERR_RX_TIMEOUT";
        case RADIOLIB_ERR_CRC_MISMATCH: return "ERR_CRC_MISMATCH";
        case RADIOLIB_ERR_INVALID_BANDWIDTH: return "ERR_INVALID_BANDWIDTH";
        case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "ERR_INVALID_SPREADING_FACTOR";
        case RADIOLIB_ERR_INVALID_CODING_RATE: return "ERR_INVALID_CODING_RATE";
        case RADIOLIB_ERR_INVALID_FREQUENCY: return "ERR_INVALID_FREQUENCY";
        case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "ERR_INVALID_OUTPUT_POWER";
        case RADIOLIB_ERR_NETWORK_NOT_JOINED: return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
        case RADIOLIB_ERR_DOWNLINK_MALFORMED: return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
        case RADIOLIB_ERR_INVALID_REVISION: return "RADIOLIB_ERR_INVALID_REVISION";
        case RADIOLIB_ERR_INVALID_PORT: return "RADIOLIB_ERR_INVALID_PORT";
        case RADIOLIB_ERR_NO_RX_WINDOW: return "RADIOLIB_ERR_NO_RX_WINDOW";
        case RADIOLIB_ERR_INVALID_CID: return "RADIOLIB_ERR_INVALID_CID";
        case RADIOLIB_ERR_UPLINK_UNAVAILABLE: return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
        case RADIOLIB_ERR_COMMAND_QUEUE_FULL: return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
        case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND: return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
        case RADIOLIB_ERR_JOIN_NONCE_INVALID: return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
        case RADIOLIB_ERR_N_FCNT_DOWN_INVALID: return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
        case RADIOLIB_ERR_A_FCNT_DOWN_INVALID: return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
        case RADIOLIB_ERR_DWELL_TIME_EXCEEDED: return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
        case RADIOLIB_ERR_CHECKSUM_MISMATCH: return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
        case RADIOLIB_ERR_NO_JOIN_ACCEPT: return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
        case RADIOLIB_LORAWAN_SESSION_RESTORED: return "RADIOLIB_LORAWAN_SESSION_RESTORED";
        case RADIOLIB_LORAWAN_NEW_SESSION: return "RADIOLIB_LORAWAN_NEW_SESSION";
        case RADIOLIB_ERR_NONCES_DISCARDED: return "RADIOLIB_ERR_NONCES_DISCARDED";
        case RADIOLIB_ERR_SESSION_DISCARDED: return "RADIOLIB_ERR_SESSION_DISCARDED";
    }
    return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

void debug(bool failed, const __FlashStringHelper *message, int state, bool halt) {
    if (failed) {
        Serial.print(message);
        Serial.print(" - ");
        Serial.print(stateDecode(state));
        Serial.print(" (");
        Serial.print(state);
        Serial.println(")");
        while (halt) { delay(1); }
    }
}

void arrayDump(uint8_t *buffer, uint16_t len) {
    for (uint16_t c = 0; c < len; c++) {
        char b = buffer[c];
        if (b < 0x10) { Serial.print('0'); }
        Serial.print(b, HEX);
    }
    Serial.println();
}

// --- Board and peripheral setup ---

void setupBoards() {
    Serial.begin(115200);
    while (!Serial);

    // SPI initialization
#if defined(ARDUINO_ARCH_ESP32)
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
#elif defined(ARDUINO_ARCH_STM32)
    SPI.setMISO(RADIO_MISO_PIN);
    SPI.setMOSI(RADIO_MOSI_PIN);
    SPI.setSCLK(RADIO_SCLK_PIN);
    SPI.begin();
#endif

    // GPS Serial port already initialized in beginGPS()

#ifdef RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

#ifdef RADIO_LDO_EN
    pinMode(RADIO_LDO_EN, OUTPUT);
    digitalWrite(RADIO_LDO_EN, HIGH);
#endif

#ifdef RADIO_CTRL
    pinMode(RADIO_CTRL, OUTPUT);
    digitalWrite(RADIO_CTRL, LOW);
#endif

#ifdef GPS_EN_PIN
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
#endif

#ifdef BOARD_LED
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif

    beginPower();

#ifdef HAS_GPS
    find_gps = beginGPS();
    if (!find_gps) {
        uint32_t baudrate[] = {9600, 19200, 38400, 57600, 115200};
        for (int i = 0; i < sizeof(baudrate) / sizeof(baudrate[0]); ++i) {
            Serial.printf("Update baudrate : %u\n", baudrate[i]);
            SerialGPS.updateBaudRate(baudrate[i]);
            if (recoveryGPS()) {
                Serial.println("UBlox GNSS init succeeded, using UBlox GNSS Module\n");
                gps_model = "UBlox";
                find_gps = true;
                break;
            }
        }
    } else {
        gps_model = "L76K";
    }
#endif

    Serial.println("init done.");
}

// --- Setup and Loop ---

void setup() {
    setupBoards();

    Serial.println(F("GPS initialized"));

    Serial.println(F("Initialise the radio"));
    int16_t state = radio.begin();
    debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

    node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

    store.begin("radiolib");

    if (store.isKey("nonces")) {
        uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
        store.getBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
        state = node.setBufferNonces(buffer);
        debug(state != RADIOLIB_ERR_NONE, F("Restoring nonces buffer failed"), state, false);

        state = node.setBufferSession(LWsession);
        debug(state != RADIOLIB_ERR_NONE, F("Restoring session buffer failed"), state, false);

        if (state == RADIOLIB_ERR_NONE) {
            Serial.println(F("Successfully restored session - now activating"));
            state = node.activateOTAA();
            debug(state != RADIOLIB_LORAWAN_SESSION_RESTORED, F("Failed to activate restored session"), state, true);
            store.end();
        }
    } else {
        Serial.println(F("No Nonces saved - starting fresh."));
    }

    uint32_t sleepForSeconds = 60 * 1000;
    state = RADIOLIB_ERR_NETWORK_NOT_JOINED;

    while (state != RADIOLIB_LORAWAN_NEW_SESSION) {
        Serial.println(F("Join ('login') to the LoRaWAN Network"));
        state = node.activateOTAA();

        uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
        uint8_t *persist = node.getBufferNonces();
        memcpy(buffer, persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
        store.putBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

        if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
            Serial.print(F("Join failed: "));
            Serial.println(state);
            Serial.print(F("Retrying join in "));
            Serial.print(sleepForSeconds / 1000);
            Serial.println(F(" seconds"));
            delay(sleepForSeconds);
        }
    }

    Serial.print("[LoRaWAN] DevAddr: ");
    Serial.println((unsigned long)node.getDevAddr(), HEX);

    node.setADR(true);
    node.setDatarate(5);
    node.setDutyCycle(true, 1250);
    node.setDwellTime(true, 400);

    Serial.println(F("Ready!\n"));
}

void loop() {

    int16_t state = RADIOLIB_ERR_NONE;
    uint8_t battLevel = 146;
    node.setDeviceStatus(battLevel);

    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    uint8_t uplinkPayload[8];
    if (gps.location.isValid()) {
        int32_t lat = int32_t(gps.location.lat() * 1000000);
        int32_t lng = int32_t(gps.location.lng() * 1000000);

        uplinkPayload[0] = (lat >> 24) & 0xFF;
        uplinkPayload[1] = (lat >> 16) & 0xFF;
        uplinkPayload[2] = (lat >> 8) & 0xFF;
        uplinkPayload[3] = lat & 0xFF;

        uplinkPayload[4] = (lng >> 24) & 0xFF;
        uplinkPayload[5] = (lng >> 16) & 0xFF;
        uplinkPayload[6] = (lng >> 8) & 0xFF;
        uplinkPayload[7] = lng & 0xFF;

        Serial.print(F("GPS Fix: Lat = "));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(", Lng = "));
        Serial.println(gps.location.lng(), 6);

    } else {
        Serial.println(F("No valid GPS fix yet, sending empty payload"));
        memset(uplinkPayload, 0xFF, sizeof(uplinkPayload));
    }

    uint8_t downlinkPayload[10];
    size_t downlinkSize;

    LoRaWANEvent_t uplinkDetails;
    LoRaWANEvent_t downlinkDetails;
    uint8_t fPort = 10;
    uint32_t fCntUp = node.getFCntUp();

    Serial.println(F("Sending uplink"));
    if (fCntUp == 1) {
        Serial.println(F("and requesting LinkCheck and DeviceTime"));
        node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
        node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
        state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload), fPort, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails);
    } else {
        state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload), fPort, downlinkPayload, &downlinkSize, false, &uplinkDetails, &downlinkDetails);
    }

    debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);

    if (state > 0) {
        Serial.println(F("Received a downlink"));
        if (downlinkSize > 0) {
            Serial.println(F("Downlink data: "));
            arrayDump(downlinkPayload, downlinkSize);
        } else {
            Serial.println(F("<MAC commands only>"));
        }

        Serial.print(F("[LoRaWAN] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        Serial.print(F("[LoRaWAN] SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        uint8_t margin = 0, gwCnt = 0;
        if (node.getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
            Serial.print(F("[LoRaWAN] LinkCheck margin:\t"));
            Serial.println(margin);
            Serial.print(F("[LoRaWAN] LinkCheck count:\t"));
            Serial.println(gwCnt);
        }

        uint32_t networkTime = 0;
        uint8_t fracSecond = 0;
        if (node.getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
            Serial.print(F("[LoRaWAN] DeviceTime Unix:\t"));
            Serial.println(networkTime);
            Serial.print(F("[LoRaWAN] DeviceTime second:\t1/"));
            Serial.println(fracSecond);
        }

    } else {
        Serial.println(F("[LoRaWAN] No downlink received"));
    }

    uint32_t minimumDelay = uplinkIntervalSeconds * 1000UL;
    uint32_t interval = node.timeUntilUplink();
    uint32_t delayMs = max(interval, minimumDelay);

    Serial.print(F("[LoRaWAN] Next uplink in "));
    Serial.print(delayMs / 1000);
    Serial.println(F(" seconds\n"));

    delay(delayMs);
}
