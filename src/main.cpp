/* T-Beam SX1276 + AXP2101/AXP192 + TinyGPS++ + RadioLib LoRaWAN OTAA
   Payload (10 bytes, 1e7 GPS precision):
     bytes[0..3] = latitude  (int32, signed, ×1e7)
     bytes[4..7] = longitude (int32, signed, ×1e7)
     byte[8]     = battery_percent (0..100)
     byte[9]     = flags
                   bit0 = buzzer
                   bit1 = collar_state
                   bit2 = gps_valid
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <Preferences.h>
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "gps_reader.h"

// ------------------- Constants -------------------
static const double COORD_SCALE = 10000000.0; // 1e7

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define BUTTON_PIN 38
#define I2C_SDA 21
#define I2C_SCL 22
#define PMU_IRQ 35
#define BOARD_LED 4
#define LED_ON LOW
#define LED_OFF HIGH

static const uint32_t GPSBaud = 9600;

// ------------------- Radio Pins -------------------
#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 19
#define RADIO_MOSI_PIN 27
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_DIO1_PIN 33
#define RADIO_RST_PIN 23

// ------------------- PMU -------------------
#define HAS_PMU
#define PMU_WIRE_PORT Wire
#ifdef HAS_PMU
#include "XPowersLibInterface.hpp"
#include "XPowersAXP2101.tpp"
#include "XPowersAXP192.tpp"
Preferences store;
XPowersLibInterface *PMU = nullptr;
volatile bool pmuInterrupt = false;
static void IRAM_ATTR setPmuFlag() { pmuInterrupt = true; }
#endif

// ------------------- GPS -------------------
TinyGPSPlus gps;

// ------------------- Radio -------------------
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

// ------------------- LoRaWAN -------------------
#define RADIOLIB_LORAWAN_JOIN_EUI 0x7E207DCE9297F26F
#define RADIOLIB_LORAWAN_DEV_EUI  0x70B3D57ED00740C8
#define RADIOLIB_LORAWAN_APP_KEY  0xDE,0x2E,0x4A,0xC2,0x9C,0x98,0xDC,0x5A,0x71,0xF1,0xF1,0xCD,0xFD,0x4E,0x0E,0xD0
#define RADIOLIB_LORAWAN_NWK_KEY  0x94,0xE9,0x1F,0xAE,0x47,0xEF,0x69,0x02,0x72,0x37,0xDB,0xE8,0xF8,0x68,0x7B,0x75

const uint32_t uplinkIntervalSeconds = 60;
const LoRaWANBand_t Region = EU868;
const uint8_t subBand = 0;

uint64_t joinEUI = { RADIOLIB_LORAWAN_JOIN_EUI };
uint64_t devEUI  = { RADIOLIB_LORAWAN_DEV_EUI };
uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

LoRaWANNode node(&radio, &Region, subBand);
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

// ------------------- Helpers -------------------
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available()) gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

// Read battery percent
uint8_t readBatteryPercent() {
#ifdef HAS_PMU
  if (PMU && PMU->isBatteryConnect()) return PMU->getBatteryPercent();
#endif
  return 50;
}

// Handle PMU interrupts
void handlePmuInterruptIfAny() {
#ifdef HAS_PMU
  if (!PMU || !pmuInterrupt) return;
  pmuInterrupt = false;
  uint32_t status = PMU->getIrqStatus();
  Serial.print("PMU IRQ status: 0x"); Serial.println(status, HEX);
  if (PMU->isVbusInsertIrq()) Serial.println("Vbus Insert");
  if (PMU->isVbusRemoveIrq()) Serial.println("Vbus Remove");
  if (PMU->isBatInsertIrq()) Serial.println("Battery Insert");
  if (PMU->isBatRemoveIrq()) Serial.println("Battery Remove");
  if (PMU->isBatChargeStartIrq()) Serial.println("Battery charge start");
  if (PMU->isBatChargeDoneIrq())  Serial.println("Battery charge done");
  PMU->clearIrqStatus();
#endif
}

// Debug print helper
void arrayDump(uint8_t *buffer, uint16_t len) {
  for (uint16_t c = 0; c < len; c++) {
    if (buffer[c] < 0x10) Serial.print('0');
    Serial.print(buffer[c], HEX); Serial.print(' ');
  }
  Serial.println();
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LED_ON);

  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

#ifdef HAS_PMU
  PMU = new XPowersAXP2101(PMU_WIRE_PORT);
  if (!PMU->init()) {
    delete PMU;
    PMU = new XPowersAXP192(PMU_WIRE_PORT);
    if (!PMU->init()) { delete PMU; PMU = nullptr; }
  }
  if (PMU) {
    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, setPmuFlag, FALLING);
    PMU->enableBattVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableSystemVoltageMeasure();
  }
#endif

  Serial1.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(200);
  Serial.print(F("TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());

  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("Radio init failed");
    while (true);
  }

  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

  // Restore nonces/session
  store.begin("radiolib");
  if (store.isKey("nonces")) {
      uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
      store.getBytes("nonces", buffer, sizeof(buffer));
      node.setBufferNonces(buffer);
      node.setBufferSession(LWsession);
      if (node.activateOTAA() == RADIOLIB_ERR_NONE)
          Serial.println(F("Restored session and activated"));
  }
  store.end();

  // Join loop
  while (node.activateOTAA() != RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.println(F("Join failed - retrying in 60s"));
      delay(60000);
  }

  node.setADR(true);
  node.setDatarate(5); // SF7
  node.setDutyCycle(true, 1250);
  node.setDwellTime(true, 400);

  Serial.println(F("System ready"));
}

// ------------------- Main Loop -------------------
void loop() {
    handlePmuInterruptIfAny();

    // --- GPS update every 60s ---
    static uint32_t lastGpsRead = 0;
    static double lastLat = 0, lastLon = 0;
    const uint32_t gpsReadInterval = 60000;

    if (millis() - lastGpsRead > gpsReadInterval) {
        lastGpsRead = millis();
        smartDelay(200);
        if (gps.location.isValid()) {
            lastLat = gps.location.lat();
            lastLon = gps.location.lng();

            Serial.print("GPS Fix: ");
            Serial.print(lastLat, 7); Serial.print(", ");
            Serial.println(lastLon, 7);

            int32_t latScaled = (int32_t)(lastLat * COORD_SCALE);
            int32_t lonScaled = (int32_t)(lastLon * COORD_SCALE);
            Serial.print("Scaled lat: "); Serial.print(latScaled);
            Serial.print("  lon: "); Serial.println(lonScaled);
        } else {
            Serial.println("No GPS fix");
        }
    }

    // --- Build payload ---
    uint8_t uplinkPayload[10];
    memset(uplinkPayload, 0xFF, sizeof(uplinkPayload));
    bool gps_valid = (lastLat != 0 && lastLon != 0);

    if (gps_valid) {
        int32_t lat = (int32_t)(lastLat * COORD_SCALE);
        int32_t lon = (int32_t)(lastLon * COORD_SCALE);

        uplinkPayload[0] = lat >> 24;
        uplinkPayload[1] = lat >> 16;
        uplinkPayload[2] = lat >> 8;
        uplinkPayload[3] = lat;

        uplinkPayload[4] = lon >> 24;
        uplinkPayload[5] = lon >> 16;
        uplinkPayload[6] = lon >> 8;
        uplinkPayload[7] = lon;
    }

    uint8_t battery = readBatteryPercent();
    uint8_t buzzer_state = 0;
    uint8_t collar_state = digitalRead(BUTTON_PIN) ? 1 : 0;

    uplinkPayload[8] = battery;
    uplinkPayload[9] =
        (buzzer_state & 0x01) |
        ((collar_state & 0x01) << 1) |
        ((gps_valid ? 1 : 0) << 2);

    // --- Debug payload ---
    Serial.print("Outgoing payload (hex): ");
    arrayDump(uplinkPayload, sizeof(uplinkPayload));

    // --- Send unconfirmed uplink ---
    uint8_t downlinkPayload[10];
    size_t downlinkSize = 0;
    LoRaWANEvent_t uplinkDetails, downlinkDetails;

    bool confirmed = false;
    int16_t state = node.sendReceive(
        uplinkPayload,
        sizeof(uplinkPayload),
        10,
        downlinkPayload,
        &downlinkSize,
        confirmed,
        &uplinkDetails,
        &downlinkDetails
    );

    if (state < 0) Serial.println("Send error");
    else if (state > 0) {
        Serial.println("Received downlink/MAC:");
        if (downlinkSize > 0) arrayDump(downlinkPayload, downlinkSize);
        else Serial.println("<MAC only or empty>");
    } else Serial.println("[LoRaWAN] No downlink");

    // --- Next uplink ---
    uint32_t minDelay = uplinkIntervalSeconds * 1000UL;
    uint32_t interval = node.timeUntilUplink();
    uint32_t delayMs = max(interval, minDelay);

    Serial.print("[LoRaWAN] Next uplink in ");
    Serial.print(delayMs / 1000); Serial.println(" sec\n");

    delay(delayMs);
}