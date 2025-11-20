#pragma once

#include "utilities.h"

#ifdef HAS_SDCARD
#include <SD.h>
#endif

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#ifdef HAS_PMU
#include <XPowersLib.h>
#endif

#include <esp_mac.h>

// Online device flags (display & WiFi removed)
enum {
    POWERMANAGE_ONLINE  = _BV(0),
    RADIO_ONLINE        = _BV(2),
    GPS_ONLINE          = _BV(3),
    PSRAM_ONLINE        = _BV(4),
    SDCARD_ONLINE       = _BV(5),
    AXDL345_ONLINE      = _BV(6),
    BME280_ONLINE       = _BV(7),
    BMP280_ONLINE       = _BV(8),
    BME680_ONLINE       = _BV(9),
    QMC6310_ONLINE      = _BV(10),
    QMI8658_ONLINE      = _BV(11),
    PCF8563_ONLINE      = _BV(12),
    OSC32768_ONLINE     = _BV(13),
};

typedef struct {
    String chipModel;
    float psramSize;
    uint8_t chipModelRev;
    uint8_t chipFreq;
    uint8_t flashSize;
    uint8_t flashSpeed;
} DevInfo_t;

void setupBoards(bool disable_u8g2 = false);

#ifdef HAS_SDCARD
bool beginSDCard();
#else
#define beginSDCard()
#endif

void printResult(bool radio_online);

#ifdef BOARD_LED
void flashLed();
#else
#define flashLed()
#endif

void scanDevices(TwoWire *w);

bool beginGPS();
bool recoveryGPS();

#ifdef HAS_PMU
extern XPowersLibInterface *PMU;
extern bool pmuInterrupt;
void loopPMU(void (*pressed_cb)(void));
bool beginPower();
void disablePeripherals();
#else
#define beginPower()
#endif

#if defined(ARDUINO_ARCH_ESP32)
#if defined(HAS_SDCARD)
extern SPIClass SDCardSPI;
#endif
#define SerialGPS Serial1
#elif defined(ARDUINO_ARCH_STM32)
extern HardwareSerial SerialGPS;
#endif

#ifdef HAS_NTC
float getTempForNTC();
#endif

#ifdef ENABLE_BLE
void setupBLE();
#endif

extern uint32_t deviceOnline;
