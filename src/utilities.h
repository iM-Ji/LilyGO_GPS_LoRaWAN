/**
 * @file      utilities.h
 * @author    Lewis He
 * @license   MIT
 * @date      2024-05-12
 * @last-update 2025-07-07
 */
#pragma once

#define UNUSED_PIN                   (0)

//------------------ Select Board ------------------//
// Only SX1276 boards are used
#ifndef T_BEAM_SX1276
#define T_BEAM_SX1276
#endif
// #ifndef T3_V1_3_SX1276
// #define T3_V1_3_SX1276
// #endif
// #ifndef T3_V1_6_SX1276
// #define T3_V1_6_SX1276
// #endif

//------------------ MCU / Common Pins ------------------//
#define GPS_RX_PIN                  34
#define GPS_TX_PIN                  12
#define BUTTON_PIN                  38
#define BUTTON_PIN_MASK             GPIO_SEL_38
#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

#define GPS_BAUD_RATE               9600
#define HAS_GPS

//------------------ Radio Pins ------------------//
#if defined(T_BEAM_SX1276)
#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26
#define RADIO_DIO1_PIN              33
#define RADIO_DIO2_PIN              32
#define RADIO_RST_PIN               23
#define RADIO_BUSY_PIN              33   // SX1276 BUSY/DIO1
#define BOARD_VARIANT_NAME          "T-Beam SX1276"
#endif

#if defined(T3_V1_3_SX1276)
#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26
#define RADIO_DIO1_PIN              33
#define RADIO_DIO2_PIN              32
#define RADIO_RST_PIN               14
#define RADIO_BUSY_PIN              32
#define BOARD_VARIANT_NAME          "T3 V1.3 SX1276"
#endif

#if defined(T3_V1_6_SX1276)
#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26
#define RADIO_DIO1_PIN              33
#define RADIO_DIO2_PIN              32
#define RADIO_RST_PIN               23
#define RADIO_BUSY_PIN              32
#define BOARD_VARIANT_NAME          "T3 V1.6 SX1276"
#endif
