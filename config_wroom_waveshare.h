/**
 * ============================================================================
 * config.h - ESP32 WROOM DevKit + Waveshare SX1262 HF Configuration
 * ============================================================================
 * SX1262 Driver Suite v1.0.0
 * Board: ESP32 WROOM DevKit + External Waveshare SX1262 HF Module
 * MCU: ESP32 (Classic)
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// BOARD IDENTIFICATION
// ============================================================================
#define BOARD_NAME              "ESP32 WROOM + Waveshare SX1262 HF"
#define BOARD_HAS_BUILTIN_SX1262 0
#define BOARD_HAS_OLED          0

// ============================================================================
// SX1262 PIN CONFIGURATION - ESP32 WROOM + WAVESHARE
// ============================================================================

// SPI Pins (Using VSPI - the default SPI bus on ESP32)
// VSPI is GPIO 18 (SCK), 19 (MISO), 23 (MOSI), 5 (SS)
#define PIN_SX1262_MOSI         23
#define PIN_SX1262_MISO         19
#define PIN_SX1262_SCK          18
#define PIN_SX1262_CS           5

// Control Pins
#define PIN_SX1262_NRESET       16
#define PIN_SX1262_BUSY         4
#define PIN_SX1262_DIO1         2

// RF Switch Control (Waveshare module has external RF switch)
#define USE_DIO2_RF_SWITCH      true    // DIO2 controls TXEN internally
#define PIN_SX1262_TXEN         17      // Optional: manual TXEN control
// RXEN should be tied HIGH (3.3V) when using DIO2 for TX control

// ============================================================================
// ESP32 WROOM PIN RESTRICTIONS - IMPORTANT!
// ============================================================================
// DO NOT USE these pins (connected to internal flash):
//   GPIO 6, 7, 8, 9, 10, 11
// 
// STRAPPING PINS (avoid or handle carefully):
//   GPIO 0  - Boot mode (has internal pull-up)
//   GPIO 2  - Boot mode (must be LOW or floating for boot)
//   GPIO 12 - Flash voltage (must be LOW for 3.3V flash)
//   GPIO 15 - Silences boot messages if pulled LOW
//
// INPUT-ONLY PINS (cannot be used for output):
//   GPIO 34, 35, 36, 39
//
// ADC2 PINS (unavailable when WiFi active):
//   GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27

// ============================================================================
// ALTERNATIVE HSPI CONFIGURATION (if VSPI is needed elsewhere)
// ============================================================================
// Uncomment to use HSPI instead of VSPI:
// #define USE_HSPI_BUS          1
// #define PIN_SX1262_MOSI       13
// #define PIN_SX1262_MISO       12
// #define PIN_SX1262_SCK        14
// #define PIN_SX1262_CS         15
// Note: GPIO 12 is a strapping pin - may cause boot issues!

// ============================================================================
// SPI CONFIGURATION
// ============================================================================
#define SPI_FREQUENCY           8000000   // 8 MHz
#define SPI_MODE                SPI_MODE0

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
#define DEBUG_ENABLED           1
#define DEBUG_VERBOSE_SPI       0
#define DEBUG_VERBOSE_STATE     0
#define DEBUG_VERBOSE_IRQ       0

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define BUSY_TIMEOUT_MS         100
#define TX_TIMEOUT_MARGIN_MS    500
#define RX_POST_IRQ_DELAY_MS    3

// ============================================================================
// PLATFORM IDENTIFICATION
// ============================================================================
#define PLATFORM_ESP32          1       // Classic ESP32 (not S2/S3/C3)
#define HAS_FREERTOS            1
#define USE_HARDWARE_SPI        1

// ============================================================================
// OPTIONAL: STATUS LED
// ============================================================================
#define PIN_LED                 2       // Built-in LED on most ESP32 DevKits
                                        // Note: GPIO 2 is shared with DIO1 in this config
                                        // Change to 22 or other free pin if needed

#endif // CONFIG_H
