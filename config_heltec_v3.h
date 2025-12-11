/**
 * ============================================================================
 * config.h - Heltec WiFi LoRa 32 V3 Configuration
 * ============================================================================
 * SX1262 Driver Suite v1.0.0
 * Board: Heltec WiFi LoRa 32 V3 (Built-in SX1262)
 * MCU: ESP32-S3
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// BOARD IDENTIFICATION
// ============================================================================
#define BOARD_NAME              "Heltec WiFi LoRa 32 V3"
#define BOARD_HAS_BUILTIN_SX1262 1
#define BOARD_HAS_OLED          1

// ============================================================================
// SX1262 PIN CONFIGURATION - HELTEC V3 (Fixed, on-board)
// ============================================================================

// SPI Pins (Hardware SPI on Heltec V3)
#define PIN_SX1262_MOSI         10
#define PIN_SX1262_MISO         11
#define PIN_SX1262_SCK          9
#define PIN_SX1262_CS           8

// Control Pins
#define PIN_SX1262_NRESET       12
#define PIN_SX1262_BUSY         13
#define PIN_SX1262_DIO1         14

// RF Switch - Heltec V3 uses DIO2 for automatic RF switch control
#define USE_DIO2_RF_SWITCH      true

// ============================================================================
// OLED DISPLAY PINS (Heltec V3)
// ============================================================================
#define PIN_OLED_SDA            17
#define PIN_OLED_SCL            18
#define PIN_OLED_RST            21

// ============================================================================
// VEXT CONTROL (External power for peripherals)
// ============================================================================
#define PIN_VEXT                36    // Controls power to OLED and external sensors

// ============================================================================
// LED PINS
// ============================================================================
#define PIN_LED                 35    // White LED on Heltec V3

// ============================================================================
// BATTERY MONITORING
// ============================================================================
#define PIN_VBAT_ADC            1     // Battery voltage via voltage divider
#define VBAT_MULTIPLIER         2.0f  // Voltage divider ratio

// ============================================================================
// SPI CONFIGURATION
// ============================================================================
#define SPI_FREQUENCY           8000000   // 8 MHz (safe for all conditions)
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
#define PLATFORM_ESP32S3        1
#define HAS_FREERTOS            1
#define USE_HARDWARE_SPI        1

#endif // CONFIG_H
