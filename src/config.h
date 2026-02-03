/**
 * ============================================================================
 * config.h - SX1262 Driver Configuration with Board Presets
 * ============================================================================
 * SX1262 Driver Suite v2.0
 * 
 * USAGE:
 *   1. Uncomment ONE board preset below (or use BOARD_CUSTOM)
 *   2. For custom boards, define all required pins in the BOARD_CUSTOM section
 *   3. Optionally adjust debug levels and timing parameters
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// BOARD SELECTION - Uncomment exactly ONE option
// ============================================================================

//#define BOARD_HELTEC_WIFI_LORA_32_V3      // Heltec WiFi LoRa 32 V3 (integrated SX1262)
#define BOARD_ESP32S3_WAVESHARE           // ESP32-S3 + Waveshare SX1262 HF module  
//#define BOARD_ESP32_WROOM_WAVESHARE       // ESP32-WROOM + Waveshare SX1262 HF module
//#define BOARD_CUSTOM                        // Custom board - define pins manually below

// ============================================================================
// BOARD PRESET CONFIGURATIONS
// ============================================================================

#if defined(BOARD_HELTEC_WIFI_LORA_32_V3)
// ----------------------------------------------------------------------------
// Heltec WiFi LoRa 32 V3 - Integrated SX1262
// MCU: ESP32-S3
// ----------------------------------------------------------------------------
#define BOARD_NAME                  "Heltec WiFi LoRa 32 V3"
#define BOARD_HAS_BUILTIN_SX1262    1

// SPI Pins
#define PIN_SX1262_MOSI             10
#define PIN_SX1262_MISO             11
#define PIN_SX1262_SCK              9
#define PIN_SX1262_CS               8

// Control Pins
#define PIN_SX1262_NRESET           12
#define PIN_SX1262_BUSY             13
#define PIN_SX1262_DIO1             14

// SPI Host
#define SPI_HOST_ID                 SPI2_HOST

// Board-specific features
#define PIN_VEXT                    36      // Power control for peripherals (active LOW)
#define PIN_LED                     35      // White LED
#define USE_DIO2_RF_SWITCH          true    // DIO2 controls RF switch

// Optional: OLED Display
#define BOARD_HAS_OLED              1
#define PIN_OLED_SDA                17
#define PIN_OLED_SCL                18
#define PIN_OLED_RST                21

// Optional: Battery monitoring
#define PIN_VBAT_ADC                1
#define VBAT_MULTIPLIER             2.0f

// ----------------------------------------------------------------------------
#elif defined(BOARD_ESP32S3_WAVESHARE)
// ----------------------------------------------------------------------------
// ESP32-S3 (diymore N16R8) + Waveshare SX1262 HF Module
// MCU: ESP32-S3
// ----------------------------------------------------------------------------
#define BOARD_NAME                  "ESP32-S3 + Waveshare SX1262"
#define BOARD_HAS_BUILTIN_SX1262    0

// SPI Pins
#define PIN_SX1262_MOSI             11
#define PIN_SX1262_MISO             13
#define PIN_SX1262_SCK              12
#define PIN_SX1262_CS               10

// Control Pins
#define PIN_SX1262_NRESET           16
#define PIN_SX1262_BUSY             5
#define PIN_SX1262_DIO1             2

// SPI Host
#define SPI_HOST_ID                 SPI2_HOST

// Board-specific features
#define USE_DIO2_RF_SWITCH          true    // Waveshare uses DIO2 for RF switch

// ----------------------------------------------------------------------------
#elif defined(BOARD_ESP32_WROOM_WAVESHARE)
// ----------------------------------------------------------------------------
// ESP32 WROOM DevKit + Waveshare SX1262 HF Module
// MCU: ESP32 (Classic)
// ----------------------------------------------------------------------------
#define BOARD_NAME                  "ESP32 WROOM + Waveshare SX1262"
#define BOARD_HAS_BUILTIN_SX1262    0

// SPI Pins (VSPI - default SPI bus on ESP32)
#define PIN_SX1262_MOSI             23
#define PIN_SX1262_MISO             19
#define PIN_SX1262_SCK              18
#define PIN_SX1262_CS               5

// Control Pins
#define PIN_SX1262_NRESET           16
#define PIN_SX1262_BUSY             4
#define PIN_SX1262_DIO1             17

// SPI Host
#define SPI_HOST_ID                 VSPI_HOST

// Board-specific features
#define USE_DIO2_RF_SWITCH          true
#define PIN_LED                     2       // Built-in LED (optional)

// ----------------------------------------------------------------------------
#elif defined(BOARD_CUSTOM)
// ----------------------------------------------------------------------------
// Custom Board - Define all pins manually
// ----------------------------------------------------------------------------
#define BOARD_NAME                  "Custom Board"
#define BOARD_HAS_BUILTIN_SX1262    0

// SPI Pins - REQUIRED: Set these to match your wiring
#define PIN_SX1262_MOSI             11      // ← Change to your MOSI pin
#define PIN_SX1262_MISO             13      // ← Change to your MISO pin
#define PIN_SX1262_SCK              12      // ← Change to your SCK pin
#define PIN_SX1262_CS               10      // ← Change to your CS pin

// Control Pins - REQUIRED: Set these to match your wiring
#define PIN_SX1262_NRESET           16      // ← Change to your RESET pin
#define PIN_SX1262_BUSY             5       // ← Change to your BUSY pin
#define PIN_SX1262_DIO1             2       // ← Change to your DIO1 pin

// SPI Host - Set according to your MCU
// ESP32-S3: SPI2_HOST or SPI3_HOST
// ESP32:    VSPI_HOST or HSPI_HOST
#define SPI_HOST_ID                 SPI2_HOST

// RF Switch Control
#define USE_DIO2_RF_SWITCH          true    // Set false if using manual RF switch

// Optional pins - Uncomment and set if your board has these features
// #define PIN_VEXT                 36      // Power control pin (if applicable)
// #define PIN_LED                  2       // Status LED pin

// ----------------------------------------------------------------------------
#else
    #error "No board selected! Uncomment exactly one BOARD_* define in config.h"
#endif

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================
#ifndef PIN_SX1262_MOSI
    #error "PIN_SX1262_MOSI not defined! Check board configuration."
#endif
#ifndef PIN_SX1262_MISO
    #error "PIN_SX1262_MISO not defined! Check board configuration."
#endif
#ifndef PIN_SX1262_SCK
    #error "PIN_SX1262_SCK not defined! Check board configuration."
#endif
#ifndef PIN_SX1262_CS
    #error "PIN_SX1262_CS not defined! Check board configuration."
#endif
#ifndef PIN_SX1262_NRESET
    #error "PIN_SX1262_NRESET not defined! Check board configuration."
#endif
#ifndef PIN_SX1262_BUSY
    #error "PIN_SX1262_BUSY not defined! Check board configuration."
#endif
#ifndef PIN_SX1262_DIO1
    #error "PIN_SX1262_DIO1 not defined! Check board configuration."
#endif
#ifndef SPI_HOST_ID
    #error "SPI_HOST_ID not defined! Check board configuration."
#endif

// ============================================================================
// SPI CONFIGURATION
// ============================================================================
#ifndef SPI_FREQUENCY_HZ
    #define SPI_FREQUENCY_HZ        8000000     // Changed from 2 MHz (conservative, safe from sleep) to 8 Mhz
#endif
#define SERIAL_BAUD                 115200      // Serial debug baud rate

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
// Reset timing
#define TIMING_RESET_LOW_US         100         // NRESET pulse width (µs)
#define TIMING_RESET_WAIT_MS        5           // Wait after reset release (ms)

// BUSY pin timeout
#define TIMING_BUSY_POLL_TIMEOUT_MS 1000        // General BUSY timeout (ms)

// Mode transition timings (from datasheet, typical values in µs)
#define TIMING_STBY_RC_STBY_XOSC_US 31          // STBY_RC → STBY_XOSC
#define TIMING_STBY_RC_FS_US        50          // STBY_RC → FS
#define TIMING_STBY_RC_TX_US        126         // STBY_RC → TX
#define TIMING_STBY_RC_RX_US        83          // STBY_RC → RX
#define TIMING_STBY_XOSC_FS_US      40          // STBY_XOSC → FS
#define TIMING_STBY_XOSC_RX_US      62          // STBY_XOSC → RX
#define TIMING_STBY_XOSC_TX_US      105         // STBY_XOSC → TX
#define TIMING_SLEEP_COLD_WAKE_MS   3500        // Cold sleep wake time (ms)

// Safety margin for mode transitions
#define TIMING_SAFETY_MARGIN_FACTOR 2.0

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
#define DEBUG_ENABLED               0           // Master debug switch
#define DEBUG_VERBOSE_HAL           0           // Log HAL operations
#define DEBUG_VERBOSE_SPI           0           // Log SPI commands
#define DEBUG_VERBOSE_STATE         0           // Log state transitions

// ============================================================================
// LOGGING SYSTEM
// ============================================================================
// Log Level Constants
#define SX1262_LOG_LEVEL_NONE       0           // No logging (production)
#define SX1262_LOG_LEVEL_ERROR      1           // Errors only
#define SX1262_LOG_LEVEL_WARN       2           // Warnings + errors
#define SX1262_LOG_LEVEL_INFO       3           // Info + warnings + errors
#define SX1262_LOG_LEVEL_DEBUG      4           // Everything (verbose)

// Current Log Level - Uncomment ONE:
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_NONE
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_ERROR
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_WARN
#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_INFO
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_DEBUG

// ----------------------------------------------------------------------------
// Default Log Level (fallback if none selected above)
// ----------------------------------------------------------------------------
#ifndef SX1262_LOG_LEVEL
  #define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_ERROR  // Default: errors only
#endif

// ----------------------------------------------------------------------------
// Log Output Backend (User can override this)
// ----------------------------------------------------------------------------
#ifndef SX1262_LOG_OUTPUT
  #ifdef ARDUINO
    #define SX1262_LOG_OUTPUT(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
  #else
    #include <stdio.h>
    #define SX1262_LOG_OUTPUT(fmt, ...) printf(fmt, ##__VA_ARGS__)
  #endif
#endif

// ----------------------------------------------------------------------------
// Log Macros (Compile to nothing when below threshold)
// ----------------------------------------------------------------------------

// ERROR: Critical failures that prevent operation
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_ERROR
  #define SX1262_LOG_ERROR(fmt, ...) SX1262_LOG_OUTPUT("[ERROR] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_ERROR(fmt, ...)
#endif

// WARN: Non-critical issues that may indicate problems
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_WARN
  #define SX1262_LOG_WARN(fmt, ...) SX1262_LOG_OUTPUT("[WARN] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_WARN(fmt, ...)
#endif

// INFO: General operational information
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_INFO
  #define SX1262_LOG_INFO(fmt, ...) SX1262_LOG_OUTPUT("[INFO] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_INFO(fmt, ...)
#endif

// DEBUG: Detailed trace information
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_DEBUG
  #define SX1262_LOG_DEBUG(fmt, ...) SX1262_LOG_OUTPUT("[DEBUG] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_DEBUG(fmt, ...)
#endif

// ----------------------------------------------------------------------------
// Module-Specific Logging Macros
// ----------------------------------------------------------------------------
// These respect BOTH the log level AND the DEBUG_VERBOSE_* flags
// Module logs are DEBUG level, so they only appear when:
//   1. SX1262_LOG_LEVEL >= DEBUG, AND
//   2. The specific DEBUG_VERBOSE_* flag is enabled (set to 1)

// SPI Protocol Layer
#if DEBUG_VERBOSE_SPI && (SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_DEBUG)
  #define SX1262_LOG_SPI(fmt, ...) SX1262_LOG_OUTPUT("[DEBUG] [SPI] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_SPI(fmt, ...)
#endif

// State Machine
#if DEBUG_VERBOSE_STATE && (SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_DEBUG)
  #define SX1262_LOG_STATE(fmt, ...) SX1262_LOG_OUTPUT("[DEBUG] [STATE] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_STATE(fmt, ...)
#endif

// HAL Layer
#if DEBUG_VERBOSE_HAL && (SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_DEBUG)
  #define SX1262_LOG_HAL(fmt, ...) SX1262_LOG_OUTPUT("[DEBUG] [HAL] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_HAL(fmt, ...)
#endif

// Configuration Layer (respects log level only - no separate flag)
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_DEBUG
  #define SX1262_LOG_CONFIG(fmt, ...) SX1262_LOG_OUTPUT("[DEBUG] [CONFIG] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_CONFIG(fmt, ...)
#endif

// Driver Core (respects log level only - no separate flag)
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_DEBUG
  #define SX1262_LOG_DRIVER(fmt, ...) SX1262_LOG_OUTPUT("[DEBUG] [DRIVER] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_DRIVER(fmt, ...)
#endif

// Errata Workarounds (INFO level - shows unless NONE or ERROR-only)
#if SX1262_LOG_LEVEL >= SX1262_LOG_LEVEL_INFO
  #define SX1262_LOG_ERRATA(fmt, ...) SX1262_LOG_OUTPUT("[INFO] [ERRATA] " fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG_ERRATA(fmt, ...)
#endif

// ----------------------------------------------------------------------------
// Backward Compatibility
// ----------------------------------------------------------------------------
// Simple SX1262_LOG() macro for quick migration from old code
// Respects DEBUG_ENABLED flag for backward compatibility
#if DEBUG_ENABLED
  #define SX1262_LOG(fmt, ...) SX1262_LOG_DEBUG(fmt, ##__VA_ARGS__)
#else
  #define SX1262_LOG(fmt, ...)
#endif

#endif // CONFIG_H