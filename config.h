#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h> // Added to support uint32_t, int8_t, bool types used in structs

// ============ GPIO MAPPING (ESP32-S3 diymore N16R8) ============
// These pins are verified against the provided pinout (diymore ESP32_pinout.pdf)
#define PIN_SX1262_NRESET    16
#define PIN_SX1262_BUSY      5
#define PIN_SX1262_DIO1      2
#define PIN_SX1262_DIO2      6
#define PIN_SX1262_CS        10

// ============ SPI CONFIGURATION ============
#define SPI_FREQUENCY_HZ     2000000  // 2 MHz (conservative, safe from sleep)
// FIX: HSPI_HOST is not defined on ESP32-S3. Use SPI2_HOST or SPI3_HOST.
// SPI2_HOST is a standard secondary SPI bus on ESP32-S3.
#define SPI_HOST             SPI2_HOST  // ESP32-S3 SPI2 (or SPI3_HOST)
#define SERIAL_BAUD          115200     // Baud rate for Serial debug output

// ============ TIMING CONSTANTS (us unless noted) ============
#define TIMING_RESET_LOW_US            100     // NRESET pulse width
#define TIMING_RESET_WAIT_MS           5       // Wait after release for POR calibration
#define TIMING_BUSY_POLL_TIMEOUT_MS    1000    // General busypin timeout (was 100)

// Mode transition timings (from datasheet, typical values)
#define TIMING_STBY_RC_STBY_XOSC_US    31      // STBY_RC -> STBY_XOSC
#define TIMING_STBY_RC_FS_US           50      // STBY_RC -> FS
#define TIMING_STBY_RC_TX_US           126     // STBY_RC -> TX
#define TIMING_STBY_RC_RX_US           83      // STBY_RC -> RX
#define TIMING_STBY_XOSC_FS_US         40      // STBY_XOSC -> FS
#define TIMING_STBY_XOSC_RX_US         62      // STBY_XOSC -> RX
#define TIMING_STBY_XOSC_TX_US         105     // STBY_XOSC -> TX
#define TIMING_SLEEP_COLD_WAKE_MS      3500    // SLEEP cold wake (use 5 ms safe)

// ============ TIMING SAFETY MARGINS ============
// For mode transitions: use typical * TIMING_SAFETY_MARGIN_FACTOR for poll timeout
#define TIMING_SAFETY_MARGIN_FACTOR    2.0

// ============ DEBUG FLAGS ============
#define DEBUG_ENABLED                  0       // Master switch for all debug messages
#define DEBUG_VERBOSE_HAL              0       // Log HAL operations
#define DEBUG_VERBOSE_SPI              0       // Log every SPI command
#define DEBUG_VERBOSE_STATE            0       // Log every state transition

// ============================================================================
// LOGGING SYSTEM
// ============================================================================
// Professional logging abstraction - removes hard dependency on Arduino Serial
// Allows compile-time filtering, custom output redirection, and zero overhead
// when disabled.
//
// Usage:
//   SX1262_LOG_ERROR("Critical failure: %d", error_code);
//   SX1262_LOG_WARN("Unusual condition detected");
//   SX1262_LOG_INFO("Operation complete");
//   SX1262_LOG_DEBUG("Detailed trace: addr=0x%04X", addr);
//
// Configuration:
//   - Set SX1262_LOG_LEVEL below to control verbosity
//   - Override SX1262_LOG_OUTPUT to redirect output
//   - Logs compile to nothing when level is below threshold (zero overhead)
// ============================================================================

// ----------------------------------------------------------------------------
// Log Level Constants (DO NOT COMMENT THESE OUT!)
// ----------------------------------------------------------------------------
#define SX1262_LOG_LEVEL_NONE   0  // No logging (production builds)
#define SX1262_LOG_LEVEL_ERROR  1  // Critical errors only
#define SX1262_LOG_LEVEL_WARN   2  // Warnings and errors
#define SX1262_LOG_LEVEL_INFO   3  // General information + above
#define SX1262_LOG_LEVEL_DEBUG  4  // Everything (verbose)

// ----------------------------------------------------------------------------
// Current Log Level Selection
// ----------------------------------------------------------------------------
// Uncomment exactly ONE of these lines to set the desired log level:

//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_NONE    // Complete silence
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_ERROR   // Errors only
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_WARN    // Warnings + errors
#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_INFO      // Info + warn + error (DEFAULT)
//#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_DEBUG   // Everything (verbose)

// ----------------------------------------------------------------------------
// Default Log Level (fallback if none selected above)
// ----------------------------------------------------------------------------
#ifndef SX1262_LOG_LEVEL
  #define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_ERROR  // Default: errors only
#endif

// ----------------------------------------------------------------------------
// Log Output Backend (User can override this)
// ----------------------------------------------------------------------------
// Default: Use Arduino Serial or standard printf depending on platform
// To redirect logs, define SX1262_LOG_OUTPUT before including this file:
//
// Example - File logging:
//   #define SX1262_LOG_OUTPUT(fmt, ...) fprintf(logfile, fmt, ##__VA_ARGS__)
//
// Example - Network logging:
//   #define SX1262_LOG_OUTPUT(fmt, ...) sendToServer(fmt, ##__VA_ARGS__)
//
// Example - Disable completely:
//   #define SX1262_LOG_OUTPUT(fmt, ...)
// ----------------------------------------------------------------------------
#ifndef SX1262_LOG_OUTPUT
  #ifdef ARDUINO
    // Arduino environment: Use Serial
    #define SX1262_LOG_OUTPUT(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
  #else
    // Standard C environment: Use printf
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
