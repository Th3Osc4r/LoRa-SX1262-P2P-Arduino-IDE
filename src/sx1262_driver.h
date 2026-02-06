#ifndef SX1262_DRIVER_H
#define SX1262_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>          
#include "config.h"            
#include "sx1262_hal.h"
#include "sx1262_spi_protocol.h"
#include "sx1262_regs.h"



// ============================================================================
// RETURN CODES
// ============================================================================
typedef enum {
    SX1262_OK = 0,
    SX1262_ERROR_INVALID_STATE = -1,
    SX1262_ERROR_INVALID_PARAM = -2,
    SX1262_ERROR_TIMEOUT = -3,
    SX1262_ERROR_SPI = -4,
    SX1262_ERROR_BUSY_TIMEOUT = -5,
    SX1262_ERROR_NOT_INITIALIZED = -6,
    SX1262_ERROR_MUTEX = -7,
    SX1262_ERROR_WATCHDOG = -8,
    SX1262_ERROR_HARDWARE = -9
} sx1262_result_t;

// ============================================================================
// RADIO STATES
// ============================================================================
typedef enum {
    SX1262_STATE_UNINITIALIZED = 0,  // Driver not initialized
    SX1262_STATE_SLEEP,              // Sleep mode (lowest power)
    SX1262_STATE_STANDBY_RC,         // Standby with RC oscillator
    SX1262_STATE_STANDBY_XOSC,       // Standby with crystal oscillator
    SX1262_STATE_FS,                 // Frequency synthesis mode
    SX1262_STATE_TX,                 // Transmitting
    SX1262_STATE_RX,                 // Receiving
    SX1262_STATE_CAD,                // Channel Activity Detection
    SX1262_STATE_ERROR               // Error state (needs reset)
} sx1262_state_t;

// ============================================================================
// STATE TRANSITION EVENT (for logging)
// ============================================================================
typedef struct {
    sx1262_state_t from_state;
    sx1262_state_t to_state;
    uint32_t timestamp_ms;
    bool success;
    sx1262_result_t error_code;
} sx1262_state_transition_t;

// ============================================================================
// WATCHDOG CONFIGURATION
// ============================================================================
typedef struct {
    bool enabled;                    // Enable/disable watchdog
    uint8_t consecutive_failures;    // Count of consecutive failures
    uint8_t max_failures;            // Max failures before emergency reset (default 3)
} sx1262_watchdog_t;

// ============================================================================
// PHASE 3.1: POWER MANAGEMENT TYPES
// ============================================================================

/**
 * Sleep Mode Selection
 * 
 * The SX1262 supports two sleep modes with different power/wake tradeoffs:
 * 
 * COLD START (SX1262_SLEEP_COLD):
 *   - Current: ~160 nA (lowest possible)
 *   - Wake time: ~3.5 ms (full re-initialization required)
 *   - Config: Lost on wake - must call sx1262_config_lora() again
 *   - Use case: Long sleep periods (hours/days), battery critical
 * 
 * WARM START (SX1262_SLEEP_WARM):
 *   - Current: ~600 nA (still very low)
 *   - Wake time: ~340 µs (fast resume)
 *   - Config: Retained in chip's retention memory
 *   - Use case: Duty-cycled operation, frequent wake (seconds/minutes)
 * 
 * For railway monitoring with 15-minute intervals, WARM start is recommended
 * as it allows immediate TX after wake without reconfiguration.
 */
typedef enum {
    SX1262_SLEEP_COLD = 0x00,  // All blocks off, config lost, 160 nA, 3.5ms wake
    SX1262_SLEEP_WARM = 0x04   // Config retained, 600 nA, 340µs wake
} sx1262_sleep_mode_t;

/**
 * Power State Tracking Structure
 * 
 * Tracks sleep statistics for power consumption analysis and debugging.
 */
typedef struct {
    bool is_sleeping;                 // True if chip is currently in sleep mode
    sx1262_sleep_mode_t sleep_mode;   // Current/last sleep mode used
    uint32_t sleep_enter_time_ms;     // Timestamp when sleep was entered
    uint32_t total_sleep_time_ms;     // Cumulative sleep time across all cycles
    uint32_t wake_count;              // Number of wake cycles completed
} sx1262_power_state_t;

// ============================================================================
// PHASE 1.2: LORA CONFIGURATION TYPES
// ============================================================================

/**
 * LoRa Spreading Factor (SF5-SF12)
 */
typedef enum {
    SX1262_LORA_SF5  = 5,
    SX1262_LORA_SF6  = 6,
    SX1262_LORA_SF7  = 7,   // Default - balanced
    SX1262_LORA_SF8  = 8,
    SX1262_LORA_SF9  = 9,
    SX1262_LORA_SF10 = 10,
    SX1262_LORA_SF11 = 11,
    SX1262_LORA_SF12 = 12   // Maximum range
} sx1262_lora_sf_t;

/**
 * LoRa Bandwidth
 * * Values from SX1262 datasheet Table 13-48 (SetModulationParams LoRa BW).
 * NOTE: The values are NOT sequential! This matches the hardware register values.
 */
typedef enum {
    SX1262_LORA_BW_7P8   = 0x00,  // 7.81 kHz
    SX1262_LORA_BW_10P4  = 0x08,  // 10.42 kHz
    SX1262_LORA_BW_15P6  = 0x01,  // 15.63 kHz
    SX1262_LORA_BW_20P8  = 0x09,  // 20.83 kHz
    SX1262_LORA_BW_31P25 = 0x02,  // 31.25 kHz
    SX1262_LORA_BW_41P7  = 0x0A,  // 41.67 kHz
    SX1262_LORA_BW_62P5  = 0x03,  // 62.5 kHz
    SX1262_LORA_BW_125   = 0x04,  // 125 kHz (Default - most common)
    SX1262_LORA_BW_250   = 0x05,  // 250 kHz
    SX1262_LORA_BW_500   = 0x06   // 500 kHz
} sx1262_lora_bw_t;

/**
 * LoRa Coding Rate
 */
typedef enum {
    SX1262_LORA_CR_4_5 = 1,  // Default
    SX1262_LORA_CR_4_6 = 2,
    SX1262_LORA_CR_4_7 = 3,
    SX1262_LORA_CR_4_8 = 4
} sx1262_lora_cr_t;

/**
 * LoRa Header Mode
 */
typedef enum {
    SX1262_LORA_HEADER_EXPLICIT = 0,  // Variable length (default)
    SX1262_LORA_HEADER_IMPLICIT = 1   // Fixed length
} sx1262_lora_header_t;

/**
 * LoRa CRC Mode
 */
typedef enum {
    SX1262_LORA_CRC_OFF = 0,
    SX1262_LORA_CRC_ON  = 1   // Default
} sx1262_lora_crc_t;

/**
 * LoRa IQ Polarity
 */
typedef enum {
    SX1262_LORA_IQ_NORMAL   = 0,  // Default
    SX1262_LORA_IQ_INVERTED = 1
} sx1262_lora_iq_t;

/**
 * TX Ramp Time
 */
typedef enum {
    SX1262_RAMP_10U   = 0x00,
    SX1262_RAMP_20U   = 0x01,
    SX1262_RAMP_40U   = 0x02,  // Default
    SX1262_RAMP_80U   = 0x03,
    SX1262_RAMP_200U  = 0x04,
    SX1262_RAMP_800U  = 0x05,
    SX1262_RAMP_1700U = 0x06,
    SX1262_RAMP_3400U = 0x07
} sx1262_ramp_time_t;

/**
 * Complete LoRa Configuration Structure
 */
typedef struct {
    // RF Parameters
    uint32_t frequency_hz;
    int8_t tx_power_dbm;
    sx1262_ramp_time_t ramp_time;
    
    // LoRa Modulation
    sx1262_lora_sf_t spreading_factor;
    sx1262_lora_bw_t bandwidth;
    sx1262_lora_cr_t coding_rate;
    bool low_data_rate_optimize;
    
    // LoRa Packet
    uint16_t preamble_length;
    sx1262_lora_header_t header_type;
    uint8_t payload_length;
    sx1262_lora_crc_t crc_type;
    sx1262_lora_iq_t invert_iq;
    
    // Sync Word
    uint16_t sync_word;
    
    // Internal
    bool configured;
} sx1262_lora_config_t;

// ============================================================================
// DRIVER CONTEXT (singleton instance)
// ============================================================================
typedef struct {
    // State management
    sx1262_state_t current_state;
    sx1262_state_t previous_state;
    uint32_t state_entry_time_ms;
    
    // Thread safety (FreeRTOS)
    void* state_mutex;               // SemaphoreHandle_t cast to void*
    bool mutex_initialized;
    void* blocking_task_handle;      // Phase 3: Task handle waiting for IRQ
    
    // Watchdog
    sx1262_watchdog_t watchdog;
    
    // Hardware status
    bool hardware_initialized;
    bool spi_initialized;
    uint8_t chip_mode;               // Last known chip mode (from status byte)
    
    // State transition history (circular buffer for debugging)
    sx1262_state_transition_t transition_history[16];
    uint8_t transition_history_idx;
    
    // Statistics
    uint32_t total_state_transitions;
    uint32_t failed_state_transitions;
    uint32_t watchdog_resets;
    
    // Error tracking
    sx1262_result_t last_error;
    const char* last_error_function;
    uint32_t last_error_timestamp_ms;
    
    // Radio configuration (Phase 1.2)
    sx1262_lora_config_t config;
    bool config_valid;
	bool radio_configured;
    
    // Power management (Phase 3.1)
    sx1262_power_state_t power;
    
} sx1262_driver_context_t;

// ============================================================================
// INITIALIZATION & DEINITIALIZATION
// ============================================================================

/**
 * Initialize the SX1262 driver
 * * This function initializes the HAL layer, SPI, performs hardware reset,
 * sets up the state machine, creates mutexes, and initializes the watchdog.
 * * @return SX1262_OK on success, error code on failure
 */
sx1262_result_t sx1262_driver_init();

/**
 * Deinitialize the driver and release resources
 * Puts radio in sleep mode, destroys mutexes, cleans up
 * * @return SX1262_OK on success
 */
sx1262_result_t sx1262_driver_deinit();

/**
 * Check if driver is initialized
 * * @return true if initialized, false otherwise
 */
bool sx1262_driver_is_initialized();

// ============================================================================
// SIMPLIFIED INITIALIZATION API
// ============================================================================
// These functions combine driver initialization and radio configuration into
// single calls, making it easier to get started with the SX1262 driver.
// Use these for typical applications; use the granular API for advanced control.
// ============================================================================

/**
 * Initialize SX1262 driver with common parameters (simplified API)
 * * This function combines driver initialization, hardware setup, and radio
 * configuration into a single call. It uses sensible defaults for most
 * parameters while allowing customization of the most commonly changed values.
 * * Default values applied:
 * - Spreading Factor: SF7 (balanced range/speed)
 * - Bandwidth: 125 kHz (standard LoRa)
 * - Coding Rate: 4/5 (minimal overhead)
 * - Preamble: 8 symbols
 * - Header: Explicit (variable length packets)
 * - CRC: Enabled
 * - Sync Word: 0x1424 (private network)
 * - Ramp Time: 40 µs
 * - LDRO: Auto-detected based on SF/BW
 * * After successful return, the driver is ready for sx1262_transmit() and
 * sx1262_receive() operations.
 * * @param frequency_hz  RF frequency in Hz (e.g., 868100000 for 868.1 MHz)
 * Valid range: 150,000,000 - 960,000,000 Hz
 * @param tx_power_dbm  Transmit power in dBm
 * Valid range: -9 to +22 dBm
 * * @return SX1262_OK on success, error code on failure
 * Call sx1262_get_init_error_help() for diagnostic guidance on failure
 * * Example:
 * @code
 * // Initialize for EU868 at +14 dBm
 * sx1262_result_t res = sx1262_init_simple(868100000, 14);
 * if (res == SX1262_OK) {
 * Serial.println("Radio ready!");
 * // Now call sx1262_transmit() or sx1262_receive()
 * } else {
 * char help[128];
 * sx1262_get_init_error_help(res, help, sizeof(help));
 * Serial.println(help);
 * }
 * @endcode
 */
sx1262_result_t sx1262_init_simple(uint32_t frequency_hz, int8_t tx_power_dbm);

/**
 * Initialize SX1262 driver with extended parameters
 * * Provides control over spreading factor and bandwidth while using
 * sensible defaults for other parameters. Use this when you need to
 * optimize for range (higher SF) or speed (lower SF, higher BW).
 * * LDRO (Low Data Rate Optimization) is automatically enabled when required
 * based on the SF/BW combination.
 * * @param frequency_hz  RF frequency in Hz (150,000,000 - 960,000,000)
 * @param tx_power_dbm  Transmit power in dBm (-9 to +22)
 * @param sf            Spreading factor (SX1262_LORA_SF5 to SX1262_LORA_SF12)
 * SF7 is recommended for most applications
 * SF12 provides maximum range but slowest data rate
 * @param bw            Bandwidth (SX1262_LORA_BW_125 recommended)
 * Higher BW = faster but shorter range
 * * @return SX1262_OK on success, error code on failure
 * * Example:
 * @code
 * // Initialize for long range with SF10, 125kHz BW
 * sx1262_result_t res = sx1262_init_extended(
 * 868100000,           // 868.1 MHz
 * 22,                  // +22 dBm (max power)
 * SX1262_LORA_SF10,    // SF10 for better range
 * SX1262_LORA_BW_125   // 125 kHz bandwidth
 * );
 * @endcode
 */
sx1262_result_t sx1262_init_extended(
    uint32_t frequency_hz, 
    int8_t tx_power_dbm,
    sx1262_lora_sf_t sf,
    sx1262_lora_bw_t bw
);

/**
 * Get human-readable description of initialization failure
 * * Call this after sx1262_init_simple() or sx1262_init_extended() returns
 * an error to get diagnostic guidance for troubleshooting.
 * * @param error   Error code returned by init function
 * @param buffer  Buffer to receive description (min 128 bytes recommended)
 * @param len     Buffer length
 * * Example:
 * @code
 * sx1262_result_t res = sx1262_init_simple(868100000, 14);
 * if (res != SX1262_OK) {
 * char help[128];
 * sx1262_get_init_error_help(res, help, sizeof(help));
 * Serial.printf("Init failed: %s\n", help);
 * }
 * @endcode
 */
void sx1262_get_init_error_help(sx1262_result_t error, char* buffer, size_t len);

// ============================================================================
// HARDWARE SELF-TEST
// ============================================================================
// Run before radio operations to verify hardware connectivity and function.
// This helps diagnose wiring issues, damaged modules, or configuration problems.
// ============================================================================

/**
 * Self-test result structure
 * * Contains detailed results from each test phase, allowing users to
 * identify exactly which hardware component is failing.
 */
typedef struct {
    // Overall result
    bool passed;                    ///< true if ALL tests passed
    
    // Individual test results
    bool busy_pin_readable;         ///< BUSY pin GPIO can be read
    bool busy_pin_responsive;       ///< BUSY pin responds to chip operations
    bool spi_communication;         ///< SPI send/receive works
    bool chip_responding;           ///< Chip responds to GetStatus command
    bool chip_id_valid;             ///< Status byte format matches SX126x
    bool reset_working;             ///< NRST pin successfully resets chip
    bool dio1_pin_readable;         ///< DIO1 pin GPIO can be read
    bool standby_command;           ///< SetStandby command executes correctly
    
    // Diagnostic data
    uint8_t status_byte;            ///< Raw status byte from GetStatus
    uint8_t chip_mode;              ///< Chip mode extracted from status (2=STDBY_RC, 3=STDBY_XOSC)
    uint8_t command_status;         ///< Command status from status byte (should be 0x02 or 0x03)
    uint32_t busy_response_us;      ///< Time for BUSY to respond after reset (microseconds)
    
    // Failure information
    char failure_reason[64];        ///< Human-readable description of first failure
} sx1262_self_test_result_t;

/**
 * Perform hardware self-test
 * * This function performs a comprehensive test of the SX1262 hardware
 * connections WITHOUT initializing the full driver. Use this to verify
 * wiring before attempting radio operations.
 * * Tests performed (in order):
 * 1. BUSY pin read test - verify GPIO input works
 * 2. SPI communication test - send GetStatus, verify response
 * 3. Chip identity test - verify status byte format is valid
 * 4. Hardware reset test - toggle NRST, verify BUSY responds
 * 5. DIO1 pin read test - verify GPIO input works
 * 6. SetStandby test - verify chip accepts commands
 * * The test stops at the first failure and reports which test failed.
 * * IMPORTANT: This function performs low-level hardware access and should
 * be called BEFORE sx1262_driver_init() or sx1262_init_simple(). If the
 * driver is already initialized, call sx1262_driver_deinit() first.
 * * @param result  Pointer to result structure (required, cannot be NULL)
 * @return SX1262_OK if all tests pass, SX1262_ERROR_HARDWARE if any test fails,
 * SX1262_ERROR_INVALID_PARAM if result is NULL
 * * Example:
 * @code
 * sx1262_self_test_result_t test;
 * * if (sx1262_self_test(&test) == SX1262_OK) {
 * Serial.println("Hardware OK - proceeding with initialization");
 * sx1262_init_simple(868100000, 14);
 * } else {
 * Serial.printf("Hardware test FAILED: %s\n", test.failure_reason);
 * Serial.printf("  BUSY pin: %s\n", test.busy_pin_readable ? "OK" : "FAIL");
 * Serial.printf("  SPI:      %s\n", test.spi_communication ? "OK" : "FAIL");
 * Serial.printf("  Chip ID:  %s\n", test.chip_id_valid ? "OK" : "FAIL");
 * Serial.printf("  Reset:    %s\n", test.reset_working ? "OK" : "FAIL");
 * }
 * @endcode
 */
sx1262_result_t sx1262_self_test(sx1262_self_test_result_t* result);

/**
 * Print self-test results to log output
 * * Convenience function to display all test results in a formatted table.
 * Uses the logging system (SX1262_LOG_INFO), so output respects log level.
 * * @param result  Pointer to completed test result structure
 */
void sx1262_print_self_test_result(const sx1262_self_test_result_t* result);

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

/**
 * Get current radio state (thread-safe)
 * * @return Current state
 */
sx1262_state_t sx1262_get_state();

/**
 * Get state as human-readable string
 * * @param state State to convert
 * @return String representation
 */
const char* sx1262_state_to_string(sx1262_state_t state);

/**
 * Request state transition (thread-safe)
 * * Validates the transition, executes hardware commands,
 * updates state machine, logs transition, and updates watchdog.
 * * @param new_state Desired state
 * @return SX1262_OK on success, error code on failure
 */
sx1262_result_t sx1262_set_state(sx1262_state_t new_state);

/**
 * Check if state transition is valid
 * * Not all state transitions are valid. This function checks
 * the state machine transition rules.
 * * @param from_state Current state
 * @param to_state Desired state
 * @return true if transition is valid, false otherwise
 */
bool sx1262_is_valid_transition(sx1262_state_t from_state, sx1262_state_t to_state);

/**
 * Get time spent in current state
 * * @return Time in milliseconds since entering current state
 */
uint32_t sx1262_get_time_in_state();

/**
 * Get state transition history (for debugging)
 * * @param history Array to fill with history (min 16 entries)
 * @param count Number of entries to retrieve (max 16)
 * @return Number of entries actually retrieved
 */
uint8_t sx1262_get_transition_history(sx1262_state_transition_t* history, uint8_t count);

// ============================================================================
// RESET OPERATIONS
// ============================================================================

/**
 * Perform hardware reset
 * * Toggles NRESET pin, waits for chip to boot,
 * resets state machine to STANDBY_RC
 * * @return SX1262_OK on success
 */
sx1262_result_t sx1262_reset();

/**
 * Perform software reset
 * * Sends sleep command, then wakes up - equivalent to reset
 * but without toggling NRESET pin
 * * @return SX1262_OK on success
 */
sx1262_result_t sx1262_soft_reset();

/**
 * Emergency reset (called by watchdog or on critical error)
 * * Performs hardware reset, clears all state, resets statistics,
 * re-initializes the driver
 * * @return SX1262_OK on success
 */
sx1262_result_t sx1262_emergency_reset();

// ============================================================================
// WATCHDOG FUNCTIONS
// ============================================================================

/**
 * Initialize watchdog with configuration
 * * @param max_failures Maximum consecutive failures before emergency reset
 * @return SX1262_OK on success
 */
sx1262_result_t sx1262_watchdog_init(uint8_t max_failures);

/**
 * Enable or disable watchdog
 * * @param enable true to enable, false to disable
 */
void sx1262_watchdog_enable(bool enable);

/**
 * Feed the watchdog (clear failure counter)
 * * Should be called after every successful operation.
 * Clears the consecutive failure counter.
 */
void sx1262_watchdog_feed();

/**
 * Get watchdog statistics
 * * @param consecutive_failures Number of consecutive failures
 * @return true if watchdog is enabled
 */
bool sx1262_watchdog_get_status(uint8_t* consecutive_failures);

// ============================================================================
// THREAD SAFETY (MUTEX OPERATIONS)
// ============================================================================

/**
 * Lock the state mutex (internal use)
 * Blocks until mutex is available
 * * @param timeout_ms Maximum time to wait for mutex (0 = wait forever)
 * @return SX1262_OK on success, SX1262_ERROR_MUTEX on timeout
 */
sx1262_result_t sx1262_mutex_lock(uint32_t timeout_ms);

/**
 * Unlock the state mutex (internal use)
 */
void sx1262_mutex_unlock();

// ============================================================================
// ERROR HANDLING
// ============================================================================

/**
 * Get last error information
 * * @param error Pointer to store error code
 * @param function Pointer to store function name where error occurred
 * @param timestamp_ms Pointer to store error timestamp
 */
void sx1262_get_last_error(
    sx1262_result_t* error,
    const char** function,
    uint32_t* timestamp_ms
);

/**
 * Clear last error
 */
void sx1262_clear_error();

/**
 * Convert error code to human-readable string
 * * @param error Error code
 * @return String representation
 */
const char* sx1262_error_to_string(sx1262_result_t error);

// ============================================================================
// STATISTICS & DIAGNOSTICS
// ============================================================================

/**
 * Get driver statistics
 */
typedef struct {
    sx1262_state_t current_state;
    uint32_t time_in_state_ms;
    uint32_t total_transitions;
    uint32_t failed_transitions;
    uint32_t watchdog_resets;
    uint32_t uptime_ms;
    sx1262_result_t last_error;
    uint32_t time_since_last_error_ms;
} sx1262_statistics_t;

void sx1262_get_statistics(sx1262_statistics_t* stats);

/**
 * Reset statistics counters (not state machine)
 */
void sx1262_reset_statistics();

/**
 * Print current driver status (for debugging)
 */
void sx1262_print_status();

/**
 * Print state transition history (for debugging)
 */
void sx1262_print_transition_history();

// ============================================================================
// PHASE 1.2: RADIO CONFIGURATION API
// ============================================================================

/**
 * Initialize configuration with default values
 */
void sx1262_config_init_defaults(sx1262_lora_config_t* config);

/**
 * Apply complete LoRa configuration
 */
sx1262_result_t sx1262_config_lora(const sx1262_lora_config_t* config);

/**
 * Set RF frequency
 */
sx1262_result_t sx1262_set_frequency(uint32_t frequency_hz);

/**
 * Set TX power
 */
sx1262_result_t sx1262_set_tx_power(int8_t power_dbm, sx1262_ramp_time_t ramp_time);

/**
 * Set modulation parameters
 */
sx1262_result_t sx1262_set_modulation_params(
    sx1262_lora_sf_t sf,
    sx1262_lora_bw_t bw,
    sx1262_lora_cr_t cr,
    bool low_data_rate_optimize
);

/**
 * Set packet parameters
 */
sx1262_result_t sx1262_set_packet_params(
    uint16_t preamble_length,
    sx1262_lora_header_t header_type,
    uint8_t payload_length,
    sx1262_lora_crc_t crc_type,
    sx1262_lora_iq_t invert_iq
);

/**
 * Set sync word
 */
sx1262_result_t sx1262_set_sync_word(uint16_t sync_word);

/**
 * Get current configuration
 */
sx1262_result_t sx1262_get_config(sx1262_lora_config_t* config);

/**
 * Validate configuration
 */
sx1262_result_t sx1262_validate_config(const sx1262_lora_config_t* config);

/**
 * Print configuration (debug)
 */
void sx1262_print_config();

/**
 * Get bandwidth in Hz
 */
uint32_t sx1262_get_bandwidth_hz(sx1262_lora_bw_t bw);

/**
 * Get time-on-air in milliseconds
 */
uint32_t sx1262_get_time_on_air_ms(const sx1262_lora_config_t* config, uint8_t payload_length);

/**
 * Check if LDRO is required
 */
bool sx1262_is_ldro_required(sx1262_lora_sf_t sf, sx1262_lora_bw_t bw);

// ============================================================================
// PHASE 2.1: TX/RX OPERATIONS
// ============================================================================

/**
 * TX operation result structure
 * Contains detailed information about transmission for diagnostics and optimization
 */
typedef struct {
    bool success;                   // Did transmission complete successfully?
    uint32_t tx_start_ms;          // Timestamp when TX started
    uint32_t tx_done_ms;           // Timestamp when TX completed
    uint32_t tx_duration_ms;       // Actual TX duration (tx_done - tx_start)
    uint32_t calculated_toa_ms;    // Calculated Time-on-Air from configuration
    uint32_t timeout_used_ms;      // Actual timeout used (max of user timeout and ToA-based)
    bool timed_out;                // Did TX timeout occur (IRQ_RX_TX_TIMEOUT)?
    uint16_t irq_flags;            // IRQ status flags at completion
    sx1262_result_t error_code;    // Detailed error code if failed
} sx1262_tx_result_t;

/**
 * RX operation result structure
 * Contains detailed information about reception for diagnostics and link quality assessment
 */
typedef struct {
    bool success;                   // Did reception complete successfully?
    uint32_t rx_start_ms;          // Timestamp when RX started
    uint32_t rx_done_ms;           // Timestamp when RX completed
    uint32_t rx_duration_ms;       // Actual RX duration (rx_done - rx_start)
    uint32_t timeout_used_ms;      // Timeout value used for RX window
    bool timed_out;                // Did RX timeout occur (IRQ_RX_TX_TIMEOUT)?
    bool crc_error;                // CRC error detected (IRQ_CRC_ERROR)?
    uint8_t payload_length;        // Received payload length (0-255)
    uint8_t rx_buffer_offset;      // Where payload was stored in RX buffer
    int16_t rssi_pkt;              // Packet RSSI (raw value, -157 to 0 dBm typical)
    int8_t snr_pkt;                // Packet SNR (raw value, -20 to +10 dB typical)
    int16_t signal_rssi_pkt;       // Signal RSSI during reception
    uint16_t irq_flags;            // IRQ status flags at completion
    sx1262_result_t error_code;    // Detailed error code if failed
} sx1262_rx_result_t;

/**
 * Transmit a LoRa packet (blocking mode with polling)
 * * This function performs a complete transmission cycle:
 * 1. Validates state and parameters
 * 2. Calculates Time-on-Air (ToA) for timeout safety
 * 3. Writes payload to TX buffer
 * 4. Clears pending IRQs
 * 5. Configures DIO1 for TX_DONE interrupt
 * 6. Enters TX mode with calculated timeout
 * 7. Polls IRQ status until TX_DONE or timeout
 * 8. Returns to STANDBY_RC (fast path for RX transition)
 * * The function uses intelligent timeout calculation based on packet
 * parameters to prevent spurious TX timeouts. The actual timeout used
 * is the maximum of:
 * - User-specified timeout_ms
 * - Calculated ToA + 50% safety margin
 * * Blocking mode: Function does not return until TX completes or times out.
 * Thread-safe: Uses mutex protection for state machine access.
 * * @param payload Pointer to data buffer to transmit (1-255 bytes)
 * @param length Payload length in bytes (must be 1-255)
 * @param timeout_ms User timeout in milliseconds (0 = use ToA-based timeout only)
 * @param result Optional pointer to result structure (can be NULL if not needed)
 * @return SX1262_OK on success, error code on failure
 * * Error codes:
 * - SX1262_ERROR_INVALID_PARAM: Invalid payload or length
 * - SX1262_ERROR_NOT_INITIALIZED: Driver not initialized
 * - SX1262_ERROR_INVALID_STATE: Not in STANDBY mode
 * - SX1262_ERROR_SPI: SPI communication error
 * - SX1262_ERROR_TIMEOUT: TX timed out (check ToA calculation)
 * - SX1262_ERROR_HARDWARE: No IRQ fired within timeout
 * * Example:
 * @code
 * uint8_t data[] = "Hello LoRa!";
 * sx1262_tx_result_t result;
 * * sx1262_result_t res = sx1262_transmit(data, sizeof(data), 5000, &result);
 * if (res == SX1262_OK) {
 * printf("TX Success! Duration: %lu ms\n", result.tx_duration_ms);
 * }
 * @endcode
 */

// ============================================================================
// PHASE 2.2: RX OPERATIONS
// ============================================================================

/**
 * Receive a LoRa packet (blocking mode with polling)
 * * This function performs a complete reception cycle:
 * 1. Validates state and parameters
 * 2. Sets buffer base addresses (TX=0, RX=128)
 * 3. Clears pending IRQs
 * 4. Configures DIO1 for RX_DONE interrupt
 * 5. Enters RX mode with specified timeout
 * 6. Polls IRQ status until RX_DONE, CRC_ERROR, or timeout
 * 7. Extracts payload from RX buffer
 * 8. Measures RSSI/SNR signal quality
 * 9. Returns to STANDBY_RC (mirrors TX behavior)
 * * Blocking mode: Function does not return until RX completes or times out.
 * Thread-safe: Uses mutex protection for state machine access.
 * * @param payload Buffer to store received payload (must be 1-255 bytes)
 * @param max_length Maximum buffer size (safety check, typically 255)
 * @param timeout_ms RX timeout in milliseconds (0 = continuous RX, not recommended)
 * @param result Optional pointer to result structure (can be NULL if not needed)
 * @return SX1262_OK on success, error code on failure
 * * Error codes:
 * - SX1262_ERROR_INVALID_PARAM: Invalid buffer or max_length
 * - SX1262_ERROR_NOT_INITIALIZED: Driver not initialized
 * - SX1262_ERROR_INVALID_STATE: Not in STANDBY mode
 * - SX1262_ERROR_SPI: SPI communication error
 * - SX1262_ERROR_TIMEOUT: RX timed out (no packet received)
 * - SX1262_ERROR_HARDWARE: CRC error detected (production mode)
 * * Note: CRC error behavior controlled by SX1262_DEBUG_CRC_ERRORS:
 * - If defined: Payload extracted, result->crc_error = true, returns SX1262_OK
 * - If not defined: No payload extracted, returns SX1262_ERROR_HARDWARE
 * * Example:
 * @code
 * uint8_t rx_buffer[255];
 * sx1262_rx_result_t result;
 * * sx1262_result_t res = sx1262_receive(rx_buffer, 255, 5000, &result);
 * if (res == SX1262_OK && result.success) {
 * printf("RX Success! Length: %d, RSSI: %d dBm\n", 
 * result.payload_length, result.rssi_pkt / 2);
 * }
 * @endcode
 */
sx1262_result_t sx1262_receive(
    uint8_t* payload,
    uint8_t max_length,
    uint32_t timeout_ms,
    sx1262_rx_result_t* result
);

// ============================================================================
// PHASE 2.1: TX OPERATIONS
// ============================================================================

sx1262_result_t sx1262_transmit(
    const uint8_t* payload,
    uint8_t length,
    uint32_t timeout_ms,
    sx1262_tx_result_t* result
);

// ============================================================================
// PHASE 3.1: TX-RX TRANSITION FUNCTIONS
// ============================================================================
// These functions simplify the common pattern of switching between TX and RX
// modes, ensuring proper state machine transitions and timing.
// ============================================================================

/**
 * @brief Prepare radio for RX mode
 * 
 * Ensures the radio is in a clean state ready for reception:
 * - Returns to STANDBY_RC if in TX/RX/other active state
 * - Clears any pending IRQs
 * - Does NOT enter RX mode (call sx1262_receive() after this)
 * 
 * Use this when you need to ensure clean state before RX, or when
 * implementing custom RX logic.
 * 
 * Thread-safe: Uses mutex protection.
 * Blocking: May block briefly during state transition.
 * 
 * @return SX1262_OK on success
 * @return SX1262_ERROR_NOT_INITIALIZED if driver not initialized
 * @return SX1262_ERROR_MUTEX if mutex acquisition failed
 * @return SX1262_ERROR_HARDWARE if state transition failed
 * 
 * Example:
 * @code
 *   // After TX completes, prepare for RX
 *   if (sx1262_prepare_for_rx() == SX1262_OK) {
 *       sx1262_receive(buffer, 255, 5000, &rx_result);
 *   }
 * @endcode
 */
sx1262_result_t sx1262_prepare_for_rx(void);

/**
 * @brief Prepare radio for TX mode
 * 
 * Ensures the radio is in a clean state ready for transmission:
 * - Returns to STANDBY_RC if in TX/RX/other active state
 * - Clears any pending IRQs
 * - Does NOT enter TX mode (call sx1262_transmit() after this)
 * 
 * Use this when you need to ensure clean state before TX, or when
 * implementing custom TX logic.
 * 
 * Thread-safe: Uses mutex protection.
 * Blocking: May block briefly during state transition.
 * 
 * @return SX1262_OK on success
 * @return SX1262_ERROR_NOT_INITIALIZED if driver not initialized
 * @return SX1262_ERROR_MUTEX if mutex acquisition failed
 * @return SX1262_ERROR_HARDWARE if state transition failed
 * 
 * Example:
 * @code
 *   // After RX completes, prepare for TX response
 *   if (sx1262_prepare_for_tx() == SX1262_OK) {
 *       sx1262_transmit(ack_data, ack_len, 0, &tx_result);
 *   }
 * @endcode
 */
sx1262_result_t sx1262_prepare_for_tx(void);

/**
 * @brief Fast turnaround from TX to RX
 * 
 * Convenience function that combines state preparation with reception.
 * Ensures correct sequencing: TX -> STANDBY -> RX
 * 
 * This function:
 * 1. Ensures radio is in STANDBY state
 * 2. Clears pending IRQs
 * 3. Calls sx1262_receive() with provided parameters
 * 
 * Typical use case: Gateway responding to sensor after TX, or sensor
 * waiting for ACK after transmitting data.
 * 
 * Thread-safe: Uses mutex protection.
 * Blocking: Blocks until RX completes or times out.
 * 
 * @param rx_payload Buffer to store received data
 * @param max_length Maximum buffer size (1-255)
 * @param timeout_ms RX timeout in milliseconds
 * @param result Optional pointer to RX result structure (can be NULL)
 * @return SX1262_OK on successful reception
 * @return SX1262_ERROR_TIMEOUT if no packet received within timeout
 * @return SX1262_ERROR_HARDWARE if CRC error detected
 * @return Other error codes as per sx1262_receive()
 * 
 * Example:
 * @code
 *   // Transmit sensor data
 *   sx1262_transmit(sensor_data, len, 0, NULL);
 *   
 *   // Wait for gateway ACK (500ms window)
 *   sx1262_rx_result_t rx_result;
 *   if (sx1262_turnaround_tx_to_rx(ack_buf, 32, 500, &rx_result) == SX1262_OK) {
 *       // ACK received, process response
 *   }
 * @endcode
 */
sx1262_result_t sx1262_turnaround_tx_to_rx(
    uint8_t* rx_payload,
    uint8_t max_length,
    uint32_t timeout_ms,
    sx1262_rx_result_t* result
);

/**
 * @brief Fast turnaround from RX to TX
 * 
 * Convenience function that combines state preparation with transmission.
 * Ensures correct sequencing: RX -> STANDBY -> TX
 * 
 * This function:
 * 1. Ensures radio is in STANDBY state
 * 2. Clears pending IRQs
 * 3. Calls sx1262_transmit() with provided parameters
 * 
 * Typical use case: Gateway sending ACK/response immediately after
 * receiving sensor data.
 * 
 * Thread-safe: Uses mutex protection.
 * Blocking: Blocks until TX completes or times out.
 * 
 * @param tx_payload Data to transmit
 * @param length Payload length (1-255)
 * @param timeout_ms TX timeout (0 = use ToA-based timeout)
 * @param result Optional pointer to TX result structure (can be NULL)
 * @return SX1262_OK on successful transmission
 * @return SX1262_ERROR_TIMEOUT if TX timed out
 * @return Other error codes as per sx1262_transmit()
 * 
 * Example:
 * @code
 *   // Receive sensor data
 *   sx1262_rx_result_t rx_result;
 *   if (sx1262_receive(rx_buf, 255, 10000, &rx_result) == SX1262_OK) {
 *       // Send ACK immediately
 *       uint8_t ack[] = {0x06, rx_result.payload_length};
 *       sx1262_turnaround_rx_to_tx(ack, sizeof(ack), 0, NULL);
 *   }
 * @endcode
 */
sx1262_result_t sx1262_turnaround_rx_to_tx(
    const uint8_t* tx_payload,
    uint8_t length,
    uint32_t timeout_ms,
    sx1262_tx_result_t* result
);

// ============================================================================
// PHASE 3.1: POWER MANAGEMENT FUNCTIONS
// ============================================================================

/**
 * @brief Enter sleep mode for power conservation
 * 
 * Puts the SX1262 into low-power sleep mode. Two modes are available:
 * 
 * SX1262_SLEEP_WARM (recommended for most applications):
 *   - Power: ~600 nA
 *   - Wake time: ~340 µs
 *   - Configuration retained in chip memory
 *   - Can TX/RX immediately after wake
 * 
 * SX1262_SLEEP_COLD (maximum power savings):
 *   - Power: ~160 nA  
 *   - Wake time: ~3.5 ms
 *   - Configuration lost - must reconfigure after wake
 *   - Call sx1262_config_lora() after wake before TX/RX
 * 
 * Thread-safe: Uses mutex protection.
 * 
 * @param mode Sleep mode selection (WARM or COLD)
 * @return SX1262_OK on success
 * @return SX1262_ERROR_NOT_INITIALIZED if driver not initialized
 * @return SX1262_ERROR_INVALID_STATE if already sleeping
 * @return SX1262_ERROR_MUTEX if mutex acquisition failed
 * 
 * Example (typical sensor duty cycle):
 * @code
 *   // Transmit sensor data
 *   sx1262_transmit(data, len, 0, NULL);
 *   
 *   // Wait for ACK
 *   sx1262_turnaround_tx_to_rx(ack_buf, 32, 500, &rx_result);
 *   
 *   // Enter sleep until next cycle (15 minutes)
 *   sx1262_sleep(SX1262_SLEEP_WARM);
 *   delay(15 * 60 * 1000);  // MCU sleep would be better
 *   sx1262_wake();
 *   
 *   // Ready for next TX immediately (warm start)
 * @endcode
 */
sx1262_result_t sx1262_sleep(sx1262_sleep_mode_t mode);

/**
 * @brief Wake from sleep mode
 * 
 * Wakes the SX1262 from sleep mode and returns it to STANDBY_RC.
 * 
 * Wake timing depends on sleep mode:
 *   - WARM start: ~340 µs (config retained, ready immediately)
 *   - COLD start: ~3.5 ms (config lost, must reconfigure)
 * 
 * After COLD start wake, config_valid is set to false. You must call
 * sx1262_config_lora() before TX/RX operations.
 * 
 * Thread-safe: Uses mutex protection.
 * 
 * @return SX1262_OK on success
 * @return SX1262_ERROR_NOT_INITIALIZED if driver not initialized
 * @return SX1262_ERROR_INVALID_STATE if not currently sleeping
 * @return SX1262_ERROR_TIMEOUT if chip failed to wake (BUSY stuck high)
 * 
 * Example:
 * @code
 *   sx1262_wake();
 *   
 *   // After cold start, must reconfigure
 *   if (!sx1262_is_config_valid()) {
 *       sx1262_config_lora(&my_config);
 *   }
 *   
 *   // Now ready for TX/RX
 *   sx1262_transmit(data, len, 0, NULL);
 * @endcode
 */
sx1262_result_t sx1262_wake(void);

/**
 * @brief Check if radio is currently sleeping
 * 
 * @return true if in sleep mode, false otherwise
 */
bool sx1262_is_sleeping(void);

/**
 * @brief Get power management statistics
 * 
 * Returns cumulative sleep statistics for power analysis.
 * 
 * @param total_sleep_ms Pointer to store total sleep time in ms (can be NULL)
 * @param wake_count Pointer to store number of wake cycles (can be NULL)
 * 
 * Example:
 * @code
 *   uint32_t sleep_ms, wakes;
 *   sx1262_get_power_stats(&sleep_ms, &wakes);
 *   float avg_sleep = (wakes > 0) ? (float)sleep_ms / wakes : 0;
 *   printf("Average sleep: %.1f ms over %lu cycles\n", avg_sleep, wakes);
 * @endcode
 */
void sx1262_get_power_stats(uint32_t* total_sleep_ms, uint32_t* wake_count);

// ============================================================================
// INTERNAL HELPER MACROS (for state transition logging)
// ============================================================================

#ifdef DEBUG_VERBOSE_STATE
    #define SX1262_LOG_STATE_TRANSITION(from, to) \
        SX1262_LOG_STATE("State: %s -> %s\n", \
                     sx1262_state_to_string(from), \
                     sx1262_state_to_string(to))
    
    #define SX1262_LOG_STATE_ERROR(state, error) \
        SX1262_LOG_ERROR("[STATE] ERROR in state %s: %s\n", \
                     sx1262_state_to_string(state), \
                     sx1262_error_to_string(error))
#else
    #define SX1262_LOG_STATE_TRANSITION(from, to)
    #define SX1262_LOG_STATE_ERROR(state, error)
#endif

// ============================================================================
// GLOBAL DRIVER INSTANCE (internal - do not access directly)
// ============================================================================
// The actual instance is defined in sx1262_driver.cpp
// All access should go through the API functions above

#endif // SX1262_DRIVER_H
