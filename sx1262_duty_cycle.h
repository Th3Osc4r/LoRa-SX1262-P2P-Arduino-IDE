#ifndef SX1262_DUTY_CYCLE_H
#define SX1262_DUTY_CYCLE_H

/**
 * @file sx1262_duty_cycle.h
 * @brief Duty Cycle Manager for regulatory compliance (ETSI, FCC)
 * 
 * This module tracks transmission airtime and enforces regulatory duty cycle
 * limits. It is designed as a standalone, optional layer that works alongside
 * the SX1262 driver without modifying core driver files.
 * 
 * Regulatory Background:
 * 
 *   ETSI (Europe - 868 MHz):
 *     - Sub-band g1 (868.0-868.6 MHz): 1% duty cycle = 36 sec/hour
 *     - Sub-band g2 (868.7-869.2 MHz): 0.1% duty cycle = 3.6 sec/hour
 *     - Sub-band g3 (869.4-869.65 MHz): 10% duty cycle = 360 sec/hour
 *   
 *   FCC (USA - 915 MHz):
 *     - Frequency hopping: 400ms max dwell time per channel
 *     - Digital modulation: 125 kHz max occupied bandwidth
 * 
 * Usage Pattern:
 * @code
 *   // Initialization
 *   sx1262_dc_init(SX1262_DC_REGION_ETSI_G1);
 *   
 *   // Before each transmission
 *   uint32_t wait_ms;
 *   if (sx1262_dc_can_transmit(estimated_airtime_ms, &wait_ms)) {
 *       sx1262_transmit(...);
 *       sx1262_dc_record_transmission(actual_airtime_ms);
 *   } else {
 *       // Must wait 'wait_ms' before transmitting
 *       delay(wait_ms);
 *   }
 * @endcode
 * 
 * Architecture:
 *   - Uses a sliding window algorithm (default 1 hour)
 *   - Tracks individual transmissions with timestamps
 *   - Automatically expires old transmissions outside window
 *   - Supports multiple channels for frequency hopping
 * 
 * @author Oscar / Claude
 * @date December 2024
 * @version 1.0.0
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// COMPILE-TIME CONFIGURATION
// ============================================================================

/**
 * @brief Maximum number of transmission records to track
 * 
 * Each record uses ~12 bytes. More records = finer granularity but more RAM.
 * 100 records supports ~100 transmissions per hour tracking.
 */
#ifndef SX1262_DC_MAX_TX_RECORDS
#define SX1262_DC_MAX_TX_RECORDS    100
#endif

/**
 * @brief Maximum number of channels to track independently
 * 
 * For single-channel operation, set to 1.
 * For frequency hopping, set to number of channels.
 */
#ifndef SX1262_DC_MAX_CHANNELS
#define SX1262_DC_MAX_CHANNELS      8
#endif

/**
 * @brief Default time window for duty cycle calculation (milliseconds)
 * 
 * ETSI specifies 1 hour (3,600,000 ms) as the reference period.
 */
#define SX1262_DC_DEFAULT_WINDOW_MS     3600000UL

// ============================================================================
// REGULATORY REGIONS
// ============================================================================

/**
 * @brief Regulatory region/sub-band presets
 * 
 * Each preset configures the appropriate duty cycle limit.
 */
typedef enum {
    SX1262_DC_REGION_ETSI_G1 = 0,   ///< 868.0-868.6 MHz, 1% duty cycle
    SX1262_DC_REGION_ETSI_G2,        ///< 868.7-869.2 MHz, 0.1% duty cycle  
    SX1262_DC_REGION_ETSI_G3,        ///< 869.4-869.65 MHz, 10% duty cycle
    SX1262_DC_REGION_FCC_915,        ///< 902-928 MHz, frequency hopping rules
    SX1262_DC_REGION_CUSTOM          ///< User-defined limit
} sx1262_dc_region_t;

// ============================================================================
// RETURN CODES
// ============================================================================

/**
 * @brief Duty cycle manager return codes
 */
typedef enum {
    SX1262_DC_OK = 0,                    ///< Operation successful
    SX1262_DC_ERROR_INVALID_PARAM = -1,  ///< Invalid parameter
    SX1262_DC_ERROR_NOT_INITIALIZED = -2,///< Module not initialized
    SX1262_DC_ERROR_CHANNEL_FULL = -3,   ///< TX record buffer full
    SX1262_DC_ERROR_INVALID_CHANNEL = -4,///< Channel index out of range
    SX1262_DC_TX_BLOCKED = -5            ///< Transmission would violate duty cycle
} sx1262_dc_result_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Single transmission record
 */
typedef struct {
    uint32_t timestamp_ms;    ///< When transmission started (millis())
    uint32_t airtime_ms;      ///< Duration of transmission
} sx1262_dc_tx_record_t;

/**
 * @brief Per-channel duty cycle tracking
 */
typedef struct {
    uint32_t frequency_hz;                              ///< Channel frequency (for reference)
    uint32_t duty_cycle_permille;                       ///< Duty cycle limit (1000 = 100%, 10 = 1%)
    uint32_t window_ms;                                 ///< Time window for calculation
    uint32_t total_airtime_ms;                          ///< Total airtime in current window
    sx1262_dc_tx_record_t records[SX1262_DC_MAX_TX_RECORDS]; ///< TX history
    uint8_t record_head;                                ///< Circular buffer head
    uint8_t record_count;                               ///< Number of valid records
} sx1262_dc_channel_t;

/**
 * @brief Duty cycle manager statistics
 */
typedef struct {
    uint32_t total_transmissions;       ///< Total TX count since init
    uint32_t blocked_transmissions;     ///< TX blocked due to duty cycle
    uint32_t total_airtime_ms;          ///< Total airtime since init
    uint32_t current_window_airtime_ms; ///< Airtime in current window
    uint32_t budget_remaining_ms;       ///< Available airtime in window
    uint32_t max_single_tx_ms;          ///< Longest single transmission
    float duty_cycle_percent;           ///< Current duty cycle usage (%)
} sx1262_dc_stats_t;

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize duty cycle manager with regional preset
 * 
 * Configures the duty cycle limit based on regulatory region.
 * Uses channel 0 as the default/only channel.
 * 
 * @param region  Regulatory region preset
 * @return SX1262_DC_OK on success
 * 
 * Example:
 * @code
 *   sx1262_dc_init(SX1262_DC_REGION_ETSI_G1);  // 1% duty cycle
 * @endcode
 */
sx1262_dc_result_t sx1262_dc_init(sx1262_dc_region_t region);

/**
 * @brief Initialize with custom duty cycle limit
 * 
 * @param duty_cycle_permille  Duty cycle limit in permille (10 = 1%, 100 = 10%)
 * @param window_ms            Time window in milliseconds (typically 3600000 for 1 hour)
 * @return SX1262_DC_OK on success
 * 
 * Example:
 * @code
 *   // Custom 2.5% duty cycle over 30 minutes
 *   sx1262_dc_init_custom(25, 1800000);
 * @endcode
 */
sx1262_dc_result_t sx1262_dc_init_custom(uint32_t duty_cycle_permille, uint32_t window_ms);

/**
 * @brief Deinitialize and reset duty cycle manager
 */
void sx1262_dc_deinit(void);

/**
 * @brief Check if module is initialized
 * 
 * @return true if initialized
 */
bool sx1262_dc_is_initialized(void);

// ============================================================================
// MULTI-CHANNEL SUPPORT
// ============================================================================

/**
 * @brief Configure a specific channel
 * 
 * For frequency hopping or multi-channel operation. Each channel
 * maintains independent duty cycle tracking.
 * 
 * @param channel_idx         Channel index (0 to SX1262_DC_MAX_CHANNELS-1)
 * @param frequency_hz        Channel frequency (for reference)
 * @param duty_cycle_permille Duty cycle limit for this channel
 * @return SX1262_DC_OK on success
 */
sx1262_dc_result_t sx1262_dc_configure_channel(
    uint8_t channel_idx,
    uint32_t frequency_hz,
    uint32_t duty_cycle_permille
);

// ============================================================================
// TRANSMISSION GATING
// ============================================================================

/**
 * @brief Check if transmission is allowed under duty cycle limits
 * 
 * Call this BEFORE transmitting. If returns false, wait_ms indicates
 * how long to wait before the transmission would be allowed.
 * 
 * @param airtime_ms        Expected transmission airtime in milliseconds
 * @param out_wait_ms       Pointer to receive wait time if blocked (can be NULL)
 * @return true if transmission is allowed, false if must wait
 * 
 * Example:
 * @code
 *   uint32_t wait_ms;
 *   uint32_t toa = sx1262_get_time_on_air_ms(&config, payload_len);
 *   
 *   if (sx1262_dc_can_transmit(toa, &wait_ms)) {
 *       sx1262_transmit(payload, len, 0, &result);
 *       sx1262_dc_record_transmission(result.tx_duration_ms);
 *   } else {
 *       Serial.printf("Must wait %lu ms before TX\n", wait_ms);
 *   }
 * @endcode
 */
bool sx1262_dc_can_transmit(uint32_t airtime_ms, uint32_t* out_wait_ms);

/**
 * @brief Check if transmission is allowed on specific channel
 * 
 * @param channel_idx  Channel index
 * @param airtime_ms   Expected transmission airtime
 * @param out_wait_ms  Pointer to receive wait time if blocked
 * @return true if transmission is allowed
 */
bool sx1262_dc_can_transmit_on_channel(
    uint8_t channel_idx,
    uint32_t airtime_ms,
    uint32_t* out_wait_ms
);

/**
 * @brief Record a completed transmission
 * 
 * Call this AFTER transmitting to update the duty cycle tracking.
 * Uses channel 0 (default channel).
 * 
 * @param airtime_ms  Actual transmission airtime in milliseconds
 * @return SX1262_DC_OK on success
 */
sx1262_dc_result_t sx1262_dc_record_transmission(uint32_t airtime_ms);

/**
 * @brief Record a transmission on specific channel
 * 
 * @param channel_idx  Channel index
 * @param airtime_ms   Actual transmission airtime
 * @return SX1262_DC_OK on success
 */
sx1262_dc_result_t sx1262_dc_record_transmission_on_channel(
    uint8_t channel_idx,
    uint32_t airtime_ms
);

// ============================================================================
// BUDGET QUERIES
// ============================================================================

/**
 * @brief Get remaining airtime budget in current window
 * 
 * @return Remaining airtime in milliseconds
 */
uint32_t sx1262_dc_get_budget_remaining_ms(void);

/**
 * @brief Get remaining budget for specific channel
 * 
 * @param channel_idx  Channel index
 * @return Remaining airtime in milliseconds
 */
uint32_t sx1262_dc_get_channel_budget_remaining_ms(uint8_t channel_idx);

/**
 * @brief Get current duty cycle usage as percentage
 * 
 * @return Current duty cycle (0.0 to 100.0)
 */
float sx1262_dc_get_current_usage_percent(void);

/**
 * @brief Get time until next transmission is allowed
 * 
 * If budget is exhausted, returns time until oldest TX record expires.
 * 
 * @param airtime_ms  Planned transmission airtime
 * @return Wait time in milliseconds (0 if can transmit now)
 */
uint32_t sx1262_dc_get_wait_time_ms(uint32_t airtime_ms);

// ============================================================================
// MAINTENANCE
// ============================================================================

/**
 * @brief Expire old transmission records
 * 
 * Called automatically by other functions, but can be called manually
 * to clean up the record buffer.
 */
void sx1262_dc_expire_old_records(void);

/**
 * @brief Reset all duty cycle tracking
 * 
 * Clears all transmission records and statistics.
 * Does NOT change configuration (region, limits).
 */
void sx1262_dc_reset(void);

// ============================================================================
// STATISTICS
// ============================================================================

/**
 * @brief Get duty cycle statistics
 * 
 * @param stats  Pointer to statistics structure to fill
 * @return SX1262_DC_OK on success
 */
sx1262_dc_result_t sx1262_dc_get_stats(sx1262_dc_stats_t* stats);

/**
 * @brief Reset statistics counters
 * 
 * Clears total_transmissions, blocked_transmissions, etc.
 * Does NOT clear current window tracking.
 */
void sx1262_dc_reset_stats(void);

/**
 * @brief Get human-readable region name
 * 
 * @param region  Region enum value
 * @return Region name string
 */
const char* sx1262_dc_region_to_string(sx1262_dc_region_t region);

/**
 * @brief Print duty cycle status to log
 * 
 * Outputs current usage, remaining budget, etc.
 */
void sx1262_dc_print_status(void);

#ifdef __cplusplus
}
#endif

#endif // SX1262_DUTY_CYCLE_H
