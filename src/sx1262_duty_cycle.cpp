/**
 * @file sx1262_duty_cycle.cpp
 * @brief Duty Cycle Manager implementation for regulatory compliance
 * 
 * This implementation uses a sliding window algorithm to track airtime
 * usage and enforce regulatory duty cycle limits.
 * 
 * Algorithm:
 *   1. Each transmission is recorded with timestamp and duration
 *   2. Records older than the window are expired (removed)
 *   3. Current duty cycle = sum(airtime in window) / window_duration
 *   4. TX is blocked if adding new airtime would exceed limit
 * 
 * @author Oscar / Claude
 * @date December 2024
 * @version 1.0.0
 */

#include "sx1262_duty_cycle.h"
#include "config.h"

#include <string.h>

// ============================================================================
// PLATFORM ABSTRACTION
// ============================================================================

// Get current time in milliseconds (platform-specific)
#ifdef ARDUINO
    #include <Arduino.h>
    #define DC_GET_TIME_MS()  millis()
#else
    // For non-Arduino platforms, user must provide implementation
    extern uint32_t dc_get_time_ms(void);
    #define DC_GET_TIME_MS()  dc_get_time_ms()
#endif

// ============================================================================
// MODULE STATE
// ============================================================================

/**
 * @brief Duty cycle manager context
 */
typedef struct {
    bool initialized;
    sx1262_dc_region_t region;
    
    // Channel tracking
    sx1262_dc_channel_t channels[SX1262_DC_MAX_CHANNELS];
    uint8_t active_channel_count;
    
    // Global statistics
    uint32_t total_transmissions;
    uint32_t blocked_transmissions;
    uint32_t total_airtime_all_time_ms;
    uint32_t max_single_tx_ms;
    
} sx1262_dc_context_t;

static sx1262_dc_context_t g_dc_ctx;

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

/**
 * @brief Get duty cycle limit for a region (in permille)
 */
static uint32_t get_region_duty_cycle(sx1262_dc_region_t region) {
    switch (region) {
        case SX1262_DC_REGION_ETSI_G1:  return 10;   // 1%
        case SX1262_DC_REGION_ETSI_G2:  return 1;    // 0.1%
        case SX1262_DC_REGION_ETSI_G3:  return 100;  // 10%
        case SX1262_DC_REGION_FCC_915:  return 1000; // 100% (different rules apply)
        default:                         return 10;   // Default to 1%
    }
}

/**
 * @brief Expire old records from a channel
 * 
 * Removes records older than the window duration.
 */
static void expire_channel_records(sx1262_dc_channel_t* channel) {
    if (channel->record_count == 0) {
        return;
    }
    
    uint32_t now = DC_GET_TIME_MS();
    uint32_t window_start = now - channel->window_ms;
    
    // Handle millis() overflow
    bool overflow = (now < channel->window_ms);
    
    // Recalculate total airtime while expiring old records
    uint32_t new_total = 0;
    uint8_t new_count = 0;
    uint8_t read_idx = channel->record_head;
    
    // Temporary storage for valid records
    sx1262_dc_tx_record_t valid_records[SX1262_DC_MAX_TX_RECORDS];
    
    for (uint8_t i = 0; i < channel->record_count; i++) {
        // Calculate actual index (circular buffer)
        uint8_t idx = (channel->record_head + SX1262_DC_MAX_TX_RECORDS - channel->record_count + i) 
                      % SX1262_DC_MAX_TX_RECORDS;
        
        sx1262_dc_tx_record_t* record = &channel->records[idx];
        
        bool is_valid;
        if (overflow) {
            // Handle wraparound: record is valid if it's after window_start OR before now
            is_valid = (record->timestamp_ms >= window_start) || (record->timestamp_ms <= now);
        } else {
            // Normal case: record is valid if it's within the window
            is_valid = (record->timestamp_ms >= window_start);
        }
        
        if (is_valid) {
            valid_records[new_count] = *record;
            new_total += record->airtime_ms;
            new_count++;
        }
    }
    
    // Copy valid records back
    channel->record_count = new_count;
    channel->record_head = new_count % SX1262_DC_MAX_TX_RECORDS;
    channel->total_airtime_ms = new_total;
    
    for (uint8_t i = 0; i < new_count; i++) {
        uint8_t idx = i % SX1262_DC_MAX_TX_RECORDS;
        channel->records[idx] = valid_records[i];
    }
}

/**
 * @brief Add a transmission record to a channel
 */
static sx1262_dc_result_t add_tx_record(sx1262_dc_channel_t* channel, uint32_t airtime_ms) {
    // Expire old records first to make room
    expire_channel_records(channel);
    
    // Check if buffer is full
    if (channel->record_count >= SX1262_DC_MAX_TX_RECORDS) {
        SX1262_LOG_WARN("[DC] TX record buffer full, expiring oldest\n");
        // Force expire the oldest record
        channel->record_count--;
        // Recalculate from oldest record
        uint8_t oldest_idx = (channel->record_head + SX1262_DC_MAX_TX_RECORDS - channel->record_count) 
                             % SX1262_DC_MAX_TX_RECORDS;
        channel->total_airtime_ms -= channel->records[oldest_idx].airtime_ms;
    }
    
    // Add new record
    channel->records[channel->record_head].timestamp_ms = DC_GET_TIME_MS();
    channel->records[channel->record_head].airtime_ms = airtime_ms;
    
    channel->record_head = (channel->record_head + 1) % SX1262_DC_MAX_TX_RECORDS;
    channel->record_count++;
    channel->total_airtime_ms += airtime_ms;
    
    return SX1262_DC_OK;
}

/**
 * @brief Calculate remaining budget for a channel
 */
static uint32_t calculate_budget_remaining(sx1262_dc_channel_t* channel) {
    expire_channel_records(channel);
    
    // Maximum allowed airtime in window
    uint32_t max_airtime = (channel->window_ms * channel->duty_cycle_permille) / 1000;
    
    if (channel->total_airtime_ms >= max_airtime) {
        return 0;
    }
    
    return max_airtime - channel->total_airtime_ms;
}

/**
 * @brief Calculate wait time until budget is available
 */
static uint32_t calculate_wait_time(sx1262_dc_channel_t* channel, uint32_t needed_airtime_ms) {
    expire_channel_records(channel);
    
    uint32_t max_airtime = (channel->window_ms * channel->duty_cycle_permille) / 1000;
    uint32_t budget = calculate_budget_remaining(channel);
    
    if (budget >= needed_airtime_ms) {
        return 0;  // Can transmit now
    }
    
    // Need to wait for oldest records to expire
    // Find how much airtime needs to expire and when
    uint32_t airtime_to_free = needed_airtime_ms - budget;
    uint32_t now = DC_GET_TIME_MS();
    
    // Walk through records from oldest to find when we'll have enough budget
    uint32_t freed = 0;
    for (uint8_t i = 0; i < channel->record_count && freed < airtime_to_free; i++) {
        uint8_t idx = (channel->record_head + SX1262_DC_MAX_TX_RECORDS - channel->record_count + i) 
                      % SX1262_DC_MAX_TX_RECORDS;
        
        sx1262_dc_tx_record_t* record = &channel->records[idx];
        freed += record->airtime_ms;
        
        if (freed >= airtime_to_free) {
            // This record expiring will give us enough budget
            uint32_t record_expires_at = record->timestamp_ms + channel->window_ms;
            
            if (record_expires_at > now) {
                return record_expires_at - now;
            } else {
                return 0;  // Already expired, just needs refresh
            }
        }
    }
    
    // Shouldn't reach here, but return window time as safety
    return channel->window_ms;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

sx1262_dc_result_t sx1262_dc_init(sx1262_dc_region_t region) {
    uint32_t duty_cycle = get_region_duty_cycle(region);
    
    sx1262_dc_result_t res = sx1262_dc_init_custom(duty_cycle, SX1262_DC_DEFAULT_WINDOW_MS);
    
    if (res == SX1262_DC_OK) {
        g_dc_ctx.region = region;
        SX1262_LOG_INFO("[DC] Initialized for %s (%.1f%% duty cycle)\n",
                       sx1262_dc_region_to_string(region),
                       duty_cycle / 10.0);
    }
    
    return res;
}

sx1262_dc_result_t sx1262_dc_init_custom(uint32_t duty_cycle_permille, uint32_t window_ms) {
    if (duty_cycle_permille == 0 || duty_cycle_permille > 1000) {
        SX1262_LOG_ERROR("[DC] Invalid duty cycle: %lu permille\n", duty_cycle_permille);
        return SX1262_DC_ERROR_INVALID_PARAM;
    }
    
    if (window_ms == 0) {
        SX1262_LOG_ERROR("[DC] Invalid window: 0 ms\n");
        return SX1262_DC_ERROR_INVALID_PARAM;
    }
    
    // Clear context
    memset(&g_dc_ctx, 0, sizeof(g_dc_ctx));
    
    // Configure default channel (channel 0)
    g_dc_ctx.channels[0].duty_cycle_permille = duty_cycle_permille;
    g_dc_ctx.channels[0].window_ms = window_ms;
    g_dc_ctx.channels[0].frequency_hz = 0;  // Not specified
    g_dc_ctx.active_channel_count = 1;
    
    g_dc_ctx.region = SX1262_DC_REGION_CUSTOM;
    g_dc_ctx.initialized = true;
    
    SX1262_LOG_INFO("[DC] Custom init: %.1f%% over %lu ms window\n",
                   duty_cycle_permille / 10.0, window_ms);
    
    return SX1262_DC_OK;
}

void sx1262_dc_deinit(void) {
    memset(&g_dc_ctx, 0, sizeof(g_dc_ctx));
    SX1262_LOG_INFO("[DC] Deinitialized\n");
}

bool sx1262_dc_is_initialized(void) {
    return g_dc_ctx.initialized;
}

// ============================================================================
// MULTI-CHANNEL SUPPORT
// ============================================================================

sx1262_dc_result_t sx1262_dc_configure_channel(
    uint8_t channel_idx,
    uint32_t frequency_hz,
    uint32_t duty_cycle_permille
) {
    if (!g_dc_ctx.initialized) {
        return SX1262_DC_ERROR_NOT_INITIALIZED;
    }
    
    if (channel_idx >= SX1262_DC_MAX_CHANNELS) {
        return SX1262_DC_ERROR_INVALID_CHANNEL;
    }
    
    if (duty_cycle_permille == 0 || duty_cycle_permille > 1000) {
        return SX1262_DC_ERROR_INVALID_PARAM;
    }
    
    sx1262_dc_channel_t* channel = &g_dc_ctx.channels[channel_idx];
    
    channel->frequency_hz = frequency_hz;
    channel->duty_cycle_permille = duty_cycle_permille;
    channel->window_ms = SX1262_DC_DEFAULT_WINDOW_MS;
    channel->total_airtime_ms = 0;
    channel->record_head = 0;
    channel->record_count = 0;
    
    if (channel_idx >= g_dc_ctx.active_channel_count) {
        g_dc_ctx.active_channel_count = channel_idx + 1;
    }
    
    SX1262_LOG_INFO("[DC] Channel %d configured: %.3f MHz, %.1f%%\n",
                   channel_idx, frequency_hz / 1000000.0, duty_cycle_permille / 10.0);
    
    return SX1262_DC_OK;
}

// ============================================================================
// TRANSMISSION GATING
// ============================================================================

bool sx1262_dc_can_transmit(uint32_t airtime_ms, uint32_t* out_wait_ms) {
    return sx1262_dc_can_transmit_on_channel(0, airtime_ms, out_wait_ms);
}

bool sx1262_dc_can_transmit_on_channel(
    uint8_t channel_idx,
    uint32_t airtime_ms,
    uint32_t* out_wait_ms
) {
    if (!g_dc_ctx.initialized) {
        if (out_wait_ms) *out_wait_ms = 0;
        return true;  // If not initialized, don't block (fail-open)
    }
    
    if (channel_idx >= g_dc_ctx.active_channel_count) {
        if (out_wait_ms) *out_wait_ms = 0;
        return true;  // Invalid channel, fail-open
    }
    
    sx1262_dc_channel_t* channel = &g_dc_ctx.channels[channel_idx];
    
    uint32_t budget = calculate_budget_remaining(channel);
    
    if (budget >= airtime_ms) {
        if (out_wait_ms) *out_wait_ms = 0;
        return true;
    }
    
    // Calculate wait time
    uint32_t wait = calculate_wait_time(channel, airtime_ms);
    if (out_wait_ms) *out_wait_ms = wait;
    
    SX1262_LOG_DEBUG("[DC] TX blocked: need %lu ms, have %lu ms, wait %lu ms\n",
                    airtime_ms, budget, wait);
    
    return false;
}

sx1262_dc_result_t sx1262_dc_record_transmission(uint32_t airtime_ms) {
    return sx1262_dc_record_transmission_on_channel(0, airtime_ms);
}

sx1262_dc_result_t sx1262_dc_record_transmission_on_channel(
    uint8_t channel_idx,
    uint32_t airtime_ms
) {
    if (!g_dc_ctx.initialized) {
        return SX1262_DC_ERROR_NOT_INITIALIZED;
    }
    
    if (channel_idx >= g_dc_ctx.active_channel_count) {
        return SX1262_DC_ERROR_INVALID_CHANNEL;
    }
    
    sx1262_dc_channel_t* channel = &g_dc_ctx.channels[channel_idx];
    
    // Check if this would violate duty cycle (log warning but still record)
    uint32_t budget = calculate_budget_remaining(channel);
    if (airtime_ms > budget) {
        SX1262_LOG_WARN("[DC] Recording TX that exceeds budget! (%lu ms > %lu ms available)\n",
                       airtime_ms, budget);
        g_dc_ctx.blocked_transmissions++;  // Count as would-be-blocked
    }
    
    // Add the record
    sx1262_dc_result_t res = add_tx_record(channel, airtime_ms);
    
    if (res == SX1262_DC_OK) {
        g_dc_ctx.total_transmissions++;
        g_dc_ctx.total_airtime_all_time_ms += airtime_ms;
        
        if (airtime_ms > g_dc_ctx.max_single_tx_ms) {
            g_dc_ctx.max_single_tx_ms = airtime_ms;
        }
        
        SX1262_LOG_DEBUG("[DC] Recorded TX: %lu ms (budget now: %lu ms)\n",
                        airtime_ms, calculate_budget_remaining(channel));
    }
    
    return res;
}

// ============================================================================
// BUDGET QUERIES
// ============================================================================

uint32_t sx1262_dc_get_budget_remaining_ms(void) {
    return sx1262_dc_get_channel_budget_remaining_ms(0);
}

uint32_t sx1262_dc_get_channel_budget_remaining_ms(uint8_t channel_idx) {
    if (!g_dc_ctx.initialized || channel_idx >= g_dc_ctx.active_channel_count) {
        return UINT32_MAX;  // Unlimited if not tracking
    }
    
    return calculate_budget_remaining(&g_dc_ctx.channels[channel_idx]);
}

float sx1262_dc_get_current_usage_percent(void) {
    if (!g_dc_ctx.initialized) {
        return 0.0f;
    }
    
    sx1262_dc_channel_t* channel = &g_dc_ctx.channels[0];
    expire_channel_records(channel);
    
    if (channel->window_ms == 0) {
        return 0.0f;
    }
    
    return (channel->total_airtime_ms * 100.0f) / channel->window_ms;
}

uint32_t sx1262_dc_get_wait_time_ms(uint32_t airtime_ms) {
    if (!g_dc_ctx.initialized) {
        return 0;
    }
    
    return calculate_wait_time(&g_dc_ctx.channels[0], airtime_ms);
}

// ============================================================================
// MAINTENANCE
// ============================================================================

void sx1262_dc_expire_old_records(void) {
    if (!g_dc_ctx.initialized) {
        return;
    }
    
    for (uint8_t i = 0; i < g_dc_ctx.active_channel_count; i++) {
        expire_channel_records(&g_dc_ctx.channels[i]);
    }
}

void sx1262_dc_reset(void) {
    if (!g_dc_ctx.initialized) {
        return;
    }
    
    // Clear all channel records but keep configuration
    for (uint8_t i = 0; i < g_dc_ctx.active_channel_count; i++) {
        g_dc_ctx.channels[i].total_airtime_ms = 0;
        g_dc_ctx.channels[i].record_head = 0;
        g_dc_ctx.channels[i].record_count = 0;
    }
    
    // Reset statistics
    g_dc_ctx.total_transmissions = 0;
    g_dc_ctx.blocked_transmissions = 0;
    g_dc_ctx.total_airtime_all_time_ms = 0;
    g_dc_ctx.max_single_tx_ms = 0;
    
    SX1262_LOG_INFO("[DC] Reset complete\n");
}

// ============================================================================
// STATISTICS
// ============================================================================

sx1262_dc_result_t sx1262_dc_get_stats(sx1262_dc_stats_t* stats) {
    if (stats == NULL) {
        return SX1262_DC_ERROR_INVALID_PARAM;
    }
    
    if (!g_dc_ctx.initialized) {
        memset(stats, 0, sizeof(sx1262_dc_stats_t));
        return SX1262_DC_ERROR_NOT_INITIALIZED;
    }
    
    // Expire old records first for accurate stats
    expire_channel_records(&g_dc_ctx.channels[0]);
    
    stats->total_transmissions = g_dc_ctx.total_transmissions;
    stats->blocked_transmissions = g_dc_ctx.blocked_transmissions;
    stats->total_airtime_ms = g_dc_ctx.total_airtime_all_time_ms;
    stats->current_window_airtime_ms = g_dc_ctx.channels[0].total_airtime_ms;
    stats->budget_remaining_ms = calculate_budget_remaining(&g_dc_ctx.channels[0]);
    stats->max_single_tx_ms = g_dc_ctx.max_single_tx_ms;
    stats->duty_cycle_percent = sx1262_dc_get_current_usage_percent();
    
    return SX1262_DC_OK;
}

void sx1262_dc_reset_stats(void) {
    g_dc_ctx.total_transmissions = 0;
    g_dc_ctx.blocked_transmissions = 0;
    g_dc_ctx.total_airtime_all_time_ms = 0;
    g_dc_ctx.max_single_tx_ms = 0;
}

const char* sx1262_dc_region_to_string(sx1262_dc_region_t region) {
    switch (region) {
        case SX1262_DC_REGION_ETSI_G1:  return "ETSI G1 (868.0-868.6 MHz, 1%)";
        case SX1262_DC_REGION_ETSI_G2:  return "ETSI G2 (868.7-869.2 MHz, 0.1%)";
        case SX1262_DC_REGION_ETSI_G3:  return "ETSI G3 (869.4-869.65 MHz, 10%)";
        case SX1262_DC_REGION_FCC_915:  return "FCC 915 (902-928 MHz)";
        case SX1262_DC_REGION_CUSTOM:   return "Custom";
        default:                         return "Unknown";
    }
}

void sx1262_dc_print_status(void) {
    if (!g_dc_ctx.initialized) {
        SX1262_LOG_INFO("[DC] Not initialized\n");
        return;
    }
    
    sx1262_dc_stats_t stats;
    sx1262_dc_get_stats(&stats);
    
    SX1262_LOG_INFO("[DC] === Duty Cycle Status ===\n");
    SX1262_LOG_INFO("[DC] Region: %s\n", sx1262_dc_region_to_string(g_dc_ctx.region));
    SX1262_LOG_INFO("[DC] Current usage: %.3f%%\n", stats.duty_cycle_percent);
    SX1262_LOG_INFO("[DC] Budget remaining: %lu ms\n", stats.budget_remaining_ms);
    SX1262_LOG_INFO("[DC] Window airtime: %lu ms\n", stats.current_window_airtime_ms);
    SX1262_LOG_INFO("[DC] Total TX: %lu (blocked: %lu)\n", 
                   stats.total_transmissions, stats.blocked_transmissions);
    SX1262_LOG_INFO("[DC] ===========================\n");
}
