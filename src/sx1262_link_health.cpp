/**
 * @file sx1262_link_health.cpp
 * @brief Link Health Monitor implementation for multi-node LoRa networks
 * 
 * This implementation provides node tracking, timeout detection, and
 * alarm management for gateway applications monitoring multiple sensor nodes.
 * 
 * @author Oscar / Claude
 * @date December 2024
 * @version 1.0.0
 */

#include "sx1262_link_health.h"
#include "config.h"

#include <string.h>

// ============================================================================
// PLATFORM ABSTRACTION
// ============================================================================

#ifdef ARDUINO
    #include <Arduino.h>
    #define LINK_GET_TIME_MS()  millis()
#else
    extern uint32_t link_get_time_ms(void);
    #define LINK_GET_TIME_MS()  link_get_time_ms()
#endif

// ============================================================================
// INTERNAL DATA STRUCTURES
// ============================================================================

/**
 * @brief Internal node tracking structure
 */
typedef struct {
    bool active;                        ///< Slot is in use
    uint8_t node_id;                    ///< Node identifier
    sx1262_node_status_t status;        ///< Current status
    
    // Configuration
    uint32_t expected_interval_ms;      ///< Expected TX interval
    
    // Timing tracking
    uint32_t first_seen_ms;             ///< First packet timestamp
    uint32_t last_seen_ms;              ///< Last packet timestamp
    uint32_t last_interval_ms;          ///< Measured interval
    
    // Packet counts
    uint32_t packets_received;          ///< Total received
    uint8_t consecutive_missed;         ///< Current missed streak
    uint32_t total_missed;              ///< Total missed intervals
    
    // Signal quality history (circular buffer)
    int16_t rssi_history[SX1262_LINK_SIGNAL_HISTORY_SIZE];
    int8_t snr_history[SX1262_LINK_SIGNAL_HISTORY_SIZE];
    uint8_t signal_history_idx;
    uint8_t signal_history_count;
    
    // Signal min/max
    int16_t rssi_min;
    int16_t rssi_max;
    int8_t snr_min;
    int8_t snr_max;
    
    // Alarm state
    bool alarm_active;
    sx1262_link_alarm_type_t last_alarm_type;
    uint32_t last_alarm_ms;
    uint32_t last_check_ms;             ///< Last timeout check timestamp
    
} link_node_t;

/**
 * @brief Module context
 */
typedef struct {
    bool initialized;
    uint8_t max_nodes;
    uint8_t active_node_count;
    
    // Alarm callback
    sx1262_link_alarm_callback_t alarm_callback;
    
    // Configuration
    uint8_t silent_threshold_intervals;
    uint8_t jitter_threshold_percent;
    int16_t rssi_threshold_dbm;
    
    // Node table
    link_node_t nodes[SX1262_LINK_MAX_NODES];
    
} link_context_t;

static link_context_t g_link_ctx;

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

/**
 * @brief Find node by ID
 */
static link_node_t* find_node(uint8_t node_id) {
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        if (g_link_ctx.nodes[i].active && g_link_ctx.nodes[i].node_id == node_id) {
            return &g_link_ctx.nodes[i];
        }
    }
    return NULL;
}

/**
 * @brief Find free slot in node table
 */
static link_node_t* find_free_slot(void) {
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        if (!g_link_ctx.nodes[i].active) {
            return &g_link_ctx.nodes[i];
        }
    }
    return NULL;
}

/**
 * @brief Calculate average RSSI from history
 */
static int16_t calculate_rssi_avg(link_node_t* node) {
    if (node->signal_history_count == 0) {
        return 0;
    }
    
    int32_t sum = 0;
    for (uint8_t i = 0; i < node->signal_history_count; i++) {
        sum += node->rssi_history[i];
    }
    return (int16_t)(sum / node->signal_history_count);
}

/**
 * @brief Calculate average SNR from history
 */
static int8_t calculate_snr_avg(link_node_t* node) {
    if (node->signal_history_count == 0) {
        return 0;
    }
    
    int32_t sum = 0;
    for (uint8_t i = 0; i < node->signal_history_count; i++) {
        sum += node->snr_history[i];
    }
    return (int8_t)(sum / node->signal_history_count);
}

/**
 * @brief Add signal sample to history
 */
static void add_signal_sample(link_node_t* node, int16_t rssi, int8_t snr) {
    node->rssi_history[node->signal_history_idx] = rssi;
    node->snr_history[node->signal_history_idx] = snr;
    
    node->signal_history_idx = (node->signal_history_idx + 1) % SX1262_LINK_SIGNAL_HISTORY_SIZE;
    
    if (node->signal_history_count < SX1262_LINK_SIGNAL_HISTORY_SIZE) {
        node->signal_history_count++;
    }
    
    // Update min/max
    if (rssi < node->rssi_min || node->packets_received == 1) node->rssi_min = rssi;
    if (rssi > node->rssi_max || node->packets_received == 1) node->rssi_max = rssi;
    if (snr < node->snr_min || node->packets_received == 1) node->snr_min = snr;
    if (snr > node->snr_max || node->packets_received == 1) node->snr_max = snr;
}

/**
 * @brief Trigger alarm callback
 */
static void trigger_alarm(link_node_t* node, sx1262_link_alarm_type_t type, 
                          sx1262_link_severity_t severity) {
    
    node->alarm_active = (type != SX1262_LINK_ALARM_RECOVERED);
    node->last_alarm_type = type;
    node->last_alarm_ms = LINK_GET_TIME_MS();
    
    // Update node status based on severity
    switch (severity) {
        case SX1262_LINK_SEVERITY_INFO:
            if (type == SX1262_LINK_ALARM_RECOVERED) {
                node->status = SX1262_NODE_STATUS_HEALTHY;
            }
            break;
        case SX1262_LINK_SEVERITY_WARNING:
            node->status = SX1262_NODE_STATUS_WARNING;
            break;
        case SX1262_LINK_SEVERITY_CRITICAL:
            node->status = SX1262_NODE_STATUS_CRITICAL;
            break;
    }
    
    // Call user callback if registered
    if (g_link_ctx.alarm_callback != NULL) {
        sx1262_link_alarm_t alarm;
        memset(&alarm, 0, sizeof(alarm));
        
        alarm.node_id = node->node_id;
        alarm.type = type;
        alarm.severity = severity;
        alarm.timestamp_ms = LINK_GET_TIME_MS();
        
        // Fill type-specific data
        switch (type) {
            case SX1262_LINK_ALARM_SILENT:
                alarm.data.silent.last_seen_ms = node->last_seen_ms;
                alarm.data.silent.silent_duration_ms = LINK_GET_TIME_MS() - node->last_seen_ms;
                alarm.data.silent.missed_intervals = node->consecutive_missed;
                break;
                
            case SX1262_LINK_ALARM_IRREGULAR:
                alarm.data.irregular.expected_interval_ms = node->expected_interval_ms;
                alarm.data.irregular.actual_interval_ms = node->last_interval_ms;
                if (node->expected_interval_ms > 0) {
                    alarm.data.irregular.jitter_percent = 
                        ((int32_t)node->last_interval_ms - (int32_t)node->expected_interval_ms) * 100 
                        / (int32_t)node->expected_interval_ms;
                }
                break;
                
            case SX1262_LINK_ALARM_WEAK_SIGNAL:
                alarm.data.weak_signal.current_rssi = calculate_rssi_avg(node);
                alarm.data.weak_signal.threshold_rssi = g_link_ctx.rssi_threshold_dbm;
                alarm.data.weak_signal.current_snr = calculate_snr_avg(node);
                break;
                
            case SX1262_LINK_ALARM_RECOVERED:
                // No additional data needed
                break;
        }
        
        g_link_ctx.alarm_callback(&alarm);
    }
    
    SX1262_LOG_INFO("[LINK] Alarm: Node %d - %s (%s)\n",
                   node->node_id,
                   sx1262_link_alarm_type_to_string(type),
                   sx1262_link_severity_to_string(severity));
}

// ============================================================================
// INITIALIZATION
// ============================================================================

sx1262_link_result_t sx1262_link_health_init(uint8_t max_nodes, sx1262_link_alarm_callback_t callback) {
    if (max_nodes == 0 || max_nodes > SX1262_LINK_MAX_NODES) {
        SX1262_LOG_ERROR("[LINK] Invalid max_nodes: %d (max %d)\n", max_nodes, SX1262_LINK_MAX_NODES);
        return SX1262_LINK_ERROR_INVALID_PARAM;
    }
    
    memset(&g_link_ctx, 0, sizeof(g_link_ctx));
    
    g_link_ctx.max_nodes = max_nodes;
    g_link_ctx.alarm_callback = callback;
    
    // Set defaults
    g_link_ctx.silent_threshold_intervals = SX1262_LINK_DEFAULT_SILENT_INTERVALS;
    g_link_ctx.jitter_threshold_percent = SX1262_LINK_DEFAULT_JITTER_THRESHOLD_PCT;
    g_link_ctx.rssi_threshold_dbm = SX1262_LINK_DEFAULT_RSSI_THRESHOLD_DBM;
    
    g_link_ctx.initialized = true;
    
    SX1262_LOG_INFO("[LINK] Initialized for up to %d nodes\n", max_nodes);
    
    return SX1262_LINK_OK;
}

void sx1262_link_health_deinit(void) {
    memset(&g_link_ctx, 0, sizeof(g_link_ctx));
    SX1262_LOG_INFO("[LINK] Deinitialized\n");
}

bool sx1262_link_health_is_initialized(void) {
    return g_link_ctx.initialized;
}

void sx1262_link_health_set_callback(sx1262_link_alarm_callback_t callback) {
    g_link_ctx.alarm_callback = callback;
}

// ============================================================================
// NODE REGISTRATION
// ============================================================================

sx1262_link_result_t sx1262_link_health_register_node(
    uint8_t node_id,
    uint32_t expected_interval_ms
) {
    if (!g_link_ctx.initialized) {
        return SX1262_LINK_ERROR_NOT_INITIALIZED;
    }
    
    if (node_id == 0 || node_id == 255) {
        SX1262_LOG_ERROR("[LINK] Invalid node_id: %d\n", node_id);
        return SX1262_LINK_ERROR_INVALID_PARAM;
    }
    
    if (expected_interval_ms == 0) {
        SX1262_LOG_ERROR("[LINK] Invalid interval: 0\n");
        return SX1262_LINK_ERROR_INVALID_PARAM;
    }
    
    // Check if already exists
    if (find_node(node_id) != NULL) {
        SX1262_LOG_WARN("[LINK] Node %d already registered\n", node_id);
        return SX1262_LINK_ERROR_ALREADY_EXISTS;
    }
    
    // Find free slot
    link_node_t* node = find_free_slot();
    if (node == NULL) {
        SX1262_LOG_ERROR("[LINK] Node table full\n");
        return SX1262_LINK_ERROR_TABLE_FULL;
    }
    
    // Initialize node
    memset(node, 0, sizeof(link_node_t));
    node->active = true;
    node->node_id = node_id;
    node->expected_interval_ms = expected_interval_ms;
    node->status = SX1262_NODE_STATUS_UNKNOWN;
    
    g_link_ctx.active_node_count++;
    
    SX1262_LOG_INFO("[LINK] Registered node %d (interval: %lu ms)\n", 
                   node_id, expected_interval_ms);
    
    return SX1262_LINK_OK;
}

sx1262_link_result_t sx1262_link_health_unregister_node(uint8_t node_id) {
    if (!g_link_ctx.initialized) {
        return SX1262_LINK_ERROR_NOT_INITIALIZED;
    }
    
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        return SX1262_LINK_ERROR_NODE_NOT_FOUND;
    }
    
    node->active = false;
    g_link_ctx.active_node_count--;
    
    SX1262_LOG_INFO("[LINK] Unregistered node %d\n", node_id);
    
    return SX1262_LINK_OK;
}

bool sx1262_link_health_is_node_registered(uint8_t node_id) {
    return find_node(node_id) != NULL;
}

uint8_t sx1262_link_health_get_node_count(void) {
    return g_link_ctx.active_node_count;
}

// ============================================================================
// RECEPTION RECORDING
// ============================================================================

sx1262_link_result_t sx1262_link_health_record_rx(
    uint8_t node_id,
    int16_t rssi,
    int8_t snr
) {
    if (!g_link_ctx.initialized) {
        return SX1262_LINK_ERROR_NOT_INITIALIZED;
    }
    
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        SX1262_LOG_DEBUG("[LINK] RX from unregistered node %d\n", node_id);
        return SX1262_LINK_ERROR_NODE_NOT_FOUND;
    }
    
    uint32_t now = LINK_GET_TIME_MS();
    
    // Calculate interval if not first packet
    if (node->last_seen_ms > 0) {
        node->last_interval_ms = now - node->last_seen_ms;
        
        // Check for irregular timing
        if (node->expected_interval_ms > 0) {
            int32_t expected = (int32_t)node->expected_interval_ms;
            int32_t actual = (int32_t)node->last_interval_ms;
            int32_t jitter_pct = ((actual - expected) * 100) / expected;
            
            // Check if jitter exceeds threshold (absolute value)
            if (jitter_pct < 0) jitter_pct = -jitter_pct;
            
            if (jitter_pct > g_link_ctx.jitter_threshold_percent) {
                SX1262_LOG_WARN("[LINK] Node %d irregular: expected %lu ms, got %lu ms (%ld%%)\n",
                               node_id, node->expected_interval_ms, node->last_interval_ms,
                               (long)jitter_pct);
                trigger_alarm(node, SX1262_LINK_ALARM_IRREGULAR, SX1262_LINK_SEVERITY_WARNING);
            }
        }
    } else {
        // First packet from this node
        node->first_seen_ms = now;
    }
    
    node->last_seen_ms = now;
    node->packets_received++;
    
    // Clear missed counter
    if (node->consecutive_missed > 0) {
        // Was in alarm, now recovered
        if (node->alarm_active && node->last_alarm_type == SX1262_LINK_ALARM_SILENT) {
            trigger_alarm(node, SX1262_LINK_ALARM_RECOVERED, SX1262_LINK_SEVERITY_INFO);
        }
        node->consecutive_missed = 0;
    }
    
    // Update signal stats
    add_signal_sample(node, rssi, snr);
    
    // Check for weak signal
    int16_t avg_rssi = calculate_rssi_avg(node);
    if (avg_rssi < g_link_ctx.rssi_threshold_dbm) {
        if (!node->alarm_active || node->last_alarm_type != SX1262_LINK_ALARM_WEAK_SIGNAL) {
            trigger_alarm(node, SX1262_LINK_ALARM_WEAK_SIGNAL, SX1262_LINK_SEVERITY_WARNING);
        }
    }
    
    // Update status if was in alarm for weak signal but signal improved
    if (node->alarm_active && node->last_alarm_type == SX1262_LINK_ALARM_WEAK_SIGNAL) {
        if (avg_rssi >= g_link_ctx.rssi_threshold_dbm + 3) {  // 3 dB hysteresis
            trigger_alarm(node, SX1262_LINK_ALARM_RECOVERED, SX1262_LINK_SEVERITY_INFO);
        }
    }
    
    // If no alarms, mark as healthy
    if (!node->alarm_active) {
        node->status = SX1262_NODE_STATUS_HEALTHY;
    }
    
    SX1262_LOG_DEBUG("[LINK] Node %d RX: RSSI=%d, SNR=%d, pkts=%lu\n",
                    node_id, rssi, snr, node->packets_received);
    
    return SX1262_LINK_OK;
}

sx1262_link_result_t sx1262_link_health_record_rx_auto(
    uint8_t node_id,
    int16_t rssi,
    int8_t snr,
    uint32_t default_interval_ms
) {
    if (!g_link_ctx.initialized) {
        return SX1262_LINK_ERROR_NOT_INITIALIZED;
    }
    
    // Auto-register if not found
    if (find_node(node_id) == NULL) {
        sx1262_link_result_t res = sx1262_link_health_register_node(node_id, default_interval_ms);
        if (res != SX1262_LINK_OK && res != SX1262_LINK_ERROR_ALREADY_EXISTS) {
            return res;
        }
    }
    
    return sx1262_link_health_record_rx(node_id, rssi, snr);
}

// ============================================================================
// TIMEOUT CHECKING
// ============================================================================

uint8_t sx1262_link_health_check_timeouts(void) {
    if (!g_link_ctx.initialized) {
        return 0;
    }
    
    uint32_t now = LINK_GET_TIME_MS();
    uint8_t alarmed_count = 0;
    
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        link_node_t* node = &g_link_ctx.nodes[i];
        
        if (!node->active) {
            continue;
        }
        
        // Skip if never seen (can't calculate timeout)
        if (node->first_seen_ms == 0) {
            continue;
        }
        
        // Calculate how many intervals have been missed
        uint32_t silence_duration = now - node->last_seen_ms;
        uint32_t expected_since_last = node->expected_interval_ms;
        
        if (expected_since_last > 0 && silence_duration > expected_since_last) {
            uint8_t missed = (uint8_t)(silence_duration / expected_since_last);
            
            if (missed > node->consecutive_missed) {
                node->total_missed += (missed - node->consecutive_missed);
                node->consecutive_missed = missed;
                
                // Check if threshold exceeded
                if (missed >= g_link_ctx.silent_threshold_intervals) {
                    if (!node->alarm_active || node->last_alarm_type != SX1262_LINK_ALARM_SILENT) {
                        trigger_alarm(node, SX1262_LINK_ALARM_SILENT, SX1262_LINK_SEVERITY_CRITICAL);
                    }
                }
            }
        }
        
        // Count alarmed nodes
        if (node->alarm_active) {
            alarmed_count++;
        }
        
        node->last_check_ms = now;
    }
    
    return alarmed_count;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void sx1262_link_health_set_silent_threshold(uint8_t intervals) {
    if (intervals > 0) {
        g_link_ctx.silent_threshold_intervals = intervals;
        SX1262_LOG_INFO("[LINK] Silent threshold set to %d intervals\n", intervals);
    }
}

void sx1262_link_health_set_jitter_threshold(uint8_t percent) {
    g_link_ctx.jitter_threshold_percent = percent;
    SX1262_LOG_INFO("[LINK] Jitter threshold set to %d%%\n", percent);
}

void sx1262_link_health_set_rssi_threshold(int16_t rssi_dbm) {
    g_link_ctx.rssi_threshold_dbm = rssi_dbm;
    SX1262_LOG_INFO("[LINK] RSSI threshold set to %d dBm\n", rssi_dbm);
}

sx1262_link_result_t sx1262_link_health_set_node_interval(
    uint8_t node_id,
    uint32_t expected_interval_ms
) {
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        return SX1262_LINK_ERROR_NODE_NOT_FOUND;
    }
    
    node->expected_interval_ms = expected_interval_ms;
    return SX1262_LINK_OK;
}

// ============================================================================
// STATUS QUERIES
// ============================================================================

sx1262_link_result_t sx1262_link_health_get_node_stats(
    uint8_t node_id,
    sx1262_node_stats_t* stats
) {
    if (stats == NULL) {
        return SX1262_LINK_ERROR_INVALID_PARAM;
    }
    
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        return SX1262_LINK_ERROR_NODE_NOT_FOUND;
    }
    
    stats->node_id = node->node_id;
    stats->status = node->status;
    stats->expected_interval_ms = node->expected_interval_ms;
    stats->last_seen_ms = node->last_seen_ms;
    stats->last_interval_ms = node->last_interval_ms;
    stats->first_seen_ms = node->first_seen_ms;
    stats->packets_received = node->packets_received;
    stats->missed_intervals = node->consecutive_missed;
    stats->total_missed_intervals = node->total_missed;
    
    // Calculate expected packets
    if (node->first_seen_ms > 0 && node->expected_interval_ms > 0) {
        uint32_t time_since_first = LINK_GET_TIME_MS() - node->first_seen_ms;
        stats->packets_expected = (time_since_first / node->expected_interval_ms) + 1;
    } else {
        stats->packets_expected = 0;
    }
    
    stats->rssi_avg = calculate_rssi_avg(node);
    stats->rssi_min = node->rssi_min;
    stats->rssi_max = node->rssi_max;
    stats->snr_avg = calculate_snr_avg(node);
    stats->snr_min = node->snr_min;
    stats->snr_max = node->snr_max;
    
    stats->alarm_active = node->alarm_active;
    stats->last_alarm = node->last_alarm_type;
    stats->last_alarm_ms = node->last_alarm_ms;
    
    return SX1262_LINK_OK;
}

sx1262_link_result_t sx1262_link_health_get_network_status(
    uint8_t* out_total_nodes,
    uint8_t* out_healthy_nodes,
    uint8_t* out_warning_nodes,
    uint8_t* out_critical_nodes
) {
    uint8_t total = 0, healthy = 0, warning = 0, critical = 0;
    
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        link_node_t* node = &g_link_ctx.nodes[i];
        if (!node->active) continue;
        
        total++;
        
        switch (node->status) {
            case SX1262_NODE_STATUS_HEALTHY:
                healthy++;
                break;
            case SX1262_NODE_STATUS_WARNING:
                warning++;
                break;
            case SX1262_NODE_STATUS_CRITICAL:
            case SX1262_NODE_STATUS_OFFLINE:
                critical++;
                break;
            default:
                break;
        }
    }
    
    if (out_total_nodes) *out_total_nodes = total;
    if (out_healthy_nodes) *out_healthy_nodes = healthy;
    if (out_warning_nodes) *out_warning_nodes = warning;
    if (out_critical_nodes) *out_critical_nodes = critical;
    
    return SX1262_LINK_OK;
}

sx1262_link_result_t sx1262_link_health_get_alarmed_nodes(
    uint8_t* node_ids,
    uint8_t max_count,
    uint8_t* out_count
) {
    if (node_ids == NULL || out_count == NULL) {
        return SX1262_LINK_ERROR_INVALID_PARAM;
    }
    
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < g_link_ctx.max_nodes && count < max_count; i++) {
        link_node_t* node = &g_link_ctx.nodes[i];
        if (node->active && node->alarm_active) {
            node_ids[count++] = node->node_id;
        }
    }
    
    *out_count = count;
    return SX1262_LINK_OK;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

sx1262_link_result_t sx1262_link_health_clear_alarm(uint8_t node_id) {
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        return SX1262_LINK_ERROR_NODE_NOT_FOUND;
    }
    
    node->alarm_active = false;
    node->status = SX1262_NODE_STATUS_HEALTHY;
    
    return SX1262_LINK_OK;
}

void sx1262_link_health_clear_all_alarms(void) {
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        if (g_link_ctx.nodes[i].active) {
            g_link_ctx.nodes[i].alarm_active = false;
            g_link_ctx.nodes[i].status = SX1262_NODE_STATUS_HEALTHY;
        }
    }
}

sx1262_link_result_t sx1262_link_health_reset_node_stats(uint8_t node_id) {
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        return SX1262_LINK_ERROR_NODE_NOT_FOUND;
    }
    
    // Keep registration, reset stats
    uint8_t id = node->node_id;
    uint32_t interval = node->expected_interval_ms;
    bool active = node->active;
    
    memset(node, 0, sizeof(link_node_t));
    
    node->active = active;
    node->node_id = id;
    node->expected_interval_ms = interval;
    node->status = SX1262_NODE_STATUS_UNKNOWN;
    
    return SX1262_LINK_OK;
}

void sx1262_link_health_reset_all_stats(void) {
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        if (g_link_ctx.nodes[i].active) {
            sx1262_link_health_reset_node_stats(g_link_ctx.nodes[i].node_id);
        }
    }
}

const char* sx1262_link_alarm_type_to_string(sx1262_link_alarm_type_t type) {
    switch (type) {
        case SX1262_LINK_ALARM_SILENT:       return "SILENT";
        case SX1262_LINK_ALARM_IRREGULAR:    return "IRREGULAR";
        case SX1262_LINK_ALARM_WEAK_SIGNAL:  return "WEAK_SIGNAL";
        case SX1262_LINK_ALARM_RECOVERED:    return "RECOVERED";
        default:                              return "UNKNOWN";
    }
}

const char* sx1262_link_severity_to_string(sx1262_link_severity_t severity) {
    switch (severity) {
        case SX1262_LINK_SEVERITY_INFO:      return "INFO";
        case SX1262_LINK_SEVERITY_WARNING:   return "WARNING";
        case SX1262_LINK_SEVERITY_CRITICAL:  return "CRITICAL";
        default:                              return "UNKNOWN";
    }
}

const char* sx1262_link_node_status_to_string(sx1262_node_status_t status) {
    switch (status) {
        case SX1262_NODE_STATUS_UNKNOWN:     return "UNKNOWN";
        case SX1262_NODE_STATUS_HEALTHY:     return "HEALTHY";
        case SX1262_NODE_STATUS_WARNING:     return "WARNING";
        case SX1262_NODE_STATUS_CRITICAL:    return "CRITICAL";
        case SX1262_NODE_STATUS_OFFLINE:     return "OFFLINE";
        default:                              return "UNKNOWN";
    }
}

void sx1262_link_health_print_status(void) {
    if (!g_link_ctx.initialized) {
        SX1262_LOG_INFO("[LINK] Not initialized\n");
        return;
    }
    
    uint8_t total, healthy, warning, critical;
    sx1262_link_health_get_network_status(&total, &healthy, &warning, &critical);
    
    SX1262_LOG_INFO("[LINK] === Network Status ===\n");
    SX1262_LOG_INFO("[LINK] Nodes: %d total, %d healthy, %d warning, %d critical\n",
                   total, healthy, warning, critical);
    
    for (uint8_t i = 0; i < g_link_ctx.max_nodes; i++) {
        link_node_t* node = &g_link_ctx.nodes[i];
        if (!node->active) continue;
        
        uint32_t age_ms = 0;
        if (node->last_seen_ms > 0) {
            age_ms = LINK_GET_TIME_MS() - node->last_seen_ms;
        }
        
        SX1262_LOG_INFO("[LINK]   Node 0x%02X: %s, RSSI=%d dBm, last seen %lu ms ago, pkts=%lu\n",
                       node->node_id,
                       sx1262_link_node_status_to_string(node->status),
                       calculate_rssi_avg(node),
                       age_ms,
                       node->packets_received);
    }
    
    SX1262_LOG_INFO("[LINK] =========================\n");
}

void sx1262_link_health_print_node_status(uint8_t node_id) {
    link_node_t* node = find_node(node_id);
    if (node == NULL) {
        SX1262_LOG_INFO("[LINK] Node 0x%02X not found\n", node_id);
        return;
    }
    
    sx1262_node_stats_t stats;
    sx1262_link_health_get_node_stats(node_id, &stats);
    
    SX1262_LOG_INFO("[LINK] === Node 0x%02X Status ===\n", node_id);
    SX1262_LOG_INFO("[LINK]   Status: %s\n", sx1262_link_node_status_to_string(stats.status));
    SX1262_LOG_INFO("[LINK]   Expected interval: %lu ms\n", stats.expected_interval_ms);
    SX1262_LOG_INFO("[LINK]   Last seen: %lu ms ago\n", LINK_GET_TIME_MS() - stats.last_seen_ms);
    SX1262_LOG_INFO("[LINK]   Packets: %lu received, %lu expected\n", 
                   stats.packets_received, stats.packets_expected);
    SX1262_LOG_INFO("[LINK]   Missed intervals: %lu current, %lu total\n",
                   stats.missed_intervals, stats.total_missed_intervals);
    SX1262_LOG_INFO("[LINK]   RSSI: avg=%d, min=%d, max=%d dBm\n",
                   stats.rssi_avg, stats.rssi_min, stats.rssi_max);
    SX1262_LOG_INFO("[LINK]   SNR: avg=%d, min=%d, max=%d dB\n",
                   stats.snr_avg, stats.snr_min, stats.snr_max);
    SX1262_LOG_INFO("[LINK]   Alarm: %s\n", stats.alarm_active ? "ACTIVE" : "none");
    
    if (stats.alarm_active) {
        SX1262_LOG_INFO("[LINK]   Last alarm: %s at %lu ms\n",
                       sx1262_link_alarm_type_to_string(stats.last_alarm),
                       stats.last_alarm_ms);
    }
    
    SX1262_LOG_INFO("[LINK] ============================\n");
}