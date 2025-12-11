#ifndef SX1262_LINK_HEALTH_H
#define SX1262_LINK_HEALTH_H

/**
 * @file sx1262_link_health.h
 * @brief Link Health Monitor for multi-node LoRa networks
 * 
 * This module tracks the health of communication links with remote nodes,
 * detecting silent nodes, irregular transmission patterns, and link quality
 * degradation. It is designed as a standalone, optional layer for gateway
 * applications.
 * 
 * Features:
 *   - Per-node tracking (last seen, RSSI, SNR, packet counts)
 *   - Silent node detection (no packets for N intervals)
 *   - Irregular transmission detection (timing jitter)
 *   - Link quality trending (RSSI/SNR averages)
 *   - Callback-based alarms for immediate notification
 * 
 * Typical Usage (Gateway):
 * @code
 *   // Initialization
 *   sx1262_link_health_init(20, on_alarm_callback);  // Track up to 20 nodes
 *   
 *   // Configure expected nodes
 *   sx1262_link_health_register_node(0x01, 900000);  // Node 1, 15-min interval
 *   sx1262_link_health_register_node(0x02, 900000);  // Node 2, 15-min interval
 *   
 *   // In RX loop - record successful receptions
 *   sx1262_link_health_record_rx(node_id, rssi, snr);
 *   
 *   // Periodic check (call from main loop or timer)
 *   sx1262_link_health_check_timeouts();
 * @endcode
 * 
 * Alarm Conditions:
 *   1. SILENT: Node hasn't transmitted for 3× expected interval
 *   2. IRREGULAR: Transmission timing varies by >50% from expected
 *   3. WEAK_SIGNAL: RSSI below configured threshold
 *   4. RECOVERED: Previously alarmed node is now communicating normally
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
 * @brief Maximum number of nodes to track
 * 
 * Each node uses ~48 bytes of RAM. Adjust based on your network size.
 */
#ifndef SX1262_LINK_MAX_NODES
#define SX1262_LINK_MAX_NODES           32
#endif

/**
 * @brief Number of RSSI/SNR samples to average
 * 
 * Higher values give smoother averages but slower response to changes.
 */
#ifndef SX1262_LINK_SIGNAL_HISTORY_SIZE
#define SX1262_LINK_SIGNAL_HISTORY_SIZE 8
#endif

/**
 * @brief Default number of missed intervals before SILENT alarm
 */
#define SX1262_LINK_DEFAULT_SILENT_INTERVALS    3

/**
 * @brief Default timing jitter threshold (percentage)
 * 
 * If actual interval differs from expected by more than this %, 
 * an IRREGULAR alarm is raised.
 */
#define SX1262_LINK_DEFAULT_JITTER_THRESHOLD_PCT    50

/**
 * @brief Default weak signal threshold (dBm)
 */
#define SX1262_LINK_DEFAULT_RSSI_THRESHOLD_DBM      -110

// ============================================================================
// ALARM TYPES
// ============================================================================

/**
 * @brief Alarm type enumeration
 */
typedef enum {
    SX1262_LINK_ALARM_SILENT = 0,       ///< Node hasn't transmitted in expected window
    SX1262_LINK_ALARM_IRREGULAR,         ///< Node transmitting at unexpected times
    SX1262_LINK_ALARM_WEAK_SIGNAL,       ///< RSSI below threshold
    SX1262_LINK_ALARM_RECOVERED          ///< Previously alarmed node recovered
} sx1262_link_alarm_type_t;

/**
 * @brief Alarm severity levels
 */
typedef enum {
    SX1262_LINK_SEVERITY_INFO = 0,      ///< Informational (e.g., recovered)
    SX1262_LINK_SEVERITY_WARNING,        ///< Warning (e.g., weak signal)
    SX1262_LINK_SEVERITY_CRITICAL        ///< Critical (e.g., silent node)
} sx1262_link_severity_t;

/**
 * @brief Alarm information structure
 * 
 * Passed to the alarm callback with details about the event.
 */
typedef struct {
    uint8_t node_id;                    ///< ID of the node that triggered alarm
    sx1262_link_alarm_type_t type;      ///< Type of alarm
    sx1262_link_severity_t severity;    ///< Severity level
    uint32_t timestamp_ms;              ///< When alarm occurred
    
    // Context-specific data
    union {
        struct {
            uint32_t last_seen_ms;      ///< When node was last heard
            uint32_t silent_duration_ms;///< How long node has been silent
            uint8_t missed_intervals;   ///< Number of missed transmission intervals
        } silent;
        
        struct {
            uint32_t expected_interval_ms;  ///< Expected interval
            uint32_t actual_interval_ms;    ///< Actual measured interval
            int32_t jitter_percent;         ///< Timing deviation percentage
        } irregular;
        
        struct {
            int16_t current_rssi;       ///< Current RSSI (dBm)
            int16_t threshold_rssi;     ///< Threshold that was crossed
            int8_t current_snr;         ///< Current SNR (dB)
        } weak_signal;
    } data;
} sx1262_link_alarm_t;

/**
 * @brief Alarm callback function type
 * 
 * User-provided function called when an alarm condition is detected.
 * 
 * @param alarm  Pointer to alarm information (valid only during callback)
 * 
 * @note Keep callback execution brief to avoid blocking the main loop.
 *       For complex handling, set a flag and process in main loop.
 */
typedef void (*sx1262_link_alarm_callback_t)(const sx1262_link_alarm_t* alarm);

// ============================================================================
// RETURN CODES
// ============================================================================

/**
 * @brief Link health monitor return codes
 */
typedef enum {
    SX1262_LINK_OK = 0,                     ///< Operation successful
    SX1262_LINK_ERROR_INVALID_PARAM = -1,   ///< Invalid parameter
    SX1262_LINK_ERROR_NOT_INITIALIZED = -2, ///< Module not initialized
    SX1262_LINK_ERROR_NODE_NOT_FOUND = -3,  ///< Node ID not registered
    SX1262_LINK_ERROR_TABLE_FULL = -4,      ///< Node table is full
    SX1262_LINK_ERROR_ALREADY_EXISTS = -5   ///< Node ID already registered
} sx1262_link_result_t;

// ============================================================================
// NODE STATUS
// ============================================================================

/**
 * @brief Node health status
 */
typedef enum {
    SX1262_NODE_STATUS_UNKNOWN = 0,     ///< Never heard from
    SX1262_NODE_STATUS_HEALTHY,          ///< Communicating normally
    SX1262_NODE_STATUS_WARNING,          ///< Minor issues (weak signal, slight jitter)
    SX1262_NODE_STATUS_CRITICAL,         ///< Major issues (silent, very irregular)
    SX1262_NODE_STATUS_OFFLINE           ///< Declared offline after extended silence
} sx1262_node_status_t;

/**
 * @brief Per-node statistics
 */
typedef struct {
    uint8_t node_id;                    ///< Node identifier
    sx1262_node_status_t status;        ///< Current health status
    
    // Timing
    uint32_t expected_interval_ms;      ///< Configured expected interval
    uint32_t last_seen_ms;              ///< Timestamp of last reception
    uint32_t last_interval_ms;          ///< Measured interval between last two packets
    uint32_t first_seen_ms;             ///< Timestamp of first ever reception
    
    // Packet counts
    uint32_t packets_received;          ///< Total packets received from this node
    uint32_t packets_expected;          ///< Expected packets based on time since first seen
    uint32_t missed_intervals;          ///< Current consecutive missed intervals
    uint32_t total_missed_intervals;    ///< Total missed intervals since registration
    
    // Signal quality (averaged)
    int16_t rssi_avg;                   ///< Average RSSI (dBm, scaled by 10)
    int16_t rssi_min;                   ///< Minimum RSSI seen
    int16_t rssi_max;                   ///< Maximum RSSI seen
    int8_t snr_avg;                     ///< Average SNR (dB)
    int8_t snr_min;                     ///< Minimum SNR seen
    int8_t snr_max;                     ///< Maximum SNR seen
    
    // Alarm state
    bool alarm_active;                  ///< Currently in alarm state
    sx1262_link_alarm_type_t last_alarm;///< Type of last alarm
    uint32_t last_alarm_ms;             ///< When last alarm was raised
} sx1262_node_stats_t;

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize link health monitor
 * 
 * @param max_nodes  Maximum number of nodes to track (1 to SX1262_LINK_MAX_NODES)
 * @param callback   Alarm callback function (can be NULL to disable callbacks)
 * @return SX1262_LINK_OK on success
 * 
 * Example:
 * @code
 *   void on_alarm(const sx1262_link_alarm_t* alarm) {
 *       Serial.printf("ALARM: Node %d - %s\n", alarm->node_id, 
 *                     sx1262_link_alarm_to_string(alarm->type));
 *   }
 *   
 *   sx1262_link_health_init(20, on_alarm);
 * @endcode
 */
sx1262_link_result_t sx1262_link_health_init(uint8_t max_nodes, sx1262_link_alarm_callback_t callback);

/**
 * @brief Deinitialize link health monitor
 */
void sx1262_link_health_deinit(void);

/**
 * @brief Check if module is initialized
 * 
 * @return true if initialized
 */
bool sx1262_link_health_is_initialized(void);

/**
 * @brief Set alarm callback
 * 
 * Can be called after init to change or disable callback.
 * 
 * @param callback  New callback function (NULL to disable)
 */
void sx1262_link_health_set_callback(sx1262_link_alarm_callback_t callback);

// ============================================================================
// NODE REGISTRATION
// ============================================================================

/**
 * @brief Register a node to track
 * 
 * @param node_id              Unique node identifier (1-254)
 * @param expected_interval_ms Expected transmission interval in milliseconds
 * @return SX1262_LINK_OK on success
 * 
 * Example:
 * @code
 *   // Register sensor nodes with 15-minute interval
 *   sx1262_link_health_register_node(0x01, 900000);
 *   sx1262_link_health_register_node(0x02, 900000);
 * @endcode
 */
sx1262_link_result_t sx1262_link_health_register_node(
    uint8_t node_id,
    uint32_t expected_interval_ms
);

/**
 * @brief Unregister a node
 * 
 * @param node_id  Node to remove from tracking
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_unregister_node(uint8_t node_id);

/**
 * @brief Check if a node is registered
 * 
 * @param node_id  Node identifier to check
 * @return true if registered
 */
bool sx1262_link_health_is_node_registered(uint8_t node_id);

/**
 * @brief Get number of registered nodes
 * 
 * @return Number of currently registered nodes
 */
uint8_t sx1262_link_health_get_node_count(void);

// ============================================================================
// RECEPTION RECORDING
// ============================================================================

/**
 * @brief Record successful packet reception from a node
 * 
 * Call this after receiving and validating a packet from a node.
 * Updates last seen time, signal stats, and clears any active alarms.
 * 
 * @param node_id  ID of the node that sent the packet
 * @param rssi     Received signal strength (dBm, typically -140 to 0)
 * @param snr      Signal-to-noise ratio (dB, typically -20 to +15)
 * @return SX1262_LINK_OK on success
 * 
 * Example:
 * @code
 *   if (sx1262_receive(...) == SX1262_OK) {
 *       uint8_t sender_id = rx_buffer[0];
 *       sx1262_link_health_record_rx(sender_id, rx_result.rssi_pkt, rx_result.snr_pkt);
 *   }
 * @endcode
 */
sx1262_link_result_t sx1262_link_health_record_rx(
    uint8_t node_id,
    int16_t rssi,
    int8_t snr
);

/**
 * @brief Record reception from unregistered node (auto-register)
 * 
 * If node_id is not registered, automatically registers it with the
 * specified default interval.
 * 
 * @param node_id          Node identifier
 * @param rssi             RSSI value
 * @param snr              SNR value
 * @param default_interval Default expected interval for auto-registration
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_record_rx_auto(
    uint8_t node_id,
    int16_t rssi,
    int8_t snr,
    uint32_t default_interval_ms
);

// ============================================================================
// TIMEOUT CHECKING
// ============================================================================

/**
 * @brief Check all nodes for timeout conditions
 * 
 * Call this periodically (e.g., every second or every minute) to detect
 * silent nodes and trigger alarms. This is the main health check function.
 * 
 * Checks performed:
 *   1. Silent detection: No packet for (expected_interval × silent_multiplier)
 *   2. Irregular detection: Timing jitter exceeds threshold
 *   3. Weak signal: Average RSSI below threshold
 * 
 * @return Number of nodes currently in alarm state
 * 
 * Example:
 * @code
 *   void loop() {
 *       // ... RX handling ...
 *       
 *       // Check for timeouts every second
 *       static uint32_t last_check = 0;
 *       if (millis() - last_check >= 1000) {
 *           last_check = millis();
 *           sx1262_link_health_check_timeouts();
 *       }
 *   }
 * @endcode
 */
uint8_t sx1262_link_health_check_timeouts(void);

// ============================================================================
// CONFIGURATION
// ============================================================================

/**
 * @brief Set silent detection threshold
 * 
 * @param intervals  Number of missed intervals before SILENT alarm (default: 3)
 */
void sx1262_link_health_set_silent_threshold(uint8_t intervals);

/**
 * @brief Set timing jitter threshold
 * 
 * @param percent  Jitter threshold percentage (default: 50)
 */
void sx1262_link_health_set_jitter_threshold(uint8_t percent);

/**
 * @brief Set weak signal threshold
 * 
 * @param rssi_dbm  RSSI threshold in dBm (default: -110)
 */
void sx1262_link_health_set_rssi_threshold(int16_t rssi_dbm);

/**
 * @brief Update expected interval for a node
 * 
 * @param node_id              Node identifier
 * @param expected_interval_ms New expected interval
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_set_node_interval(
    uint8_t node_id,
    uint32_t expected_interval_ms
);

// ============================================================================
// STATUS QUERIES
// ============================================================================

/**
 * @brief Get status of a specific node
 * 
 * @param node_id  Node identifier
 * @param stats    Pointer to stats structure to fill
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_get_node_stats(
    uint8_t node_id,
    sx1262_node_stats_t* stats
);

/**
 * @brief Get overall network health status
 * 
 * @param out_total_nodes    Pointer to receive total registered nodes
 * @param out_healthy_nodes  Pointer to receive healthy node count
 * @param out_warning_nodes  Pointer to receive warning node count
 * @param out_critical_nodes Pointer to receive critical node count
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_get_network_status(
    uint8_t* out_total_nodes,
    uint8_t* out_healthy_nodes,
    uint8_t* out_warning_nodes,
    uint8_t* out_critical_nodes
);

/**
 * @brief Get list of nodes in alarm state
 * 
 * @param node_ids      Array to receive node IDs in alarm
 * @param max_count     Maximum number of IDs to return
 * @param out_count     Pointer to receive actual count
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_get_alarmed_nodes(
    uint8_t* node_ids,
    uint8_t max_count,
    uint8_t* out_count
);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Clear alarm state for a node
 * 
 * Manually acknowledge/clear an alarm without waiting for recovery.
 * 
 * @param node_id  Node identifier
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_clear_alarm(uint8_t node_id);

/**
 * @brief Clear all alarms
 */
void sx1262_link_health_clear_all_alarms(void);

/**
 * @brief Reset statistics for a node
 * 
 * @param node_id  Node identifier
 * @return SX1262_LINK_OK on success
 */
sx1262_link_result_t sx1262_link_health_reset_node_stats(uint8_t node_id);

/**
 * @brief Reset all statistics
 */
void sx1262_link_health_reset_all_stats(void);

/**
 * @brief Get alarm type as string
 * 
 * @param type  Alarm type
 * @return Human-readable string
 */
const char* sx1262_link_alarm_type_to_string(sx1262_link_alarm_type_t type);

/**
 * @brief Get severity as string
 * 
 * @param severity  Severity level
 * @return Human-readable string
 */
const char* sx1262_link_severity_to_string(sx1262_link_severity_t severity);

/**
 * @brief Get node status as string
 * 
 * @param status  Node status
 * @return Human-readable string
 */
const char* sx1262_link_node_status_to_string(sx1262_node_status_t status);

/**
 * @brief Print status of all nodes to log
 */
void sx1262_link_health_print_status(void);

/**
 * @brief Print detailed status of a single node
 * 
 * @param node_id  Node identifier
 */
void sx1262_link_health_print_node_status(uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif // SX1262_LINK_HEALTH_H
