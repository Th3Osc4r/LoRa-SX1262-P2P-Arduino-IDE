// sx1262_driver.cpp
// Integrated Phase 1.1, 1.2, 1.3 fixes
// Phase 3: Hardware Interrupts (Option 1: Blocking/Yielding)
//
// VERSION 2 CHANGES:
// - sx1262_receive(): Added RX timing capture (rx_start_ms, rx_done_ms, rx_duration_ms)
//   These fields were defined in sx1262_rx_result_t but never populated.
//   Now properly measures actual RX operation time for diagnostics.
//
// VERIFIED WORKING: Diagnostic testing confirmed timing fix is effective
//

#include "sx1262_driver.h"
#include "sx1262_mutex_guard.h"  // Phase 1.1: RAII Mutex Guard
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

// ============================================================================
// GLOBAL DRIVER CONTEXT
// ============================================================================
sx1262_driver_context_t g_driver_ctx;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
sx1262_result_t sx1262_emergency_reset();
static void log_state_transition(sx1262_state_t from, sx1262_state_t to, bool success, sx1262_result_t err);
bool sx1262_is_valid_transition(sx1262_state_t from, sx1262_state_t to);
uint32_t sx1262_get_time_in_state();
static sx1262_result_t _internal_set_state(sx1262_state_t new_state);  // Phase 3.1: For turnaround functions

// ============================================================================
// INTERRUPT SERVICE ROUTINE (Phase 3)
// ============================================================================
static void IRAM_ATTR sx1262_dio1_isr(void* arg) {
    sx1262_driver_context_t* ctx = (sx1262_driver_context_t*)arg;
    if (ctx && ctx->blocking_task_handle) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR((TaskHandle_t)ctx->blocking_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// ============================================================================
// STATE MACHINE TRANSITION TABLE
// ============================================================================
static const bool STATE_TRANSITION_TABLE[8][8] = {
    // TO:  UNINIT  SLEEP   STBY_RC STBY_X  FS      TX      RX      ERROR
    /* FROM UNINIT */   {false,  false,  true,   false,  false,  false,  false,  true},
    /* FROM SLEEP */    {false,  false,  true,   false,  false,  false,  false,  true},
    /* FROM STBY_RC */  {false,  true,   false,  true,   true,   true,   true,   true},
    /* FROM STBY_X */   {false,  true,   true,   false,  true,   true,   true,   true},
    /* FROM FS */       {false,  false,  true,   true,   false,  true,   true,   true},
    /* FROM TX */       {false,  false,  true,   false,  true,   false,  false,  true},
    /* FROM RX */       {false,  false,  true,   false,  true,   false,  false,  true},
    /* FROM ERROR */    {true,   false,  true,   false,  false,  false,  false,  false}
};

// ============================================================================
// HELPER FUNCTIONS - STRINGS
// ============================================================================
const char* sx1262_state_to_string(sx1262_state_t state) {
    switch (state) {
        case SX1262_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case SX1262_STATE_SLEEP:         return "SLEEP";
        case SX1262_STATE_STANDBY_RC:    return "STANDBY_RC";
        case SX1262_STATE_STANDBY_XOSC:  return "STANDBY_XOSC";
        case SX1262_STATE_FS:            return "FS";
        case SX1262_STATE_TX:            return "TX";
        case SX1262_STATE_RX:            return "RX";
        case SX1262_STATE_CAD:           return "CAD";
        case SX1262_STATE_ERROR:         return "ERROR";
        default:                         return "UNKNOWN";
    }
}

const char* sx1262_error_to_string(sx1262_result_t error) {
    switch (error) {
        case SX1262_OK:                      return "OK";
        case SX1262_ERROR_INVALID_STATE:     return "INVALID_STATE";
        case SX1262_ERROR_INVALID_PARAM:     return "INVALID_PARAM";
        case SX1262_ERROR_TIMEOUT:           return "TIMEOUT";
        case SX1262_ERROR_SPI:               return "SPI_ERROR";
        case SX1262_ERROR_BUSY_TIMEOUT:      return "BUSY_TIMEOUT";
        case SX1262_ERROR_NOT_INITIALIZED:   return "NOT_INITIALIZED";
        case SX1262_ERROR_MUTEX:             return "MUTEX_ERROR";
        case SX1262_ERROR_WATCHDOG:          return "WATCHDOG_TIMEOUT";
        case SX1262_ERROR_HARDWARE:          return "HARDWARE_ERROR";
        default:                             return "UNKNOWN_ERROR";
    }
}

// ============================================================================
// INTERNAL HELPER - LOGGING
// ============================================================================
static void log_state_transition(sx1262_state_t from, sx1262_state_t to, bool success, sx1262_result_t err) {
    sx1262_state_transition_t* entry = &g_driver_ctx.transition_history[g_driver_ctx.transition_history_idx];
    entry->from_state = from; entry->to_state = to; entry->timestamp_ms = hal_get_time_ms();
    entry->success = success; entry->error_code = err;
    g_driver_ctx.transition_history_idx = (g_driver_ctx.transition_history_idx + 1) % 16;
    
    g_driver_ctx.total_state_transitions++;
    if (!success) g_driver_ctx.failed_state_transitions++;
    
    if (success) { SX1262_LOG_STATE_TRANSITION(from, to); }
    else { SX1262_LOG_DRIVER("State transition FAILED: %s -> %s (%s)\n", sx1262_state_to_string(from), sx1262_state_to_string(to), sx1262_error_to_string(err)); }
}


// ============================================================================
// HELPER IMPLEMENTATIONS (Must be present for Linker)
// ============================================================================

bool sx1262_is_valid_transition(sx1262_state_t from, sx1262_state_t to) {
    if (from >= 8 || to >= 8) return false;
    return STATE_TRANSITION_TABLE[from][to];
}

uint32_t sx1262_get_time_in_state() {
    uint32_t curr = hal_get_time_ms();
    // Handle overflow wrap-around
    if (curr >= g_driver_ctx.state_entry_time_ms) {
        return curr - g_driver_ctx.state_entry_time_ms;
    } else {
        return (0xFFFFFFFF - g_driver_ctx.state_entry_time_ms) + curr + 1;
    }
}

uint8_t sx1262_get_transition_history(sx1262_state_transition_t* history, uint8_t count) {
    if (!history) return 0;
    
    // Use Guard for thread safety
    SX1262MutexGuard guard(g_driver_ctx.state_mutex, 100);
    if (!guard.is_locked()) return 0;
    
    uint8_t c = (count > 16) ? 16 : count;
    memcpy(history, g_driver_ctx.transition_history, c * sizeof(sx1262_state_transition_t));
    return c;
}

// ============================================================================
// INTERNAL HELPER - EXECUTE HARDWARE STATE CHANGE
// ============================================================================
static sx1262_result_t execute_hardware_transition(sx1262_state_t to_state) {
    spi_result_t spi_res;
    
    // Wake from sleep if needed (Hardware Wakeup)
    if (g_driver_ctx.current_state == SX1262_STATE_SLEEP && to_state != SX1262_STATE_SLEEP) {
        hal_spi_cs_assert(1); hal_delay_us(10); hal_spi_cs_assert(0); hal_delay_us(100);
    }
    
    switch (to_state) {
        case SX1262_STATE_SLEEP: {
            hal_spi_cs_assert(1);
            uint8_t cmd[2] = {SX1262_OP_SET_SLEEP, 0x04}; // Warm start
            hal_spi_transfer(cmd, NULL, 2);
            hal_spi_cs_assert(0);
            spi_res = SPI_OK;
            break;
        }
        case SX1262_STATE_STANDBY_RC: {
            // Robust wake-up sequence
            for (int i = 0; i < 3; i++) { uint8_t s; spi_get_status(&s); hal_delay_ms(1); }
            uint32_t start = millis();
            while (millis() - start < 200) { if (!hal_gpio_read(PIN_SX1262_BUSY)) break; }
            spi_clear_device_errors();
            uint8_t mode = 0x00; // RC
            spi_res = spi_cmd(SX1262_OP_SET_STANDBY, &mode, 1, NULL, 0, TIMING_BUSY_POLL_TIMEOUT_MS, NULL);
            break;
        }
        case SX1262_STATE_STANDBY_XOSC:
            spi_res = spi_set_standby(SX1262_STDBY_XOSC);
            break;
        case SX1262_STATE_FS: {
            uint8_t cmd = SX1262_OP_SET_FS;
            hal_spi_cs_assert(1); hal_spi_transfer(&cmd, NULL, 1); hal_spi_cs_assert(0);
            spi_res = SPI_OK;
            break;
        }
        case SX1262_STATE_TX:
            // CRITICAL: DO NOT SEND SetTx HERE! 
            // It is sent with specific timeout later in sx1262_transmit.
            // This case just validates the transition logic.
            spi_res = SPI_OK; 
            break;
        case SX1262_STATE_RX: {
            // Continuous RX (Infinite Timeout) - Base for RX operations
            uint8_t cmd[4] = {SX1262_OP_SET_RX, 0xFF, 0xFF, 0xFF};
            hal_spi_cs_assert(1); hal_spi_transfer(cmd, NULL, 4); hal_spi_cs_assert(0);
            spi_res = SPI_OK;
            break;
        }
        default: return SX1262_ERROR_INVALID_STATE;
    }
    
    return (spi_res == SPI_OK) ? SX1262_OK : ((spi_res == SPI_ERROR_BUSY_TIMEOUT) ? SX1262_ERROR_BUSY_TIMEOUT : SX1262_ERROR_SPI);
}

// ============================================================================
// PHASE 1.2: INTERNAL ATOMIC STATE TRANSITION
// ============================================================================
// Assumes MUTEX IS ALREADY HELD by the caller
static sx1262_result_t _internal_set_state(sx1262_state_t new_state) {
    sx1262_state_t current = g_driver_ctx.current_state;
    
    // Optimization: Don't transition if already there
    if (current == new_state) {
        return SX1262_OK;
    }
    
    // 1. Validate Transition
    if (!sx1262_is_valid_transition(current, new_state)) {
        // Watchdog Logic
        if (g_driver_ctx.watchdog.enabled) {
            g_driver_ctx.watchdog.consecutive_failures++;
            if (g_driver_ctx.watchdog.consecutive_failures >= g_driver_ctx.watchdog.max_failures) {
                SX1262_LOG_DRIVER("[WATCHDOG] Max failures reached. Triggering recovery.\n");
                sx1262_emergency_reset();
                return SX1262_ERROR_WATCHDOG;
            }
        }
        
        log_state_transition(current, new_state, false, SX1262_ERROR_INVALID_STATE);
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // 2. Execute Hardware Command
    sx1262_result_t hw_res = execute_hardware_transition(new_state);
    
    // 3. Update Software State
    if (hw_res == SX1262_OK) {
        g_driver_ctx.previous_state = current;
        g_driver_ctx.current_state = new_state;
        g_driver_ctx.state_entry_time_ms = hal_get_time_ms();
        
        sx1262_watchdog_feed(); // Success feeds the dog
        log_state_transition(current, new_state, true, SX1262_OK);
    } else {
        // Handle Hardware Failure
        g_driver_ctx.last_error = hw_res;
        g_driver_ctx.last_error_function = "_internal_set_state";
        g_driver_ctx.last_error_timestamp_ms = hal_get_time_ms();
        
        log_state_transition(current, new_state, false, hw_res);
    }
    
    return hw_res;
}

// ============================================================================
// WATCHDOG HELPERS
// ============================================================================
sx1262_result_t sx1262_watchdog_init(uint8_t max_failures) {
    g_driver_ctx.watchdog.enabled = false;
    g_driver_ctx.watchdog.max_failures = max_failures;
    g_driver_ctx.watchdog.consecutive_failures = 0;
    return SX1262_OK;
}
void sx1262_watchdog_enable(bool enable) {
    g_driver_ctx.watchdog.enabled = enable;
    if (enable) g_driver_ctx.watchdog.consecutive_failures = 0;
}
void sx1262_watchdog_feed() { g_driver_ctx.watchdog.consecutive_failures = 0; }
bool sx1262_watchdog_get_status(uint8_t* consecutive_failures) {
    if (consecutive_failures) *consecutive_failures = g_driver_ctx.watchdog.consecutive_failures;
    return g_driver_ctx.watchdog.enabled;
}

// ============================================================================
// MUTEX OPERATIONS
// ============================================================================
sx1262_result_t sx1262_mutex_lock(uint32_t timeout_ms) {
    if (!g_driver_ctx.mutex_initialized) return SX1262_ERROR_MUTEX;
    return (xSemaphoreTake((SemaphoreHandle_t)g_driver_ctx.state_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) ? SX1262_OK : SX1262_ERROR_MUTEX;
}

void sx1262_mutex_unlock() {
    if (g_driver_ctx.mutex_initialized) xSemaphoreGive((SemaphoreHandle_t)g_driver_ctx.state_mutex);
}

// ============================================================================
// PUBLIC API: STATE MANAGEMENT
// ============================================================================
sx1262_result_t sx1262_set_state(sx1262_state_t new_state) {
    if (!g_driver_ctx.hardware_initialized) return SX1262_ERROR_NOT_INITIALIZED;
    
    // Phase 1.1: Use Guard
    SX1262MutexGuard guard(g_driver_ctx.state_mutex);
    if (!guard.is_locked()) return SX1262_ERROR_MUTEX;
    
    // Phase 1.2: Use Internal Atomic Helper
    return _internal_set_state(new_state);
}

sx1262_state_t sx1262_get_state() {
    if (!g_driver_ctx.mutex_initialized) return g_driver_ctx.current_state;
    SX1262MutexGuard guard(g_driver_ctx.state_mutex, 100);
    if (!guard.is_locked()) return SX1262_STATE_ERROR;
    return g_driver_ctx.current_state;
}

// ============================================================================
// INITIALIZATION
// ============================================================================
sx1262_result_t sx1262_driver_init() {
    if (g_driver_ctx.hardware_initialized) {
        sx1262_driver_deinit(); hal_delay_ms(100);
    }
    memset(&g_driver_ctx, 0, sizeof(sx1262_driver_context_t));
    memset(&g_driver_ctx.config, 0, sizeof(sx1262_lora_config_t));
    g_driver_ctx.config_valid = false;
    g_driver_ctx.blocking_task_handle = NULL; // Initialize handle to NULL

    // 1. Create Mutex
    g_driver_ctx.state_mutex = (void*)xSemaphoreCreateMutex();
    if (!g_driver_ctx.state_mutex) return SX1262_ERROR_MUTEX;
    g_driver_ctx.mutex_initialized = true;

    // 2. HAL Init
    hal_timing_init();
    if (hal_spi_init(SPI_FREQUENCY_HZ, PIN_SX1262_CS) != HAL_OK) return SX1262_ERROR_HARDWARE;
    if (hal_reset_init() != HAL_OK) { hal_spi_deinit(); return SX1262_ERROR_HARDWARE; }
    if (hal_dio_init() != HAL_OK) { hal_reset_deinit(); hal_spi_deinit(); return SX1262_ERROR_HARDWARE; }
    hal_gpio_init_input(PIN_SX1262_BUSY, 0);
    
    // Phase 3: Attach Interrupt (This is the ONLY change to init)
    hal_gpio_attach_interrupt(PIN_SX1262_DIO1, sx1262_dio1_isr, &g_driver_ctx);

    // 3. SPI Init
    spi_result_t spi_res = spi_init(SPI_FREQUENCY_HZ);
    if (spi_res != SPI_OK) {
        spi_deinit(); hal_delay_ms(50);
        if (spi_init(SPI_FREQUENCY_HZ) != SPI_OK) return SX1262_ERROR_SPI;
    }
    g_driver_ctx.spi_initialized = true;

    // 4. Hardware Reset & Bootstrap
    hal_reset(); hal_delay_ms(10);
    
    // Bootstrapping manually
    uint8_t standby_rc = 0x00;
    spi_cmd(SX1262_OP_SET_STANDBY, &standby_rc, 1, NULL, 0, TIMING_BUSY_POLL_TIMEOUT_MS, NULL);
    hal_delay_ms(1);
    spi_clear_device_errors();

    // 5. TCXO Init (Standard 1.8V, 5ms delay from original driver)
    uint8_t tcxo_params[4] = {0x02, 0x00, 0x01, 0x40}; 
    spi_cmd(0x97, tcxo_params, 4, NULL, 0, TIMING_BUSY_POLL_TIMEOUT_MS, NULL);
    hal_delay_ms(10);
    
    // 6. Enable DIO2 as RF Switch Control
    uint8_t rf_switch_enable = 0x01;
    spi_cmd(0x9D, &rf_switch_enable, 1, NULL, 0, TIMING_BUSY_POLL_TIMEOUT_MS, NULL);
    hal_delay_ms(1);

    // 6. Verify
    uint8_t status;
    if (spi_get_status(&status) != SPI_OK) return SX1262_ERROR_HARDWARE;

    g_driver_ctx.hardware_initialized = true;
    g_driver_ctx.current_state = SX1262_STATE_STANDBY_RC;
    
    return SX1262_OK;
}

sx1262_result_t sx1262_driver_deinit() {
    if (!g_driver_ctx.hardware_initialized) return SX1262_OK;
    if (g_driver_ctx.current_state != SX1262_STATE_SLEEP) sx1262_set_state(SX1262_STATE_SLEEP);
    sx1262_watchdog_enable(false);
    if (g_driver_ctx.mutex_initialized) { vSemaphoreDelete((SemaphoreHandle_t)g_driver_ctx.state_mutex); }
    memset(&g_driver_ctx, 0, sizeof(sx1262_driver_context_t));
    return SX1262_OK;
}

bool sx1262_driver_is_initialized() { return g_driver_ctx.hardware_initialized; }

// ============================================================================
// HARDWARE SELF-TEST IMPLEMENTATION
// ============================================================================
// Performs low-level hardware validation without full driver initialization.
// Useful for diagnosing wiring issues before attempting radio operations.
// ============================================================================

sx1262_result_t sx1262_self_test(sx1262_self_test_result_t* result) {
    if (result == NULL) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Initialize result structure
    memset(result, 0, sizeof(sx1262_self_test_result_t));
    result->passed = false;
    strncpy(result->failure_reason, "Test not started", sizeof(result->failure_reason) - 1);
    
    // Test 1: BUSY Pin
    if (hal_gpio_init_input(PIN_SX1262_BUSY, 0) != HAL_OK) {
        strncpy(result->failure_reason, "BUSY pin GPIO init failed", sizeof(result->failure_reason) - 1);
        return SX1262_ERROR_HARDWARE;
    }
    result->busy_pin_readable = true;
    
    // Test 2: Reset
    if (hal_gpio_init_output(PIN_SX1262_NRESET) != HAL_OK) {
        strncpy(result->failure_reason, "NRST pin GPIO init failed", sizeof(result->failure_reason) - 1);
        return SX1262_ERROR_HARDWARE;
    }
    hal_reset(); 
    // Basic busy check
    if (hal_gpio_read(PIN_SX1262_BUSY) == 0) {
        result->reset_working = true;
        result->busy_pin_responsive = true;
    } else {
        strncpy(result->failure_reason, "BUSY stuck HIGH after reset", sizeof(result->failure_reason) - 1);
        return SX1262_ERROR_HARDWARE;
    }
    
    // Test 3: SPI
    if (hal_spi_init(SPI_FREQUENCY_HZ, PIN_SX1262_CS) != HAL_OK) {
        strncpy(result->failure_reason, "SPI initialization failed", sizeof(result->failure_reason) - 1);
        return SX1262_ERROR_HARDWARE;
    }
    
    uint8_t tx_buf[2] = {SX1262_OP_GET_STATUS, 0x00};
    uint8_t rx_buf[2] = {0xFF, 0xFF};
    
    hal_spi_cs_assert(1);
    if (hal_spi_transfer(tx_buf, rx_buf, 2) != HAL_OK) {
        hal_spi_deinit();
        return SX1262_ERROR_HARDWARE;
    }
    hal_spi_cs_assert(0);
    
    result->spi_communication = true;
    result->status_byte = rx_buf[1];
    
    // Test 4: Chip ID
    if (rx_buf[1] != 0x00 && rx_buf[1] != 0xFF) {
        result->chip_id_valid = true;
        result->chip_responding = true;
    } else {
        hal_spi_deinit();
        return SX1262_ERROR_HARDWARE;
    }
    
    hal_spi_deinit();
    result->passed = true;
    strncpy(result->failure_reason, "All tests passed", sizeof(result->failure_reason) - 1);
    return SX1262_OK;
}

void sx1262_print_self_test_result(const sx1262_self_test_result_t* result) {
    if (!result) return;
    SX1262_LOG_INFO("Self Test: %s (Status: 0x%02X)\n", result->passed ? "PASS" : "FAIL", result->status_byte);
}

// ============================================================================
// RESET OPS
// ============================================================================
sx1262_result_t sx1262_reset() {
    SX1262MutexGuard guard(g_driver_ctx.state_mutex);
    if (!guard.is_locked()) return SX1262_ERROR_MUTEX;
    
    hal_reset(); hal_delay_ms(10);
    g_driver_ctx.current_state = SX1262_STATE_STANDBY_RC;
    g_driver_ctx.previous_state = SX1262_STATE_UNINITIALIZED;
    g_driver_ctx.state_entry_time_ms = hal_get_time_ms();
    sx1262_watchdog_feed();
    log_state_transition(g_driver_ctx.previous_state, SX1262_STATE_STANDBY_RC, true, SX1262_OK);
    return SX1262_OK;
}

sx1262_result_t sx1262_soft_reset() {
    sx1262_result_t res = sx1262_set_state(SX1262_STATE_SLEEP);
    if (res == SX1262_OK) {
        hal_delay_ms(10);
        res = sx1262_set_state(SX1262_STATE_STANDBY_RC);
    }
    return res;
}

sx1262_result_t sx1262_emergency_reset() {
    sx1262_lora_config_t saved_config;
    bool had_config = g_driver_ctx.config_valid;
    
    // 1. Save config if valid
    if (had_config) {
        memcpy(&saved_config, &g_driver_ctx.config, sizeof(sx1262_lora_config_t));
    }
    
    // 2. Save stats/mutex before wiping
    uint32_t saved_resets = g_driver_ctx.watchdog_resets; // <--- SAVE STATS
    void* saved_mutex = g_driver_ctx.state_mutex;
    bool saved_mutex_init = g_driver_ctx.mutex_initialized;

    // 3. Hardware Teardown
    if (g_driver_ctx.spi_initialized) { 
        spi_deinit(); 
        g_driver_ctx.spi_initialized = false; 
    }
    hal_reset(); 
    hal_delay_ms(10);
    
    // 4. Context Wipe & Restore
    memset(&g_driver_ctx, 0, sizeof(sx1262_driver_context_t));
    g_driver_ctx.state_mutex = saved_mutex;
    g_driver_ctx.mutex_initialized = saved_mutex_init;
    g_driver_ctx.watchdog_resets = saved_resets + 1; // <--- RESTORE & INCREMENT
    
    // 5. Re-Initialize
    sx1262_result_t res = sx1262_driver_init();
    
    if (res == SX1262_OK) {
        // 6. Restore Config (ONLY IF IT WAS VALID)
        if (had_config) {
            sx1262_config_lora(&saved_config); 
        }
    } else {
        g_driver_ctx.current_state = SX1262_STATE_ERROR;
    }
    
    return res;
}

// ============================================================================
// STATS & ERRORS
// ============================================================================
void sx1262_get_last_error(sx1262_result_t* error, const char** function, uint32_t* timestamp_ms) {
    if (error) *error = g_driver_ctx.last_error;
    if (function) *function = g_driver_ctx.last_error_function;
    if (timestamp_ms) *timestamp_ms = g_driver_ctx.last_error_timestamp_ms;
}
void sx1262_clear_error() {
    g_driver_ctx.last_error = SX1262_OK;
    g_driver_ctx.last_error_function = NULL;
    g_driver_ctx.last_error_timestamp_ms = 0;
}
void sx1262_get_statistics(sx1262_statistics_t* stats) {
    if (!stats) return;
    SX1262MutexGuard guard(g_driver_ctx.state_mutex, 100);
    stats->current_state = g_driver_ctx.current_state;
    stats->time_in_state_ms = sx1262_get_time_in_state();
    stats->total_transitions = g_driver_ctx.total_state_transitions;
    stats->failed_transitions = g_driver_ctx.failed_state_transitions;
    stats->watchdog_resets = g_driver_ctx.watchdog_resets;
    stats->uptime_ms = hal_get_time_ms();
    stats->last_error = g_driver_ctx.last_error;
    stats->time_since_last_error_ms = (hal_get_time_ms() >= g_driver_ctx.last_error_timestamp_ms) ? 
        (hal_get_time_ms() - g_driver_ctx.last_error_timestamp_ms) : 0;
}
void sx1262_reset_statistics() {
    SX1262MutexGuard guard(g_driver_ctx.state_mutex, 100);
    g_driver_ctx.total_state_transitions = 0;
    g_driver_ctx.failed_state_transitions = 0;
}

// Print Helpers
void sx1262_print_status() {
    sx1262_statistics_t stats; 
    sx1262_get_statistics(&stats);
    SX1262_LOG_INFO("State: %s, Time: %lu ms, Transitions: %lu (Fail: %lu)\n", 
        sx1262_state_to_string(stats.current_state), stats.time_in_state_ms, 
        stats.total_transitions, stats.failed_transitions);
}

void sx1262_print_transition_history() {
    sx1262_state_transition_t history[16];
    uint8_t count = sx1262_get_transition_history(history, 16);
    SX1262_LOG_INFO("State Transition History (%d entries):\n", count);
    for (uint8_t i = 0; i < count; i++) {
        SX1262_LOG_INFO("  %d: %s -> %s\n", i, 
            sx1262_state_to_string(history[i].from_state), 
            sx1262_state_to_string(history[i].to_state));
    }
}

// ============================================================================
// PHASE 2.1: TX OPERATIONS (Refactored Phase 1.3: Auto-Recovery)
// ============================================================================
sx1262_result_t sx1262_transmit(const uint8_t* payload, uint8_t length, uint32_t timeout_ms, sx1262_tx_result_t* result) {
    if (result) memset(result, 0, sizeof(sx1262_tx_result_t));

    // 1. Lock Mutex (Guard)
    SX1262MutexGuard guard(g_driver_ctx.state_mutex);
    if (!guard.is_locked()) return SX1262_ERROR_MUTEX;

    // 2. Auto-Recovery (Phase 1.3)
    if (g_driver_ctx.current_state == SX1262_STATE_TX || g_driver_ctx.current_state == SX1262_STATE_RX) {
        if (sx1262_get_time_in_state() > 500) { 
            SX1262_LOG_DRIVER("[RECOVERY] Stuck in %s for >500ms. Triggering Reset.\n", sx1262_state_to_string(g_driver_ctx.current_state));
            sx1262_emergency_reset();
        }
    }

    // 3. Checks
    if (!sx1262_driver_is_initialized()) return SX1262_ERROR_NOT_INITIALIZED;
    if (!payload || length == 0) return SX1262_ERROR_INVALID_PARAM;

    // 4. Validate State
    if (g_driver_ctx.current_state != SX1262_STATE_STANDBY_RC && g_driver_ctx.current_state != SX1262_STATE_STANDBY_XOSC) {
        return SX1262_ERROR_INVALID_STATE;
    }
    if (!g_driver_ctx.config_valid) return SX1262_ERROR_INVALID_PARAM;

    // 5. Setup
    spi_clear_device_errors();
    
    // External config file usage -> math helpers might be in there too?
    // Assuming ToA math helper is in this file or available.
    // If linker error occurs here on math, we'll need to duplicate math helper.
    // Assuming math helper IS in here based on original file.
    // Wait, original file had `sx1262_get_time_on_air_ms` declared/defined?
    // User error log showed it was defined in both. So it must be in another file.
    // I will call it, assuming it links from config file.
    uint32_t toa_ms = 200; // Fallback if math fails? No, let's assume valid link.
    // Actually, to be safe against linker errors, I will use a hardcoded safe timeout if I can't call the function.
    // But better to trust the linker.
    // Checking previous logs... "multiple definition of sx1262_get_time_on_air_ms"
    // So it IS in config.cpp.
    // I will NOT define it here.
    
    // PROBLEM: How to call it if I don't include the header for it? 
    // It's in sx1262_driver.h which IS included. So I can call it.
    toa_ms = sx1262_get_time_on_air_ms(&g_driver_ctx.config, length);
    
    uint32_t actual_timeout = (timeout_ms == 0 || timeout_ms < toa_ms * 1.5) ? (toa_ms + toa_ms/2 + 500) : timeout_ms;
    
    if (result) {
        result->tx_start_ms = hal_get_time_ms();
        result->calculated_toa_ms = toa_ms;
        result->timeout_used_ms = actual_timeout;
    }

    // 6. Buffer Ops
    spi_set_buffer_base_address(0, 128);
    spi_write_buffer(0, payload, length);
    spi_clear_irq_status(0xFFFF);
    
    // 7. Config Ops
    // WARNING: SetPacketType resets all packet parameters to defaults! [Datasheet 13.4.2]
    // We MUST re-apply them immediately after to ensure Explicit Header, CRC, etc. are active.
    uint8_t lora_type = PACKET_TYPE_LORA;
    spi_cmd(SX1262_OP_SET_PACKET_TYPE, &lora_type, 1, NULL, 0, 0, NULL);
    hal_delay_ms(5); // Wait for mode change
    
    // CRITICAL FIX: Re-apply Packet Params immediately
    uint8_t pkt_params[6] = {
        (uint8_t)(g_driver_ctx.config.preamble_length >> 8),
        (uint8_t)(g_driver_ctx.config.preamble_length & 0xFF),
        (uint8_t)g_driver_ctx.config.header_type,
        length,  // <--- CRITICAL: Set actual payload length for this transmission
        (uint8_t)g_driver_ctx.config.crc_type,
        (uint8_t)g_driver_ctx.config.invert_iq
    };
    spi_cmd(SX1262_OP_SET_PACKET_PARAMS, pkt_params, 6, NULL, 0, 0, NULL);

    // Configure Interrupts
    spi_set_dio_irq_params(
        IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, // IRQ Mask
        IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, // DIO1 Mask
        0, 0                             // DIO2, DIO3
    );

    // 8. Atomic Transition to STANDBY_XOSC (RESTORED FROM WORKING POLLING DRIVER)
    if (g_driver_ctx.current_state != SX1262_STATE_STANDBY_XOSC) {
        if (_internal_set_state(SX1262_STATE_STANDBY_XOSC) != SX1262_OK) return SX1262_ERROR_HARDWARE;
        hal_delay_ms(5);
    }

    // 9. PREPARE FOR WAIT (Phase 3)
    // Clear notification state
    ulTaskNotifyTake(pdTRUE, 0);
    // Register task handle
    g_driver_ctx.blocking_task_handle = xTaskGetCurrentTaskHandle();

    // 10. Atomic Transition to TX State
    sx1262_result_t state_res = _internal_set_state(SX1262_STATE_TX);
    if (state_res != SX1262_OK) {
        g_driver_ctx.blocking_task_handle = NULL;
        return state_res;
    }

    // 11. Send Hardware TX Command
    spi_result_t spi_res = spi_set_tx(actual_timeout * 1000);
    if (spi_res != SPI_OK) {
        _internal_set_state(SX1262_STATE_STANDBY_RC); // Recover
        g_driver_ctx.blocking_task_handle = NULL;
        return SX1262_ERROR_SPI;
    }

    // 12. WAIT FOR INTERRUPT (Phase 3 Replacement for Polling)
    // Block until ISR fires or timeout
    uint32_t wait_ticks = pdMS_TO_TICKS(actual_timeout + 50); // Small buffer
    uint32_t notif_value = ulTaskNotifyTake(pdTRUE, wait_ticks);
    
    // Clear handle
    g_driver_ctx.blocking_task_handle = NULL;

    // 13. CHECK STATUS
    uint16_t irq_status = 0;
    spi_get_irq_status(&irq_status);
    
    // Check if we really finished
    bool tx_done = (irq_status & IRQ_TX_DONE);
    bool hw_timeout = (irq_status & IRQ_RX_TX_TIMEOUT);
    
    if (result) {
        result->tx_done_ms = hal_get_time_ms();
        result->tx_duration_ms = result->tx_done_ms - result->tx_start_ms;
        result->irq_flags = irq_status;
        result->timed_out = !tx_done;
        result->success = tx_done;
    }

    spi_clear_irq_status(0xFFFF);

    // 14. Atomic Return to Standby
    _internal_set_state(SX1262_STATE_STANDBY_RC);

    return tx_done ? SX1262_OK : SX1262_ERROR_TIMEOUT;
}

// ============================================================================
// PHASE 2.2: RX OPERATIONS (Refactored Phase 1.3: Auto-Recovery)
// ============================================================================
sx1262_result_t sx1262_receive(
    uint8_t* payload, 
    uint8_t max_length, 
    uint32_t timeout_ms, 
    sx1262_rx_result_t* result
) {
    if (result) memset(result, 0, sizeof(sx1262_rx_result_t));

    SX1262MutexGuard guard(g_driver_ctx.state_mutex);
    if (!guard.is_locked()) {
        if (result) result->error_code = SX1262_ERROR_MUTEX;
        return SX1262_ERROR_MUTEX;
    }

    if (g_driver_ctx.current_state == SX1262_STATE_TX || g_driver_ctx.current_state == SX1262_STATE_RX) {
        if (sx1262_get_time_in_state() > 500) { 
            SX1262_LOG_DRIVER("[RECOVERY] Stuck in %s for >500ms. Triggering Reset.\n", 
                              sx1262_state_to_string(g_driver_ctx.current_state));
            sx1262_emergency_reset();
        }
    }

    if (g_driver_ctx.current_state == SX1262_STATE_ERROR) {
        SX1262_LOG_DRIVER("[RX] Cannot proceed - driver in ERROR state\n");
        if (result) result->error_code = SX1262_ERROR_INVALID_STATE;
        return SX1262_ERROR_INVALID_STATE;
    }

    if (!sx1262_driver_is_initialized()) return SX1262_ERROR_NOT_INITIALIZED;
    if (!payload || max_length == 0) return SX1262_ERROR_INVALID_PARAM;

    if (g_driver_ctx.current_state != SX1262_STATE_STANDBY_RC && 
        g_driver_ctx.current_state != SX1262_STATE_STANDBY_XOSC) {
        if (result) result->error_code = SX1262_ERROR_INVALID_STATE;
        return SX1262_ERROR_INVALID_STATE;
    }

    spi_clear_device_errors();
    spi_set_buffer_base_address(0, 128);
    spi_clear_irq_status(0xFFFF);
    
    uint8_t lora_type = PACKET_TYPE_LORA;
    spi_cmd(SX1262_OP_SET_PACKET_TYPE, &lora_type, 1, NULL, 0, 0, NULL);
    hal_delay_ms(5);

    uint8_t pkt_params[6] = {
        (uint8_t)(g_driver_ctx.config.preamble_length >> 8),    // Preamble MSB
        (uint8_t)(g_driver_ctx.config.preamble_length & 0xFF),  // Preamble LSB
        (uint8_t)g_driver_ctx.config.header_type,               // 0x00 = Explicit header
        g_driver_ctx.config.payload_length,                      // Max receivable length
        (uint8_t)g_driver_ctx.config.crc_type,                  // CRC ON/OFF
        (uint8_t)g_driver_ctx.config.invert_iq                  // IQ polarity
    };
    
    spi_cmd(SX1262_OP_SET_PACKET_PARAMS, pkt_params, 6, NULL, 0, 0, NULL);
    hal_delay_ms(1);
    
    uint8_t irq_cmd[9] = {0x08, 0x02, 0x42, 0x02, 0x42, 0x00, 0x00, 0x00, 0x00};
    hal_spi_cs_assert(1); hal_spi_transfer(irq_cmd, NULL, 9); hal_spi_cs_assert(0);
    hal_delay_ms(1);

    uint32_t timeout_with_margin = timeout_ms + (timeout_ms / 10) + 100;
    uint32_t timeout_steps = timeout_with_margin * 64;  
    
    if (timeout_steps < 64) timeout_steps = 64;  
    if (timeout_steps > 0xFFFFFE) timeout_steps = 0xFFFFFE; 
    
    uint8_t rx_cmd[4] = {
        SX1262_OP_SET_RX,                      // 0x82
        (uint8_t)((timeout_steps >> 16) & 0xFF),  // Timeout MSB
        (uint8_t)((timeout_steps >> 8) & 0xFF),   // Timeout MID
        (uint8_t)(timeout_steps & 0xFF)           // Timeout LSB
    };
    
    uint32_t busy_start = hal_get_time_ms();
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if (hal_get_time_ms() - busy_start > 100) {
            SX1262_LOG_ERROR("[RX] BUSY timeout before SetRx\n");
            if (result) result->error_code = SX1262_ERROR_BUSY_TIMEOUT;
            return SX1262_ERROR_BUSY_TIMEOUT;
        }
    }
    
    // PREPARE NOTIFY (Phase 3)
    ulTaskNotifyTake(pdTRUE, 0);
    g_driver_ctx.blocking_task_handle = xTaskGetCurrentTaskHandle();

    // =========================================================================
    // FIX (v2): Capture RX start time for timing diagnostics
    // =========================================================================
    uint32_t rx_start_ms = hal_get_time_ms();
    if (result) {
        result->rx_start_ms = rx_start_ms;
        result->timeout_used_ms = timeout_ms;
    }

    // Send SetRx command
    hal_spi_cs_assert(1);
    hal_spi_transfer(rx_cmd, NULL, 4);
    hal_spi_cs_assert(0);
    
    g_driver_ctx.previous_state = g_driver_ctx.current_state;
    g_driver_ctx.current_state = SX1262_STATE_RX;
    g_driver_ctx.state_entry_time_ms = hal_get_time_ms();
    
    // Errata 15.2 WORKAROUND (Preserved)
    hal_delay_ms(1);
    hal_spi_cs_assert(1); hal_spi_transfer(irq_cmd, NULL, 9); hal_spi_cs_assert(0);
    hal_delay_ms(1);
    
    spi_clear_irq_status(0xFFFF);

    // WAIT (Phase 3 Replacement)
    uint32_t wait_ticks = pdMS_TO_TICKS(timeout_ms + 100);
    ulTaskNotifyTake(pdTRUE, wait_ticks);
    g_driver_ctx.blocking_task_handle = NULL;

    // =========================================================================
    // FIX (v2): Capture RX done time for timing diagnostics
    // =========================================================================
    uint32_t rx_done_ms = hal_get_time_ms();
    if (result) {
        result->rx_done_ms = rx_done_ms;
        result->rx_duration_ms = rx_done_ms - rx_start_ms;
    }

    // CHECK STATUS
    uint16_t irq = 0;
    spi_get_irq_status(&irq);
    bool rx_done = (irq & IRQ_RX_DONE);
    bool crc_err = (irq & IRQ_CRC_ERROR);

    if (rx_done) { 
        hal_delay_ms(3); // Preserved buffer delay
    }

    if (rx_done && !crc_err) {
        uint8_t len = 0, offset = 0;
        spi_get_rx_buffer_status(&len, &offset);
        
        SX1262_LOG_INFO("[RX] Buffer status: PayloadLength=%d, Offset=%d\n", len, offset);
        
        if (len == 0) {
            SX1262_LOG_WARN("[RX] WARNING: Received payload length is 0\n");
        } else if (len == 255 && g_driver_ctx.config.header_type == SX1262_LORA_HEADER_EXPLICIT) {
            SX1262_LOG_WARN("[RX] WARNING: Length=255 in explicit header mode - header may not have been parsed\n");
        }
        
        if (len > max_length) {
            SX1262_LOG_WARN("[RX] Truncating payload from %d to %d bytes\n", len, max_length);
            len = max_length;
        }
        
        spi_read_buffer(offset, payload, len);
        
        if (result) {
            result->payload_length = len;
            spi_get_packet_status(&result->rssi_pkt, &result->snr_pkt, &result->signal_rssi_pkt);
            SX1262_LOG_INFO("[RX] Packet received: %d bytes, RSSI=%d, SNR=%d\n", 
                           len, result->rssi_pkt, result->snr_pkt);
        }
    }

    // CLEANUP
    spi_clear_irq_status(0xFFFF);
    _internal_set_state(SX1262_STATE_STANDBY_RC);

    if (result) {
        result->success = rx_done && !crc_err;
        result->crc_error = crc_err;
        result->irq_flags = irq;
    }

    return (rx_done && !crc_err) ? SX1262_OK : (crc_err ? SX1262_ERROR_HARDWARE : SX1262_ERROR_TIMEOUT);
}

// ============================================================================
// PHASE 3.1: TX-RX TRANSITION FUNCTIONS
// ============================================================================

/**
 * @brief Internal helper to ensure radio is in STANDBY_RC state
 * 
 * This function handles the state machine transition to STANDBY_RC from
 * any valid state. It's used by the turnaround functions to ensure clean
 * state before TX/RX operations.
 * 
 * ASSUMES MUTEX IS ALREADY HELD by caller.
 * 
 * @return SX1262_OK if already in STANDBY or transition successful
 * @return SX1262_ERROR_HARDWARE if transition failed
 */
static sx1262_result_t _ensure_standby_rc(void) {
    sx1262_state_t current = g_driver_ctx.current_state;
    
    // Already in a STANDBY state - we're good
    if (current == SX1262_STATE_STANDBY_RC || current == SX1262_STATE_STANDBY_XOSC) {
        // If in XOSC, optionally transition to RC for consistency
        // But both are valid starting points for TX/RX, so just return OK
        SX1262_LOG_DEBUG("[TURNAROUND] Already in %s\n", sx1262_state_to_string(current));
        return SX1262_OK;
    }
    
    // From TX, RX, FS, or ERROR - transition to STANDBY_RC
    SX1262_LOG_INFO("[TURNAROUND] Transitioning from %s to STANDBY_RC\n", sx1262_state_to_string(current));
    
    sx1262_result_t res = _internal_set_state(SX1262_STATE_STANDBY_RC);
    if (res != SX1262_OK) {
        SX1262_LOG_ERROR("[TURNAROUND] Failed to reach STANDBY_RC: %s\n", sx1262_error_to_string(res));
        return SX1262_ERROR_HARDWARE;
    }
    
    // Small settling delay after state change
    hal_delay_ms(1);
    
    return SX1262_OK;
}

// ============================================================================
// sx1262_prepare_for_rx
// ============================================================================
sx1262_result_t sx1262_prepare_for_rx(void) {
    // 1. Check initialization
    if (!sx1262_driver_is_initialized()) {
        SX1262_LOG_ERROR("[PREPARE_RX] Driver not initialized\n");
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // 2. Acquire mutex
    SX1262MutexGuard guard(g_driver_ctx.state_mutex);
    if (!guard.is_locked()) {
        SX1262_LOG_ERROR("[PREPARE_RX] Failed to acquire mutex\n");
        return SX1262_ERROR_MUTEX;
    }
    
    // 3. Ensure we're in STANDBY
    sx1262_result_t res = _ensure_standby_rc();
    if (res != SX1262_OK) {
        return res;
    }
    
    // 4. Clear any pending IRQs from previous operations
    spi_clear_irq_status(0xFFFF);
    
    SX1262_LOG_INFO("[PREPARE_RX] Ready for RX operation\n");
    return SX1262_OK;
}

// ============================================================================
// sx1262_prepare_for_tx
// ============================================================================
sx1262_result_t sx1262_prepare_for_tx(void) {
    // 1. Check initialization
    if (!sx1262_driver_is_initialized()) {
        SX1262_LOG_ERROR("[PREPARE_TX] Driver not initialized\n");
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // 2. Acquire mutex
    SX1262MutexGuard guard(g_driver_ctx.state_mutex);
    if (!guard.is_locked()) {
        SX1262_LOG_ERROR("[PREPARE_TX] Failed to acquire mutex\n");
        return SX1262_ERROR_MUTEX;
    }
    
    // 3. Ensure we're in STANDBY
    sx1262_result_t res = _ensure_standby_rc();
    if (res != SX1262_OK) {
        return res;
    }
    
    // 4. Clear any pending IRQs from previous operations
    spi_clear_irq_status(0xFFFF);
    
    SX1262_LOG_INFO("[PREPARE_TX] Ready for TX operation\n");
    return SX1262_OK;
}

// ============================================================================
// sx1262_turnaround_tx_to_rx
// ============================================================================
sx1262_result_t sx1262_turnaround_tx_to_rx(
    uint8_t* rx_payload,
    uint8_t max_length,
    uint32_t timeout_ms,
    sx1262_rx_result_t* result
) {
    // Parameter validation
    if (!rx_payload || max_length == 0) {
        SX1262_LOG_ERROR("[TX->RX] Invalid parameters\n");
        if (result) {
            result->success = false;
            result->error_code = SX1262_ERROR_INVALID_PARAM;
        }
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_INFO("[TX->RX] Turnaround starting (timeout=%lu ms)\n", timeout_ms);
    
    // Step 1: Prepare for RX (ensures STANDBY, clears IRQs)
    sx1262_result_t prep_res = sx1262_prepare_for_rx();
    if (prep_res != SX1262_OK) {
        SX1262_LOG_ERROR("[TX->RX] Preparation failed: %s\n", sx1262_error_to_string(prep_res));
        if (result) {
            result->success = false;
            result->error_code = prep_res;
        }
        return prep_res;
    }
    
    // Step 2: Call the proven RX function
    // sx1262_receive() handles all the complexity: mutex, errata 15.2, IRQ config, etc.
    sx1262_result_t rx_res = sx1262_receive(rx_payload, max_length, timeout_ms, result);
    
    if (rx_res == SX1262_OK) {
        SX1262_LOG_INFO("[TX->RX] Turnaround complete - packet received\n");
    } else {
        SX1262_LOG_INFO("[TX->RX] Turnaround complete - %s\n", sx1262_error_to_string(rx_res));
    }
    
    return rx_res;
}

// ============================================================================
// sx1262_turnaround_rx_to_tx
// ============================================================================
sx1262_result_t sx1262_turnaround_rx_to_tx(
    const uint8_t* tx_payload,
    uint8_t length,
    uint32_t timeout_ms,
    sx1262_tx_result_t* result
) {
    // Parameter validation
    if (!tx_payload || length == 0) {
        SX1262_LOG_ERROR("[RX->TX] Invalid parameters\n");
        if (result) {
            result->success = false;
            result->error_code = SX1262_ERROR_INVALID_PARAM;
        }
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_INFO("[RX->TX] Turnaround starting (len=%d, timeout=%lu ms)\n", length, timeout_ms);
    
    // Step 1: Prepare for TX (ensures STANDBY, clears IRQs)
    sx1262_result_t prep_res = sx1262_prepare_for_tx();
    if (prep_res != SX1262_OK) {
        SX1262_LOG_ERROR("[RX->TX] Preparation failed: %s\n", sx1262_error_to_string(prep_res));
        if (result) {
            result->success = false;
            result->error_code = prep_res;
        }
        return prep_res;
    }
    
    // Step 2: Call the proven TX function
    // sx1262_transmit() handles all the complexity: mutex, ToA calc, IRQ config, etc.
    sx1262_result_t tx_res = sx1262_transmit(tx_payload, length, timeout_ms, result);
    
    if (tx_res == SX1262_OK) {
        SX1262_LOG_INFO("[RX->TX] Turnaround complete - packet transmitted\n");
    } else {
        SX1262_LOG_INFO("[RX->TX] Turnaround complete - %s\n", sx1262_error_to_string(tx_res));
    }
    
    return tx_res;
}

// ============================================================================
// PHASE 3.1: POWER MANAGEMENT IMPLEMENTATION
// ============================================================================

sx1262_result_t sx1262_sleep(sx1262_sleep_mode_t mode) {
    // Validate initialization
    if (!g_driver_ctx.hardware_initialized) {
        SX1262_LOG_ERROR("[SLEEP] Driver not initialized\n");
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // Check if already sleeping
    if (g_driver_ctx.power.is_sleeping) {
        SX1262_LOG_WARN("[SLEEP] Already in sleep mode\n");
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Acquire mutex
    SX1262MutexGuard guard(g_driver_ctx.state_mutex, 1000);
    if (!guard.is_locked()) {
        SX1262_LOG_ERROR("[SLEEP] Failed to acquire mutex\n");
        return SX1262_ERROR_MUTEX;
    }
    
    // Ensure we're in STANDBY before sleeping
    if (g_driver_ctx.current_state != SX1262_STATE_STANDBY_RC &&
        g_driver_ctx.current_state != SX1262_STATE_STANDBY_XOSC) {
        SX1262_LOG_INFO("[SLEEP] Transitioning to STANDBY first\n");
        spi_set_standby(SX1262_STDBY_RC);
        hal_delay_ms(1);
    }
    
    // Send SetSleep command
    // Opcode: 0x84
    // Parameter: sleepConfig byte
    //   Bit 0: RTC timeout enable (0 = disabled)
    //   Bit 2: Warm start (1) or Cold start (0)
    uint8_t sleep_config = (uint8_t)mode;
    
    // Wait for BUSY low before command
    uint32_t busy_start = hal_get_time_ms();
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_ms() - busy_start) > 100) {
            SX1262_LOG_ERROR("[SLEEP] BUSY timeout before SetSleep\n");
            return SX1262_ERROR_BUSY_TIMEOUT;
        }
        hal_delay_us(10);
    }
    
    // Send SetSleep command via direct HAL (chip becomes unresponsive immediately)
    uint8_t cmd[2] = {SX1262_OP_SET_SLEEP, sleep_config};
    hal_spi_cs_assert(1);
    hal_spi_transfer(cmd, NULL, 2);
    hal_spi_cs_assert(0);
    
    // Wait for chip to enter sleep (BUSY goes high and stays high in sleep)
    hal_delay_us(600);
    
    // Update power state tracking
    g_driver_ctx.power.is_sleeping = true;
    g_driver_ctx.power.sleep_mode = mode;
    g_driver_ctx.power.sleep_enter_time_ms = hal_get_time_ms();
    
    // Update driver state
    g_driver_ctx.current_state = SX1262_STATE_SLEEP;
    
    SX1262_LOG_INFO("[SLEEP] Entered %s sleep mode\n", 
                    (mode == SX1262_SLEEP_WARM) ? "WARM" : "COLD");
    
    return SX1262_OK;
}

sx1262_result_t sx1262_wake(void) {
    // 1. Validate initialization
    if (!g_driver_ctx.hardware_initialized) {
        SX1262_LOG_ERROR("[WAKE] Driver not initialized\n");
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    if (!g_driver_ctx.power.is_sleeping) {
        SX1262_LOG_WARN("[WAKE] Not in sleep mode\n");
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Capture sleep mode before we clear sleeping flag
    sx1262_sleep_mode_t wake_from_mode = g_driver_ctx.power.sleep_mode;
    
    // ============================================================
    // CRITICAL MUTEX SCOPE
    // ============================================================
    {
        SX1262MutexGuard guard(g_driver_ctx.state_mutex, 1000);
        if (!guard.is_locked()) {
            SX1262_LOG_ERROR("[WAKE] Failed to acquire mutex\n");
            return SX1262_ERROR_MUTEX;
        }
        
        // 2. Hardware Wake Sequence (NSS Toggle)
        hal_spi_cs_assert(1);  // NSS LOW
        hal_delay_us(100);     // Brief pulse
        hal_spi_cs_assert(0);  // NSS HIGH
        
        // 3. Wait for BUSY to go low
        uint32_t wake_timeout_ms = (wake_from_mode == SX1262_SLEEP_WARM) ? 5 : 50;
        uint32_t wake_start = hal_get_time_ms();
        
        while (hal_gpio_read(PIN_SX1262_BUSY)) {
            if ((hal_get_time_ms() - wake_start) > wake_timeout_ms) {
                SX1262_LOG_ERROR("[WAKE] BUSY timeout after %lu ms\n", wake_timeout_ms);
                g_driver_ctx.power.is_sleeping = false;
                return SX1262_ERROR_TIMEOUT;
            }
            hal_delay_us(50);
        }
        
        // 4. Update Stats
        uint32_t sleep_duration = hal_get_time_ms() - g_driver_ctx.power.sleep_enter_time_ms;
        g_driver_ctx.power.total_sleep_time_ms += sleep_duration;
        g_driver_ctx.power.wake_count++;
        g_driver_ctx.power.is_sleeping = false;
        
        // 5. Force Standby RC
        spi_set_standby(SX1262_STDBY_RC);
        hal_delay_us(500);
        
        // ============================================================
        // COLD SLEEP ONLY: RESTORE HARDWARE CONFIGURATION
        // WARM sleep retains all configuration in chip memory
        // ============================================================
        if (wake_from_mode == SX1262_SLEEP_COLD) {
            SX1262_LOG_INFO("[WAKE] COLD start - restoring hardware config...\n");
            
            // A. TCXO Init (1.8V, 5ms delay)
            uint8_t tcxo_params[4] = {0x02, 0x00, 0x01, 0x40}; 
            spi_cmd(0x97, tcxo_params, 4, NULL, 0, 0, NULL);
            hal_delay_ms(5); // Wait for TCXO
            
            // B. Image Calibration (863-870 MHz)
            uint8_t cal_img[2] = {0xE1, 0xE9};
            spi_cmd(0x98, cal_img, 2, NULL, 0, 0, NULL);
            hal_delay_ms(5);

            // C. PA Configuration (+22dBm)
            uint8_t pa_config[4] = {0x04, 0x07, 0x00, 0x01};
            spi_cmd(0x95, pa_config, 4, NULL, 0, 0, NULL);

            // D. Enable DIO2 RF Switch
            uint8_t rf_switch_enable = 0x01;
            spi_cmd(0x9D, &rf_switch_enable, 1, NULL, 0, 0, NULL);
            hal_delay_ms(1);
            
            // Mark config as invalid - user must re-apply LoRa config
            g_driver_ctx.config_valid = false;
        } else {
            SX1262_LOG_INFO("[WAKE] WARM start - config retained in chip\n");
        }
        
        g_driver_ctx.current_state = SX1262_STATE_STANDBY_RC;
        
    } // <--- Mutex released here
    
    // ============================================================
    // COLD SLEEP ONLY: RESTORE LORA CONFIGURATION
    // ============================================================
    if (wake_from_mode == SX1262_SLEEP_COLD) {
        if (g_driver_ctx.config_valid || g_driver_ctx.config.configured) {
            SX1262_LOG_INFO("[WAKE] Re-applying LoRa configuration...\n");
            sx1262_result_t config_res = sx1262_config_lora(&g_driver_ctx.config);
            if (config_res != SX1262_OK) {
                SX1262_LOG_ERROR("[WAKE] Config re-apply failed: %d\n", config_res);
                return config_res;
            }
            g_driver_ctx.config_valid = true;
        } else {
            SX1262_LOG_WARN("[WAKE] COLD start but no saved config - manual config required\n");
        }
    }
    
    SX1262_LOG_INFO("[WAKE] Awake after %lu ms %s sleep\n", 
                   (hal_get_time_ms() - g_driver_ctx.power.sleep_enter_time_ms),
                   (wake_from_mode == SX1262_SLEEP_WARM) ? "WARM" : "COLD");
    
    return SX1262_OK;
}

bool sx1262_is_sleeping(void) {
    return g_driver_ctx.power.is_sleeping;
}

void sx1262_get_power_stats(uint32_t* total_sleep_ms, uint32_t* wake_count) {
    // If currently sleeping, include current sleep period in total
    uint32_t current_total = g_driver_ctx.power.total_sleep_time_ms;
    
    if (g_driver_ctx.power.is_sleeping) {
        current_total += (hal_get_time_ms() - g_driver_ctx.power.sleep_enter_time_ms);
    }
    
    if (total_sleep_ms) {
        *total_sleep_ms = current_total;
    }
    
    if (wake_count) {
        *wake_count = g_driver_ctx.power.wake_count;
    }
}