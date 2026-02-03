/**
 * sx1262_mutex_guard.h
 * 
 * RAII wrapper for FreeRTOS Mutex
 * Ensures mutex is ALWAYS released when the guard goes out of scope.
 * Critical for preventing deadlocks in functions with multiple return paths.
 * 
 * Phase 1.1 Implementation - Part of Driver Robustness Improvements
 */

#ifndef SX1262_MUTEX_GUARD_H
#define SX1262_MUTEX_GUARD_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class SX1262MutexGuard {
private:
    SemaphoreHandle_t _mutex;
    bool _locked;

public:
    // Constructor: Attempts to take the mutex
    SX1262MutexGuard(void* mutex_handle, uint32_t timeout_ms = 1000) {
        _mutex = (SemaphoreHandle_t)mutex_handle;
        _locked = false;

        if (_mutex != NULL) {
            TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
            if (xSemaphoreTake(_mutex, ticks) == pdTRUE) {
                _locked = true;
            }
        }
    }

    // Destructor: Automatically releases mutex if we hold it
    ~SX1262MutexGuard() {
        if (_locked && _mutex != NULL) {
            xSemaphoreGive(_mutex);
            _locked = false;
        }
    }

    // Check if lock was successfully acquired
    bool is_locked() const {
        return _locked;
    }
    
    // Explicit unlock (optional, if we want to release before scope ends)
    void unlock() {
        if (_locked && _mutex != NULL) {
            xSemaphoreGive(_mutex);
            _locked = false;
        }
    }
};

#endif // SX1262_MUTEX_GUARD_H