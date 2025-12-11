#ifndef SX1262_ERRATA_H
#define SX1262_ERRATA_H

#include <stdint.h>
#include <stdbool.h>
#include "sx1262_driver.h"

// ============================================================================
// SX1262 ERRATA WORKAROUNDS
// ============================================================================
// This file contains mandatory fixes for known SX1262 silicon bugs
// Based on Semtech official errata documentation and AN1200.22
//
// CRITICAL ERRATAS FIXED:
// - ERRATA 2.1: Sensitivity optimization for 500 kHz bandwidth
// - ERRATA 5:   Low transmit power wrap-around bug (â‰¤ -10 dBm)
// - ERRATA 7:   TCXO frequency drift warning at high power
//
// Date: November 21, 2024
// ============================================================================

/**
 * Apply ERRATA 2.1 fix: Sensitivity optimization for 500 kHz bandwidth
 * 
 * ISSUE: Sub-optimal RX sensitivity when using LoRa BW = 500 kHz if register
 *        settings are left at default. Sensitivity degraded by ~3 dB.
 * 
 * WORKAROUND: Set bit 2 of register 0x093C after configuring 500 kHz BW
 * 
 * This function MUST be called immediately after SetModulationParams
 * if bandwidth is 500 kHz.
 * 
 * @param bandwidth The bandwidth being configured
 * @return SX1262_OK on success, error code on failure
 * 
 * Reference: Semtech AN1200.22, Section "ERRATA 2.1"
 */
sx1262_result_t sx1262_errata_bw500_sensitivity_fix(uint8_t bandwidth);

/**
 * Validate TX power to prevent ERRATA 5 wrap-around bug
 * 
 * ISSUE: When setting TX power â‰¤ -10 dBm, the output power "wraps around"
 *        to near maximum (~+20 dBm) on SX1262 modules. This is a CRITICAL
 *        SAFETY issue that can violate regulatory limits!
 * 
 * WORKAROUND: 
 *   1. Reject any power setting < -9 dBm
 *   2. Use minimum ramp time of 200 Âµs for low power (â‰¤ 0 dBm)
 * 
 * This function validates power and adjusts ramp time if needed.
 * 
 * @param power_dbm Pointer to TX power in dBm (may be modified)
 * @param ramp_time Pointer to ramp time (may be modified for safety)
 * @return SX1262_OK if valid, SX1262_ERROR_INVALID_PARAM if rejected
 * 
 * Reference: Semtech AN1200.22, Section "Low Power Bug"
 */
sx1262_result_t sx1262_errata_validate_tx_power(int8_t* power_dbm, uint8_t* ramp_time);

/**
 * Check and warn about ERRATA 7: Frequency drift at high power with XTAL
 * 
 * ISSUE: Significant self-heating during +22 dBm TX causes frequency drift
 *        > Â±50 ppm on XTAL designs, violating LoRa low-data-rate requirements.
 * 
 * RECOMMENDATION: Use TCXO for any packet with time-on-air > 500 ms at high power
 * 
 * This function checks time-on-air and power level, prints warning if needed.
 * 
 * @param time_on_air_ms Time-on-air in milliseconds
 * @param power_dbm TX power in dBm
 * 
 * Reference: Semtech AN1200.22, Section "TCXO vs XTAL"
 */
void sx1262_errata_tcxo_drift_warning(uint32_t time_on_air_ms, int8_t power_dbm);

/**
 * Print all applied errata fixes (for debugging)
 * 
 * Displays which errata workarounds have been applied during configuration.
 * Useful for troubleshooting and verification.
 */
void sx1262_errata_print_applied_fixes(void);

// ============================================================================
// ERRATA STATISTICS (Optional)
// ============================================================================
typedef struct {
    uint32_t bw500_fixes_applied;      // Count of BW500 fixes applied
    uint32_t low_power_rejections;     // Count of dangerous power levels rejected
    uint32_t tcxo_warnings_shown;      // Count of TCXO warnings displayed
    uint32_t dio_irq_reconfigs;        // Count of Errata 15.2 DIO reconfigs applied
    uint32_t dio_irq_reconfig_failures; // Count of failed DIO reconfig attempts
} sx1262_errata_stats_t;

/**
 * Get errata statistics
 * 
 * @param stats Pointer to statistics structure to fill
 */
void sx1262_errata_get_stats(sx1262_errata_stats_t* stats);

/**
 * Reset errata statistics counters
 */
void sx1262_errata_reset_stats(void);

#endif // SX1262_ERRATA_H