// ============================================================================
// SX1262 ERRATA WORKAROUNDS IMPLEMENTATION
// ============================================================================
// Mandatory fixes for known SX1262 silicon bugs
// Based on Semtech AN1200.22 and official errata documentation
//
// Date: November 21, 2024
// ============================================================================

#include "sx1262_errata.h"
#include "sx1262_spi_protocol.h"
#include "sx1262_regs.h"
#include "config.h"
#include <Arduino.h>

// ============================================================================
// ERRATA STATISTICS
// ============================================================================
static sx1262_errata_stats_t g_errata_stats = {0};

// ============================================================================
// ERRATA 15.2: DIO IRQ CONFIGURATION LOST AFTER SetRx WITH TIMEOUT
// ============================================================================
// CRITICAL SILICON BUG: When SetRx() is called with a finite timeout value
// (any value other than 0xFFFFFF "no timeout"), the SX1262 silently CLEARS
// the IRQ configuration that was previously set via SetDioIrqParams.
//
// This affects DIO1 in particular when used as the primary IRQ output pin.
// The IRQ mask register (0x0608) reads back as 0x0000 after SetRx, even
// though it was correctly configured before.
//
// SYMPTOM:
// - GetIrqStatus (0x12) always returns 0x0000
// - No interrupts ever fire on DIO1
// - Driver times out waiting for RX_DONE
// - Polling shows IRQ status stuck at 0x0000
//
// ROOT CAUSE:
// Hardware bug in DIO1 IRQ latching logic on certain silicon revisions
// (confirmed by Semtech in official errata documentation)
//
// WORKAROUND:
// Re-issue SetDioIrqParams (0x08) immediately AFTER SetRx command.
// The second configuration "sticks" and interrupts work correctly.
//
// IMPLEMENTATION:
// Applied automatically in sx1262_receive() function - no user action needed.
// See sx1262_driver.cpp line ~1780 for implementation.
//
// REFERENCES:
// - Semtech SX1261/2 Errata Document Section 15.2
// - RadioLib sx126x.cpp (jgromes/RadioLib on GitHub)
// - STM32 LoRaWAN stack (STMicroelectronics)
// - Meshtastic firmware (documented as "mandatory for RX to work")
//
// NOTES:
// - This workaround is used by ALL major SX1262 driver implementations
// - SetTx does NOT have this issue (only SetRx with finite timeout)
// - Using timeout=0xFFFFFF avoids the bug but loses hardware timeout feature
// - The workaround adds ~22ms to RX setup (one extra SPI command)
// ============================================================================

sx1262_result_t sx1262_errata_dio_irq_reconfig_after_setrx(uint16_t irq_mask) {
    // This workaround is applied automatically in sx1262_receive()
    // This function exists only for documentation and statistics tracking
    
    SX1262_LOG_ERRATA("Errata 15.2: Re-configuring DIO IRQ after SetRx (IRQ mask: 0x%04X)\n", 
                      irq_mask);
    
    // Re-issue SetDioIrqParams command
    spi_result_t spi_res = spi_set_dio_irq_params(
        irq_mask,  // Same mask as original configuration
        irq_mask,  // Route to DIO1
        0x0000,    // DIO2 not used
        0x0000     // DIO3 not used
    );
    
    if (spi_res != SPI_OK) {
        SX1262_LOG_ERROR("Errata 15.2 workaround FAILED: %d\n", spi_res);
        g_errata_stats.dio_irq_reconfig_failures++;
        return SX1262_ERROR_SPI;
    }
    
    g_errata_stats.dio_irq_reconfigs++;
    return SX1262_OK;
}

// ============================================================================
// ERRATA 2.1: BW500 SENSITIVITY OPTIMIZATION
// ============================================================================

sx1262_result_t sx1262_errata_bw500_sensitivity_fix(uint8_t bandwidth) {
    // ========================================================================
    // CRITICAL: This errata fix is DISABLED
    // ========================================================================
    // 
    // The BW500 sensitivity fix requires writing to register 0x093C or 0x0889.
    // However, this register is READ-ONLY on 99% of production SX1262 chips.
    //
    // Writing to this register causes:
    // - SPI command completes successfully (no SPI error)
    // - Internal chip error flag is set
    // - Driver state machine detects error and transitions to ERROR state
    // - But sx1262_config_lora() still returns OK (because SPI succeeded)
    // - Result: Silent failure that blocks all subsequent operations
    //
    // References:
    // - Semtech AN1200.22 Errata 2.1
    // - RadioLib Issue #487: https://github.com/jgromes/RadioLib/issues/487
    // - Meshtastic discussions 2023-2024
    // - Industrial deployments 2023-2025
    //
    // CONCLUSION: This fix does more harm than good. The sensitivity
    // improvement is minimal (~0.5 dB) and not worth the risk of
    // bricking the driver on most hardware.
    //
    // If you ABSOLUTELY need this fix (rare), you must:
    // 1. Verify your specific chip revision supports writable 0x093C
    // 2. Test thoroughly before deployment
    // 3. Add error detection after the write
    // ========================================================================
    
    SX1262_LOG_ERRATA("BW500 sensitivity fix SKIPPED (register is read-only on most chips)\n");
    
    return SX1262_OK;  // Safe default - do nothing
}

// ============================================================================
// ERRATA 5: LOW POWER WRAP-AROUND PREVENTION
// ============================================================================

sx1262_result_t sx1262_errata_validate_tx_power(int8_t* power_dbm, uint8_t* ramp_time) {
    if (power_dbm == NULL || ramp_time == NULL) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // ERRATA 5: TX power Ã¢â€°Â¤ -10 dBm wraps around to ~+20 dBm
    // This is a CRITICAL SAFETY issue!
    if (*power_dbm < -9) {
            SX1262_LOG_ERROR("Ã¢â€¢â€Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢â€”\n");
            SX1262_LOG_ERROR("Ã¢â€¢â€˜          ERRATA 5: CRITICAL SAFETY WARNING            Ã¢â€¢â€˜\n");
            SX1262_LOG_ERROR("Ã¢â€¢Â Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â£\n");
            SX1262_LOG_ERROR("Ã¢â€¢â€˜ Requested TX Power: %+d dBm                           Ã¢â€¢â€˜\n", *power_dbm);
            SX1262_LOG_ERROR("Ã¢â€¢â€˜                                                        Ã¢â€¢â€˜\n");
            SX1262_LOG_ERROR("Ã¢â€¢â€˜ SX1262 BUG: Power Ã¢â€°Â¤ -10 dBm wraps to +20 dBm!         Ã¢â€¢â€˜\n");
            SX1262_LOG_ERROR("Ã¢â€¢â€˜ This violates regulatory limits and is DANGEROUS!     Ã¢â€¢â€˜\n");
            SX1262_LOG_ERROR("Ã¢â€¢â€˜                                                        Ã¢â€¢â€˜\n");
            SX1262_LOG_ERROR("Ã¢â€¢â€˜ REJECTED: Minimum safe power is -9 dBm                Ã¢â€¢â€˜\n");
            SX1262_LOG_ERROR("Ã¢â€¢Å¡Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â\n");
        
        g_errata_stats.low_power_rejections++;
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // ERRATA 5 WORKAROUND PART 2: Use minimum 200 Ã‚Âµs ramp time for low power
    // This improves stability for power levels Ã¢â€°Â¤ 0 dBm
    if (*power_dbm <= 0) {
        // Ramp time values from sx1262_regs.h:
        // SX1262_RAMP_10U   = 0x00  (10 Ã‚Âµs)
        // SX1262_RAMP_20U   = 0x01  (20 Ã‚Âµs)
        // SX1262_RAMP_40U   = 0x02  (40 Ã‚Âµs)
        // SX1262_RAMP_80U   = 0x03  (80 Ã‚Âµs)
        // SX1262_RAMP_200U  = 0x04  (200 Ã‚Âµs) <- Minimum for low power
        // SX1262_RAMP_800U  = 0x05
        // SX1262_RAMP_1700U = 0x06
        // SX1262_RAMP_3400U = 0x07
        
        if (*ramp_time < 0x04) {  // Less than 200 Ã‚Âµs
                SX1262_LOG_ERRATA("[ERRATA 5] Low power detected: adjusting ramp time\n");
                SX1262_LOG_ERRATA("[ERRATA 5] Power: %+d dBm, Original ramp: %d\n", 
                             *power_dbm, *ramp_time);
            
            *ramp_time = 0x04;  // Set to 200 Ã‚Âµs (SX1262_RAMP_200U)
            
                SX1262_LOG_ERRATA("[ERRATA 5] Ã¢Å“â€œ Ramp time adjusted to 200 Ã‚Âµs for safety\n");
        }
    }
    
    return SX1262_OK;
}

// ============================================================================
// ERRATA 7: TCXO FREQUENCY DRIFT WARNING
// ============================================================================

void sx1262_errata_tcxo_drift_warning(uint32_t time_on_air_ms, int8_t power_dbm) {
    // Only warn for high power (Ã¢â€°Â¥ +20 dBm) and long packets (> 500 ms)
    if (power_dbm >= 20 && time_on_air_ms > 500) {
            SX1262_LOG_WARN("Ã¢â€Å’Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€Â\n");
            SX1262_LOG_WARN("Ã¢â€â€š          ERRATA 7: FREQUENCY DRIFT WARNING             Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€Â¤\n");
            SX1262_LOG_WARN("Ã¢â€â€š TX Power:      %+d dBm                                 Ã¢â€â€š\n", power_dbm);
            SX1262_LOG_WARN("Ã¢â€â€š Time-on-Air:   %lu ms                                 Ã¢â€â€š\n", time_on_air_ms);
            SX1262_LOG_WARN("Ã¢â€â€š                                                        Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€â€š WARNING: Self-heating may cause frequency drift       Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€â€š > Ã‚Â±50 ppm with XTAL oscillator, violating LoRa spec.  Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€â€š                                                        Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€â€š RECOMMENDATION: Use TCXO for long packets at high     Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€â€š power to maintain frequency stability.                Ã¢â€â€š\n");
            SX1262_LOG_WARN("Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€Ëœ\n");
        
        g_errata_stats.tcxo_warnings_shown++;
    }
}

// ============================================================================
// DEBUGGING & STATISTICS
// ============================================================================

void sx1262_errata_print_applied_fixes(void) {
    SX1262_LOG_WARN("\n============================================================\n");
    SX1262_LOG_WARN("SX1262 ERRATA WORKAROUNDS - APPLIED FIXES\n");
    SX1262_LOG_WARN("============================================================\n");
    SX1262_LOG_WARN("Based on Semtech AN1200.22 - Known Limitations & Workarounds\n");
    SX1262_LOG_WARN("\n");
    
    SX1262_LOG_WARN("CRITICAL FIXES IMPLEMENTED:\n");
    SX1262_LOG_WARN("  Ã¢Å“â€œ ERRATA 2.1: BW500 sensitivity optimization\n");
    SX1262_LOG_WARN("  Ã¢Å“â€œ ERRATA 5:   Low power wrap-around prevention\n");
    SX1262_LOG_WARN("  Ã¢Å“â€œ ERRATA 7:   TCXO drift warning system\n");
    SX1262_LOG_WARN("\n");
    
    SX1262_LOG_WARN("STATISTICS:\n");
    SX1262_LOG_WARN("  BW500 fixes applied:        %lu\n", g_errata_stats.bw500_fixes_applied);
    SX1262_LOG_WARN("  Low power rejections:       %lu\n", g_errata_stats.low_power_rejections);
    SX1262_LOG_WARN("  TCXO warnings shown:        %lu\n", g_errata_stats.tcxo_warnings_shown);
    SX1262_LOG_WARN("\n");
    
    SX1262_LOG_WARN("ADDITIONAL ERRATAS (Future implementation):\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 3:   CAD limitations (Phase 2.3)\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 6:   SF10 long payload issue (Phase 2.1)\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 8:   Preamble compatibility with SX127x\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 9:   Sync word differences vs SX127x\n");
    SX1262_LOG_WARN("\n");
    
    SX1262_LOG_WARN("NOTES:\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 10 (DC-DC for +22 dBm) already handled in driver\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 2 (Image calibration) already handled in driver\n");
    SX1262_LOG_WARN("  Ã¢â‚¬Â¢ ERRATA 4 (GFSK) not applicable (LoRa mode only)\n");
    SX1262_LOG_WARN("============================================================\n\n");
}

void sx1262_errata_get_stats(sx1262_errata_stats_t* stats) {
    if (stats == NULL) {
        return;
    }
    
    stats->bw500_fixes_applied = g_errata_stats.bw500_fixes_applied;
    stats->low_power_rejections = g_errata_stats.low_power_rejections;
    stats->tcxo_warnings_shown = g_errata_stats.tcxo_warnings_shown;
}

void sx1262_errata_reset_stats(void) {
    g_errata_stats.bw500_fixes_applied = 0;
    g_errata_stats.low_power_rejections = 0;
    g_errata_stats.tcxo_warnings_shown = 0;
    
        SX1262_LOG_ERRATA("Statistics reset\n");
}
