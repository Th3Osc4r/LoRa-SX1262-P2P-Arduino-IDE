// sx1262_config.cpp - Phase 1.2 Radio Configuration Implementation
// Add this code to sx1262_driver.cpp or create a separate file

#include "sx1262_driver.h"
#include "sx1262_spi_protocol.h"
#include "sx1262_regs.h"
#include "sx1262_errata.h"
#include "config.h"
#include <Arduino.h>

// Import external driver context (defined in sx1262_driver.cpp)
extern sx1262_driver_context_t g_driver_ctx;

// ============================================================================
// BANDWIDTH LOOKUP 
// ============================================================================
// The SX1262 bandwidth register values are non-sequential (see datasheet Table 13-48).
// This function maps enum values to Hz.
static uint32_t get_bandwidth_hz(sx1262_lora_bw_t bw) {
    switch (bw) {
        case SX1262_LORA_BW_7P8:   return 7810;
        case SX1262_LORA_BW_10P4:  return 10420;
        case SX1262_LORA_BW_15P6:  return 15630;
        case SX1262_LORA_BW_20P8:  return 20830;
        case SX1262_LORA_BW_31P25: return 31250;
        case SX1262_LORA_BW_41P7:  return 41670;
        case SX1262_LORA_BW_62P5:  return 62500;
        case SX1262_LORA_BW_125:   return 125000;
        case SX1262_LORA_BW_250:   return 250000;
        case SX1262_LORA_BW_500:   return 500000;
        default:                   return 125000;  // Default to 125kHz
    }
}

// ============================================================================
// DEFAULT CONFIGURATION INITIALIZATION
// ============================================================================

void sx1262_config_init_defaults(sx1262_lora_config_t* config) {
    if (config == NULL) {
        return;
    }
    
    // RF Parameters - EU868, Private Network, Maximum Power
    config->frequency_hz = 868100000;         // 868.1 MHz (EU868 channel)
    config->tx_power_dbm = 22;                // +22 dBm (maximum for SX1262)
    config->ramp_time = SX1262_RAMP_40U;      // 40 Ãƒâ€šÃ‚Âµs ramp time
    
    // LoRa Modulation - Balanced configuration (SF7, BW125, CR4/5)
    config->spreading_factor = SX1262_LORA_SF7;        // SF7 (balanced)
    config->bandwidth = SX1262_LORA_BW_125;            // 125 kHz (standard)
    config->coding_rate = SX1262_LORA_CR_4_5;          // 4/5 (minimal overhead)
    config->low_data_rate_optimize = false;            // Not needed for SF7
    
    // LoRa Packet Parameters
    config->preamble_length = 8;                       // 8 symbols (standard)
    config->header_type = SX1262_LORA_HEADER_EXPLICIT; // Variable length
    config->payload_length = 255;                      // Maximum payload
    config->crc_type = SX1262_LORA_CRC_ON;             // CRC enabled
    config->invert_iq = SX1262_LORA_IQ_NORMAL;         // Standard IQ
    
    // Sync Word - Private Network
    config->sync_word = 0x1424;                        // Private network
    
    // Internal flags
    config->configured = false;
    
        SX1262_LOG_CONFIG("Default configuration initialized:\n");
        SX1262_LOG_CONFIG("  Frequency: %.3f MHz\n", config->frequency_hz / 1e6);
        SX1262_LOG_CONFIG("  TX Power: %+d dBm\n", config->tx_power_dbm);
        SX1262_LOG_CONFIG("  SF: %d, BW: %d kHz, CR: 4/%d\n", 
                     config->spreading_factor,
                     get_bandwidth_hz(config->bandwidth) / 1000,
                     config->coding_rate + 4);
}


// ============================================================================
// PARAMETER VALIDATION
// ============================================================================

sx1262_result_t sx1262_validate_config(const sx1262_lora_config_t* config) {
    if (config == NULL) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate frequency range (150 MHz - 960 MHz)
    if (config->frequency_hz < 150000000 || config->frequency_hz > 960000000) {
            SX1262_LOG_ERROR("[CONFIG] Invalid frequency %lu Hz (must be 150-960 MHz)\n", 
                         config->frequency_hz);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate TX power range for SX1262 (-9 to +22 dBm)
    if (config->tx_power_dbm < -9 || config->tx_power_dbm > 22) {
            SX1262_LOG_ERROR("[CONFIG] Invalid TX power %d dBm (must be -9 to +22)\n", 
                         config->tx_power_dbm);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate spreading factor (SF5 - SF12)
    if (config->spreading_factor < SX1262_LORA_SF5 || 
        config->spreading_factor > SX1262_LORA_SF12) {
            SX1262_LOG_ERROR("[CONFIG] Invalid SF %d (must be SF5-SF12)\n", 
                         config->spreading_factor);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate bandwidth (0-9)
    if (config->bandwidth > SX1262_LORA_BW_500) {
            SX1262_LOG_ERROR("[CONFIG] Invalid BW %d\n", config->bandwidth);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate coding rate (1-4)
    if (config->coding_rate < SX1262_LORA_CR_4_5 || 
        config->coding_rate > SX1262_LORA_CR_4_8) {
            SX1262_LOG_ERROR("[CONFIG] Invalid CR %d (must be 1-4)\n", 
                         config->coding_rate);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate preamble length (min 8, min 12 for SF5/SF6)
    uint16_t min_preamble = (config->spreading_factor <= SX1262_LORA_SF6) ? 12 : 8;
    if (config->preamble_length < min_preamble) {
            SX1262_LOG_ERROR("[CONFIG] Preamble too short (%d, min %d for SF%d)\n",
                         config->preamble_length, min_preamble, config->spreading_factor);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Check if LDRO should be enabled
    bool ldro_required = sx1262_is_ldro_required(config->spreading_factor, config->bandwidth);
    if (ldro_required && !config->low_data_rate_optimize) {
            SX1262_LOG_WARN("[CONFIG] LDRO recommended for SF%d BW%lu\n",
                         config->spreading_factor,
                         get_bandwidth_hz(config->bandwidth));
        // This is a warning, not an error - continue
    }
    
    return SX1262_OK;
}

// ============================================================================
// FREQUENCY CONFIGURATION
// ============================================================================

// ============================================================================
// FREQUENCY CONFIGURATION
// ============================================================================

sx1262_result_t sx1262_set_frequency(uint32_t frequency_hz) {
    if (!g_driver_ctx.hardware_initialized) {
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // Must be in STANDBY mode
    sx1262_state_t state = sx1262_get_state();
    if (state != SX1262_STATE_STANDBY_RC && state != SX1262_STATE_STANDBY_XOSC) {
        SX1262_LOG_ERROR("[CONFIG] Must be in STANDBY to set frequency (current: %s)\n",
                         sx1262_state_to_string(state));
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Validate frequency range
    if (frequency_hz < 150000000 || frequency_hz > 960000000) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Calculate frequency register value
    // Formula: RfFreq = (Frequency * 2^25) / FXTAL
    // FXTAL = 32 MHz
    uint32_t freq_reg = (uint32_t)((uint64_t)frequency_hz * (1ULL << 25) / 32000000ULL);
    
    // Prepare command: SetRfFrequency (4 bytes)
    uint8_t params[4];
    params[0] = (freq_reg >> 24) & 0xFF;
    params[1] = (freq_reg >> 16) & 0xFF;
    params[2] = (freq_reg >> 8) & 0xFF;
    params[3] = freq_reg & 0xFF;
    
    SX1262_LOG_CONFIG("Setting RF frequency to %.3f MHz (reg: 0x%08lX)...\n", 
                     frequency_hz / 1e6, freq_reg);
    
    // Send command
    spi_result_t spi_res = spi_cmd(SX1262_OP_SET_RF_FREQUENCY, params, 4, NULL, 0, 20000, NULL);
    
    if (spi_res != SPI_OK) {
        SX1262_LOG_CONFIG("ERROR: SetRfFrequency command failed (BUSY timeout)\n");
        SX1262_LOG_CONFIG("Chip may still be calibrating - waiting...\n");
    
        // Wait a bit more for calibration to complete
        hal_delay_ms(200);
        
        // Check if BUSY is still high
        if (hal_gpio_read(PIN_SX1262_BUSY)) {
            SX1262_LOG_CONFIG("ERROR: BUSY still HIGH after timeout\n");
            return SX1262_ERROR_TIMEOUT;
        }
        
        // BUSY cleared after wait - continue
        SX1262_LOG_CONFIG("BUSY cleared after wait, continuing...\n");
    }
    
    // Perform image calibration for the frequency band
    // Determine calibration band based on frequency
    uint8_t freq1, freq2;
    
    if (frequency_hz >= 863000000 && frequency_hz <= 870000000) {
        // EU868 band
        freq1 = 0xD7;
        freq2 = 0xDB;
    } else if (frequency_hz >= 902000000 && frequency_hz <= 928000000) {
        // US915/AS923 band
        freq1 = 0xE1;
        freq2 = 0xE9;
    } else if (frequency_hz >= 470000000 && frequency_hz <= 510000000) {
        // CN470 band
        freq1 = 0x75;
        freq2 = 0x81;
    } else if (frequency_hz >= 430000000 && frequency_hz <= 440000000) {
        // 433 MHz band
        freq1 = 0x6B;
        freq2 = 0x6F;
    } else if (frequency_hz >= 779000000 && frequency_hz <= 787000000) {
        // 779-787 MHz band
        freq1 = 0xC1;
        freq2 = 0xC5;
    } else {
        // Default to closest band or skip calibration
        SX1262_LOG_WARN("[CONFIG] No calibration band for %.3f MHz\n", 
                     frequency_hz / 1e6);
        freq1 = 0xE1;  // Default to 902-928 band
        freq2 = 0xE9;
    }
    
    // Calibrate image (commented out - enable if needed)
    uint8_t cal_params[2] = {freq1, freq2};
    spi_res = spi_cmd(SX1262_OP_CALIBRATE_IMAGE, cal_params, 2, NULL, 0, 500000, NULL);
    
    if (spi_res != SPI_OK) {
         SX1262_LOG_CONFIG("WARNING: Image calibration failed\n");
         // Continue anyway - calibration failure is not critical
    } else {
        SX1262_LOG_CONFIG("Image calibration complete for %.3f MHz\n", frequency_hz / 1e6);
    }
    
    // Wait for frequency to settle
    hal_delay_ms(5);
    
    // CRITICAL: Clear any error flags from calibration
    spi_clear_device_errors();
    
    // Update configuration
    g_driver_ctx.config.frequency_hz = frequency_hz;
    g_driver_ctx.config_valid = true;
    
    // Feed watchdog on success
    sx1262_watchdog_feed();
    
    SX1262_LOG_CONFIG("Frequency set to %.3f MHz successfully\n", 
                     frequency_hz / 1e6);
    
    return SX1262_OK;
}

// ============================================================================
// TX POWER CONFIGURATION
// ============================================================================

sx1262_result_t sx1262_set_tx_power(int8_t power_dbm, sx1262_ramp_time_t ramp_time) {
    if (!g_driver_ctx.hardware_initialized) {
        return SX1262_ERROR_NOT_INITIALIZED;
    }

    // ERRATA 5: Validate TX power and adjust ramp time if needed
    // This prevents dangerous wrap-around bug for power ÃƒÂ¢Ã¢â‚¬Â°Ã‚Â¤ -10 dBm
    sx1262_result_t errata_res = sx1262_errata_validate_tx_power(&power_dbm, (uint8_t*)&ramp_time);
    if (errata_res != SX1262_OK) {
        return errata_res;  // Power rejected for safety
    }    
    
    // Must be in STANDBY mode
    sx1262_state_t state = sx1262_get_state();
    if (state != SX1262_STATE_STANDBY_RC && state != SX1262_STATE_STANDBY_XOSC) {
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Configure PA based on desired power level
    // Optimal settings from datasheet (Table 13-21)
    uint8_t pa_duty_cycle, hp_max, device_sel = 0x00;  // SX1262 = 0x00
    uint8_t pa_lut = 0x01;  // Always 0x01
    
    if (power_dbm == 22) {
        // +22 dBm: Maximum power
        pa_duty_cycle = 0x04;
        hp_max = 0x07;
    } else if (power_dbm >= 20) {
        // +20 dBm: Optimized
        pa_duty_cycle = 0x03;
        hp_max = 0x05;
        power_dbm = 22;  // Use +22 setting, actual power limited by PA config
    } else if (power_dbm >= 17) {
        // +17 dBm: Optimized
        pa_duty_cycle = 0x02;
        hp_max = 0x03;
        power_dbm = 22;  // Use +22 setting, actual power limited by PA config
    } else if (power_dbm >= 14) {
        // +14 dBm: Optimized
        pa_duty_cycle = 0x02;
        hp_max = 0x02;
        power_dbm = 14;
    } else {
        // Lower powers: Use default PA settings
        pa_duty_cycle = 0x04;
        hp_max = 0x07;
    }
    
    // Step 1: Configure PA (SetPaConfig)
    uint8_t pa_params[4] = {pa_duty_cycle, hp_max, device_sel, pa_lut};
    spi_result_t spi_res = spi_cmd(SX1262_OP_SET_PA_CONFIG, pa_params, 4, NULL, 0, 0, NULL);
    
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("ERROR: SetPaConfig failed\n");
        return SX1262_ERROR_SPI;
    }
    
    // Step 2: Set TX parameters (SetTxParams)
    uint8_t tx_params[2];
    tx_params[0] = (uint8_t)power_dbm;  // Power in dBm
    tx_params[1] = (uint8_t)ramp_time;  // Ramp time
    
    spi_res = spi_cmd(SX1262_OP_SET_TX_PARAMS, tx_params, 2, NULL, 0, 0, NULL);
    
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("ERROR: SetTxParams failed\n");
        return SX1262_ERROR_SPI;
    }
    
    // Step 3: Configure OCP (Over Current Protection)
    // For SX1262 at +22 dBm, set OCP to 140 mA
    uint8_t ocp_value = 0x38;  // 140 mA for SX1262
    spi_result_t reg_res = spi_write_register(0x08E7, &ocp_value, 1);
    
    if (reg_res != SPI_OK) {
            SX1262_LOG_CONFIG("WARNING: OCP configuration failed\n");
        // Continue anyway - not critical
    }
    
    // Update configuration
    g_driver_ctx.config.tx_power_dbm = power_dbm;
    g_driver_ctx.config.ramp_time = ramp_time;
	g_driver_ctx.config_valid = true;
    
    // Feed watchdog on success
    sx1262_watchdog_feed();
    
        SX1262_LOG_CONFIG("TX Power set to %+d dBm (PA: duty=0x%02X, hp=0x%02X, ramp=%d)\n",
                     power_dbm, pa_duty_cycle, hp_max, ramp_time);
    
    return SX1262_OK;
}

// ============================================================================
// MODULATION PARAMETERS
// ============================================================================

sx1262_result_t sx1262_set_modulation_params(
    sx1262_lora_sf_t sf,
    sx1262_lora_bw_t bw,
    sx1262_lora_cr_t cr,
    bool low_data_rate_optimize
) {
    if (!g_driver_ctx.hardware_initialized) {
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // Must be in STANDBY mode
    sx1262_state_t state = sx1262_get_state();
    if (state != SX1262_STATE_STANDBY_RC && state != SX1262_STATE_STANDBY_XOSC) {
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Validate parameters
    if (sf < SX1262_LORA_SF5 || sf > SX1262_LORA_SF12) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    // Validate bandwidth (non-sequential enum values)
    if (sx1262_get_bandwidth_hz(bw) == 0) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    if (cr < SX1262_LORA_CR_4_5 || cr > SX1262_LORA_CR_4_8) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Check if LDRO is recommended
    bool ldro_recommended = sx1262_is_ldro_required(sf, bw);
    if (ldro_recommended && !low_data_rate_optimize) {
            SX1262_LOG_WARN("[CONFIG] LDRO recommended for SF%d BW%lu kHz\n",
                         sf, get_bandwidth_hz(bw) / 1000);
    }
    
    // Prepare SetModulationParams command
    // Command format: [SF] [BW] [CR] [LDRO]
    uint8_t params[4];
    params[0] = (uint8_t)sf;
    params[1] = (uint8_t)bw;
    params[2] = (uint8_t)cr;
    params[3] = low_data_rate_optimize ? 0x01 : 0x00;
    
    spi_result_t spi_res = spi_cmd(SX1262_OP_SET_MODULATION_PARAMS, params, 4, NULL, 0, 0, NULL);
    
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("ERROR: SetModulationParams failed\n");
        return SX1262_ERROR_SPI;
    }

    // ERRATA 2.1: Apply BW500 sensitivity optimization if needed
    sx1262_result_t errata_res = sx1262_errata_bw500_sensitivity_fix(bw);
    if (errata_res != SX1262_OK && errata_res != SX1262_ERROR_HARDWARE) {
        // Error applying errata fix (but continue anyway)
            SX1262_LOG_CONFIG("WARNING: ERRATA 2.1 fix failed\n");
    }  
    
    // Update configuration
    g_driver_ctx.config.spreading_factor = sf;
    g_driver_ctx.config.bandwidth = bw;
    g_driver_ctx.config.coding_rate = cr;
    g_driver_ctx.config.low_data_rate_optimize = low_data_rate_optimize;
	g_driver_ctx.config_valid = true;
    
    // Feed watchdog on success
    sx1262_watchdog_feed();
    
        SX1262_LOG_CONFIG("Modulation: SF%d, BW%lu kHz, CR4/%d, LDRO=%d\n",
                     sf, get_bandwidth_hz(bw) / 1000, cr + 4, low_data_rate_optimize);
    
    return SX1262_OK;
}

// ============================================================================
// PACKET PARAMETERS
// ============================================================================

sx1262_result_t sx1262_set_packet_params(
    uint16_t preamble_length,
    sx1262_lora_header_t header_type,
    uint8_t payload_length,
    sx1262_lora_crc_t crc_type,
    sx1262_lora_iq_t invert_iq
) {
    if (!g_driver_ctx.hardware_initialized) {
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // Must be in STANDBY mode
    sx1262_state_t state = sx1262_get_state();
    if (state != SX1262_STATE_STANDBY_RC && state != SX1262_STATE_STANDBY_XOSC) {
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Validate preamble length (min 8)
    // Note: SF5/SF6 require min 12 symbols, but we check this in validate_config
    // Here we just enforce a general minimum of 8 symbols
    if (preamble_length < 8) {
            SX1262_LOG_ERROR("[CONFIG] Preamble too short (%d, min 8)\n",
                         preamble_length);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Prepare SetPacketParams command
    // Command format: [PreambleLen_MSB] [PreambleLen_LSB] [HeaderType] [PayloadLen] [CRC] [IQ]
    uint8_t params[6];
    params[0] = (preamble_length >> 8) & 0xFF;
    params[1] = preamble_length & 0xFF;
    params[2] = (uint8_t)header_type;
    params[3] = payload_length;
    params[4] = (uint8_t)crc_type;
    params[5] = (uint8_t)invert_iq;
    
    spi_result_t spi_res = spi_cmd(SX1262_OP_SET_PACKET_PARAMS, params, 6, NULL, 0, 0, NULL);
    
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("ERROR: SetPacketParams failed\n");
        return SX1262_ERROR_SPI;
    }
    
    // Update configuration
    g_driver_ctx.config.preamble_length = preamble_length;
    g_driver_ctx.config.header_type = header_type;
    g_driver_ctx.config.payload_length = payload_length;
    g_driver_ctx.config.crc_type = crc_type;
    g_driver_ctx.config.invert_iq = invert_iq;
	g_driver_ctx.config_valid = true;
    
    // Feed watchdog on success
    sx1262_watchdog_feed();
    
        SX1262_LOG_CONFIG("Packet: Preamble=%d, Header=%s, Payload=%d, CRC=%s, IQ=%s\n",
                     preamble_length,
                     header_type == SX1262_LORA_HEADER_EXPLICIT ? "Explicit" : "Implicit",
                     payload_length,
                     crc_type == SX1262_LORA_CRC_ON ? "ON" : "OFF",
                     invert_iq == SX1262_LORA_IQ_NORMAL ? "Normal" : "Inverted");
    
    return SX1262_OK;
}

// ============================================================================
// SYNC WORD CONFIGURATION
// ============================================================================

sx1262_result_t sx1262_set_sync_word(uint16_t sync_word) {
    if (!g_driver_ctx.hardware_initialized) {
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    // Must be in STANDBY mode
    sx1262_state_t state = sx1262_get_state();
    if (state != SX1262_STATE_STANDBY_RC && state != SX1262_STATE_STANDBY_XOSC) {
        return SX1262_ERROR_INVALID_STATE;
    }
    
    // Write sync word to registers 0x0740-0x0741
    uint8_t sync_bytes[2];
    sync_bytes[0] = (sync_word >> 8) & 0xFF;  // MSB
    sync_bytes[1] = sync_word & 0xFF;         // LSB
    
    spi_result_t spi_res = spi_write_register(0x0740, sync_bytes, 2);
    
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("ERROR: Sync word write failed\n");
        return SX1262_ERROR_SPI;
    }
    
    // Update configuration
    g_driver_ctx.config.sync_word = sync_word;
	g_driver_ctx.config_valid = true;
    
    // Feed watchdog on success
    sx1262_watchdog_feed();
    
        SX1262_LOG_CONFIG("Sync word set to 0x%04X (%s network)\n",
                     sync_word,
                     sync_word == 0x3444 ? "Public" : "Private");
    
    return SX1262_OK;
}

// ============================================================================
// COMPLETE CONFIGURATION
// ============================================================================

sx1262_result_t sx1262_config_lora(const sx1262_lora_config_t* config) {
    if (!g_driver_ctx.hardware_initialized) {
        return SX1262_ERROR_NOT_INITIALIZED;
    }
    
    if (config == NULL) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
        SX1262_LOG_CONFIG("Applying LoRa configuration...\n");
    
    // Step 1: Validate configuration
    sx1262_result_t res = sx1262_validate_config(config);
    if (res != SX1262_OK) {
            SX1262_LOG_CONFIG("ERROR: Configuration validation failed\n");
        return res;
    }
    
    // Step 2: Ensure we're in STANDBY_RC mode
    if (sx1262_get_state() != SX1262_STATE_STANDBY_RC) {
        res = sx1262_set_state(SX1262_STATE_STANDBY_RC);
        if (res != SX1262_OK) {
                SX1262_LOG_CONFIG("ERROR: Failed to enter STANDBY_RC\n");
            return res;
        }
    }
    
    // Step 3: Set packet type to LoRa
    uint8_t packet_type = PACKET_TYPE_LORA;
    spi_result_t spi_res = spi_cmd(SX1262_OP_SET_PACKET_TYPE, &packet_type, 1, NULL, 0, 0, NULL);
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("ERROR: SetPacketType failed\n");
        return SX1262_ERROR_SPI;
    }
    
    // Step 4: Set regulator mode to DC-DC (for optimal efficiency)
    uint8_t reg_mode = REGULATOR_MODE_DC_DC;
    spi_res = spi_cmd(SX1262_OP_SET_REGULATOR_MODE, &reg_mode, 1, NULL, 0, 0, NULL);
    if (spi_res != SPI_OK) {
            SX1262_LOG_CONFIG("WARNING: SetRegulatorMode failed\n");
        // Continue anyway - not critical
    }
    
    // Step 5: Configure RF frequency
    res = sx1262_set_frequency(config->frequency_hz);
    if (res != SX1262_OK) {
            SX1262_LOG_CONFIG("ERROR: Frequency configuration failed\n");
        return res;
    }
    
    // Step 6: Configure TX power
    res = sx1262_set_tx_power(config->tx_power_dbm, config->ramp_time);
    if (res != SX1262_OK) {
            SX1262_LOG_CONFIG("ERROR: TX power configuration failed\n");
        return res;
    }
    
    // Step 7: Set modulation parameters
    res = sx1262_set_modulation_params(
        config->spreading_factor,
        config->bandwidth,
        config->coding_rate,
        config->low_data_rate_optimize
    );
    if (res != SX1262_OK) {
            SX1262_LOG_CONFIG("ERROR: Modulation parameters failed\n");
        return res;
    }
    
    // Step 8: Set packet parameters
    res = sx1262_set_packet_params(
        config->preamble_length,
        config->header_type,
        config->payload_length,
        config->crc_type,
        config->invert_iq
    );
    if (res != SX1262_OK) {
            SX1262_LOG_CONFIG("ERROR: Packet parameters failed\n");
        return res;
    }
    
    // Step 9: Set sync word
    res = sx1262_set_sync_word(config->sync_word);
    if (res != SX1262_OK) {
            SX1262_LOG_CONFIG("ERROR: Sync word configuration failed\n");
        return res;
    }
    
    // Step 10: Copy configuration to driver context
    memcpy(&g_driver_ctx.config, config, sizeof(sx1262_lora_config_t));
    g_driver_ctx.config.configured = true;
    g_driver_ctx.config_valid = true;
	g_driver_ctx.radio_configured = true;  // Enable fast-path for TX/RX
    
    // Feed watchdog on complete success
    sx1262_watchdog_feed();
    
        
    
    // ========================================================================
    // CRITICAL: Clear any error flags that may have been set during config
    // ========================================================================
    // Some configuration commands (especially calibration, PA config, and
    // frequency setting) can internally trigger chip error flags even though
    // the SPI command itself succeeds. These flags MUST be cleared or the
    // chip will refuse to enter TX/RX modes.
    //
    // This is separate from the clear in driver_init() because:
    // - driver_init() clears errors from previous sessions
    // - This clears errors generated BY the configuration process itself
    // ========================================================================
    SX1262_LOG_CONFIG("Clearing any residual error flags after configuration...\n");
    spi_result_t clear_res = spi_clear_device_errors();
    if (clear_res != SPI_OK) {
        SX1262_LOG_CONFIG("WARNING: Failed to clear errors after config (continuing anyway)\n");
        // Don't fail here - config was already applied successfully
    }
    
    // ========================================================================
    // CRITICAL: Update driver state to match hardware
    // ========================================================================
    // After configuration, the chip is in STANDBY mode (either RC or XOSC).
    // We need to update the driver context to reflect this, otherwise
    // sx1262_transmit() will think the driver isn't initialized properly.
    // ========================================================================
    
    // Read actual chip status to determine which STANDBY mode we're in
    uint8_t status;
    if (spi_get_status(&status) == SPI_OK) {
        uint8_t mode = (status >> 4) & 0x07;
        if (mode == 2) {
            g_driver_ctx.current_state = SX1262_STATE_STANDBY_RC;
            SX1262_LOG_CONFIG("Chip in STANDBY_RC after config\n");
        } else if (mode == 3) {
            g_driver_ctx.current_state = SX1262_STATE_STANDBY_XOSC;
            SX1262_LOG_CONFIG("Chip in STANDBY_XOSC after config\n");
        } else {
            // Unexpected mode, but continue
            g_driver_ctx.current_state = SX1262_STATE_STANDBY_RC;
            SX1262_LOG_CONFIG("WARNING: Unexpected chip mode %d after config\n", mode);
        }
    } else {
        // Can't read status, assume STANDBY_RC
        g_driver_ctx.current_state = SX1262_STATE_STANDBY_RC;
    }
    
    SX1262_LOG_CONFIG("Ã¢Å“â€œ LoRa configuration applied successfully\n");
    sx1262_print_config();
    
    return SX1262_OK;
}

// ============================================================================
// CONFIGURATION RETRIEVAL
// ============================================================================

sx1262_result_t sx1262_get_config(sx1262_lora_config_t* config) {
    if (config == NULL) {
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    if (!g_driver_ctx.config_valid) {
        return SX1262_ERROR_INVALID_STATE;
    }
    
    memcpy(config, &g_driver_ctx.config, sizeof(sx1262_lora_config_t));
    return SX1262_OK;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

uint32_t sx1262_get_bandwidth_hz(sx1262_lora_bw_t bw) {
    // Validate that bw is a known value (non-sequential enum)
    uint32_t hz = get_bandwidth_hz(bw);
    // get_bandwidth_hz returns 125000 for unknown values, but we should return 0 for invalid
    switch (bw) {
        case SX1262_LORA_BW_7P8:
        case SX1262_LORA_BW_10P4:
        case SX1262_LORA_BW_15P6:
        case SX1262_LORA_BW_20P8:
        case SX1262_LORA_BW_31P25:
        case SX1262_LORA_BW_41P7:
        case SX1262_LORA_BW_62P5:
        case SX1262_LORA_BW_125:
        case SX1262_LORA_BW_250:
        case SX1262_LORA_BW_500:
            return hz;
        default:
            return 0;  // Invalid bandwidth
    }
}

bool sx1262_is_ldro_required(sx1262_lora_sf_t sf, sx1262_lora_bw_t bw) {
    // LDRO should be enabled when symbol time >= 16.38 ms
    // Symbol time = (2^SF) / BW
    
    uint32_t bw_hz = sx1262_get_bandwidth_hz(bw);
    if (bw_hz == 0) {
        return false;
    }
    
    // Calculate symbol time in microseconds
    // symbol_time_us = (1000000 * (1 << sf)) / bw_hz
    uint64_t symbol_time_us = (1000000ULL * (1ULL << sf)) / bw_hz;
    
    // LDRO required if symbol time >= 16380 Ãƒâ€šÃ‚Âµs (16.38 ms)
    return (symbol_time_us >= 16380);
}

uint32_t sx1262_get_time_on_air_ms(const sx1262_lora_config_t* config, uint8_t payload_length) {
    // ============================================================================
    // CRITICAL: Comprehensive safety checks to prevent division by zero
    // ============================================================================
    if (config == NULL) {
            SX1262_LOG_CONFIG("ERROR: ToA calculation - config is NULL\n");
        return 0;
    }
    
    // Check if configuration is valid
    if (!g_driver_ctx.config_valid) {
            SX1262_LOG_CONFIG("ERROR: ToA calculation - config not valid\n");
        return 0;
    }
    
    // Get bandwidth in Hz
    uint32_t bw_hz = sx1262_get_bandwidth_hz(config->bandwidth);
    
    // CRITICAL: Check for zero bandwidth
    if (bw_hz == 0) {
            SX1262_LOG_CONFIG("ERROR: ToA calculation - bandwidth is 0\n");
        return 0;
    }
    
    // Check for zero spreading factor
    if (config->spreading_factor == 0 || config->spreading_factor > 12) {
            SX1262_LOG_ERROR("[CONFIG] ToA calculation - invalid SF: %d\n", 
                         config->spreading_factor);
        return 0;
    }
    // ============================================================================
    
    // Calculate symbol duration (ms)
    float symbol_duration_ms = (1000.0 * (1 << config->spreading_factor)) / bw_hz;
    
    // Calculate preamble time
    float preamble_time_ms = (config->preamble_length + 4.25) * symbol_duration_ms;
    
    // Calculate payload symbol count
    int header_mode = (config->header_type == SX1262_LORA_HEADER_IMPLICIT) ? 0 : 1;
    int crc_mode = (config->crc_type == SX1262_LORA_CRC_ON) ? 1 : 0;
    int ldro = config->low_data_rate_optimize ? 1 : 0;
    
    float payload_symbols = 8 + max(
        (int)ceil((8.0 * payload_length - 4 * config->spreading_factor + 28 + 16 * crc_mode - 20 * header_mode) / 
                  (4.0 * (config->spreading_factor - 2 * ldro))) * (config->coding_rate + 4),
        0
    );
    
    float payload_time_ms = payload_symbols * symbol_duration_ms;
    
    return (uint32_t)(preamble_time_ms + payload_time_ms);
}

// ============================================================================
// DEBUG PRINTING
// ============================================================================

void sx1262_print_config() {
    if (!g_driver_ctx.config_valid) {
        SX1262_LOG_INFO("No valid configuration\n");
        return;
    }
    
    sx1262_lora_config_t* cfg = &g_driver_ctx.config;
    
    // Determine ramp time value for display
    int ramp_us = (cfg->ramp_time == SX1262_RAMP_10U) ? 10 :
                  (cfg->ramp_time == SX1262_RAMP_20U) ? 20 :
                  (cfg->ramp_time == SX1262_RAMP_40U) ? 40 :
                  (cfg->ramp_time == SX1262_RAMP_80U) ? 80 :
                  (cfg->ramp_time == SX1262_RAMP_200U) ? 200 :
                  (cfg->ramp_time == SX1262_RAMP_800U) ? 800 :
                  (cfg->ramp_time == SX1262_RAMP_1700U) ? 1700 : 3400;
    
    SX1262_LOG_INFO("\n============================================================\n");
    SX1262_LOG_INFO("SX1262 LORA CONFIGURATION\n");
    SX1262_LOG_INFO("============================================================\n");
    SX1262_LOG_INFO("RF Frequency:   %.3f MHz\n", cfg->frequency_hz / 1e6);
    SX1262_LOG_INFO("TX Power:       %+d dBm\n", cfg->tx_power_dbm);
    SX1262_LOG_INFO("Ramp Time:      %d us\n", ramp_us);
    SX1262_LOG_INFO("\nModulation:\n");
    SX1262_LOG_INFO("  Spreading Factor: SF%d\n", cfg->spreading_factor);
    SX1262_LOG_INFO("  Bandwidth:        %lu kHz\n", get_bandwidth_hz(cfg->bandwidth) / 1000);
    SX1262_LOG_INFO("  Coding Rate:      4/%d\n", cfg->coding_rate + 4);
    SX1262_LOG_INFO("  LDRO:             %s\n", cfg->low_data_rate_optimize ? "Enabled" : "Disabled");
    SX1262_LOG_INFO("\nPacket:\n");
    SX1262_LOG_INFO("  Preamble Length:  %d symbols\n", cfg->preamble_length);
    SX1262_LOG_INFO("  Header Type:      %s\n", 
                 cfg->header_type == SX1262_LORA_HEADER_EXPLICIT ? "Explicit" : "Implicit");
    SX1262_LOG_INFO("  Payload Length:   %d bytes\n", cfg->payload_length);
    SX1262_LOG_INFO("  CRC:              %s\n", cfg->crc_type == SX1262_LORA_CRC_ON ? "Enabled" : "Disabled");
    SX1262_LOG_INFO("  IQ Polarity:      %s\n", cfg->invert_iq == SX1262_LORA_IQ_NORMAL ? "Normal" : "Inverted");
    SX1262_LOG_INFO("\nSync Word:        0x%04X (%s)\n", 
                 cfg->sync_word,
                 cfg->sync_word == 0x3444 ? "Public/LoRaWAN" : "Private");
    SX1262_LOG_INFO("\nTime-on-Air (estimated):\n");
    SX1262_LOG_INFO("  10 bytes:  %lu ms\n", sx1262_get_time_on_air_ms(cfg, 10));
    SX1262_LOG_INFO("  50 bytes:  %lu ms\n", sx1262_get_time_on_air_ms(cfg, 50));
    SX1262_LOG_INFO("  255 bytes: %lu ms\n", sx1262_get_time_on_air_ms(cfg, 255));
    SX1262_LOG_INFO("============================================================\n\n");
}

// ============================================================================
// SIMPLIFIED INITIALIZATION API
// ============================================================================
// These functions provide a user-friendly way to initialize the driver with
// a single call, hiding the complexity of the multi-step initialization
// process while still allowing customization of key parameters.
// ============================================================================

/**
 * Internal helper to perform the actual initialization
 * Used by both sx1262_init_simple() and sx1262_init_extended()
 */
static sx1262_result_t _init_with_config(
    uint32_t frequency_hz,
    int8_t tx_power_dbm,
    sx1262_lora_sf_t sf,
    sx1262_lora_bw_t bw
) {
    sx1262_result_t res;
    
    SX1262_LOG_INFO("=== SX1262 Unified Initialization ===\n");
    SX1262_LOG_INFO("Frequency: %.3f MHz, Power: %+d dBm, SF%d, BW%lu kHz\n",
                   frequency_hz / 1e6, tx_power_dbm, sf, sx1262_get_bandwidth_hz(bw) / 1000);
    
    // ========================================================================
    // STEP 1: Parameter validation (fail fast before touching hardware)
    // ========================================================================
    
    // Validate frequency range
    if (frequency_hz < 150000000 || frequency_hz > 960000000) {
        SX1262_LOG_ERROR("[INIT] Invalid frequency: %lu Hz (must be 150-960 MHz)\n", frequency_hz);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate TX power range
    if (tx_power_dbm < -9 || tx_power_dbm > 22) {
        SX1262_LOG_ERROR("[INIT] Invalid TX power: %d dBm (must be -9 to +22)\n", tx_power_dbm);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate spreading factor
    if (sf < SX1262_LORA_SF5 || sf > SX1262_LORA_SF12) {
        SX1262_LOG_ERROR("[INIT] Invalid spreading factor: %d (must be SF5-SF12)\n", sf);
        return SX1262_ERROR_INVALID_PARAM;
    }
    
    // Validate bandwidth (check for known enum values)
    switch (bw) {
        case SX1262_LORA_BW_7P8:
        case SX1262_LORA_BW_10P4:
        case SX1262_LORA_BW_15P6:
        case SX1262_LORA_BW_20P8:
        case SX1262_LORA_BW_31P25:
        case SX1262_LORA_BW_41P7:
        case SX1262_LORA_BW_62P5:
        case SX1262_LORA_BW_125:
        case SX1262_LORA_BW_250:
        case SX1262_LORA_BW_500:
            break;  // Valid
        default:
            SX1262_LOG_ERROR("[INIT] Invalid bandwidth value: 0x%02X\n", bw);
            return SX1262_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_INFO("[INIT] Parameters validated OK\n");
    
    // ========================================================================
    // STEP 2: Driver initialization (hardware setup)
    // ========================================================================
    
    SX1262_LOG_INFO("[INIT] Initializing driver...\n");
    res = sx1262_driver_init();
    if (res != SX1262_OK) {
        SX1262_LOG_ERROR("[INIT] Driver initialization failed: %s\n", sx1262_error_to_string(res));
        return res;
    }
    SX1262_LOG_INFO("[INIT] Driver initialized OK\n");
    
    // ========================================================================
    // STEP 3: Build configuration structure
    // ========================================================================
    
    sx1262_lora_config_t config;
    sx1262_config_init_defaults(&config);
    
    // Override with user-specified values
    config.frequency_hz = frequency_hz;
    config.tx_power_dbm = tx_power_dbm;
    config.spreading_factor = sf;
    config.bandwidth = bw;
    
    // Auto-detect LDRO requirement
    // LDRO should be enabled when symbol time >= 16.38 ms
    config.low_data_rate_optimize = sx1262_is_ldro_required(sf, bw);
    if (config.low_data_rate_optimize) {
        SX1262_LOG_INFO("[INIT] LDRO auto-enabled for SF%d/BW%lu\n", 
                       sf, sx1262_get_bandwidth_hz(bw) / 1000);
    }
    
    // Adjust preamble for SF5/SF6 (require minimum 12 symbols)
    if (sf <= SX1262_LORA_SF6 && config.preamble_length < 12) {
        config.preamble_length = 12;
        SX1262_LOG_INFO("[INIT] Preamble adjusted to 12 symbols for SF%d\n", sf);
    }
    
    // ========================================================================
    // STEP 4: Apply radio configuration
    // ========================================================================
    
    SX1262_LOG_INFO("[INIT] Applying radio configuration...\n");
    res = sx1262_config_lora(&config);
    if (res != SX1262_OK) {
        SX1262_LOG_ERROR("[INIT] Radio configuration failed: %s\n", sx1262_error_to_string(res));
        // Attempt cleanup
        sx1262_driver_deinit();
        return res;
    }
    SX1262_LOG_INFO("[INIT] Radio configured OK\n");
    
    // ========================================================================
    // STEP 5: Verify chip is ready
    // ========================================================================
    
    uint8_t status;
    spi_result_t spi_res = spi_get_status(&status);
    if (spi_res != SPI_OK) {
        SX1262_LOG_ERROR("[INIT] Failed to read chip status after configuration\n");
        sx1262_driver_deinit();
        return SX1262_ERROR_HARDWARE;
    }
    
    // Extract chip mode from status byte (bits 6:4)
    uint8_t chip_mode = (status >> 4) & 0x07;
    
    // Mode 2 = STANDBY_RC, Mode 3 = STANDBY_XOSC
    if (chip_mode != 2 && chip_mode != 3) {
        SX1262_LOG_ERROR("[INIT] Chip in unexpected mode %d (expected STANDBY)\n", chip_mode);
        sx1262_driver_deinit();
        return SX1262_ERROR_HARDWARE;
    }
    
    // ========================================================================
    // STEP 6: Enable watchdog for operational safety
    // ========================================================================
    
    sx1262_watchdog_init(3);   // Reset after 3 consecutive failures
    sx1262_watchdog_enable(true);
    SX1262_LOG_INFO("[INIT] Watchdog enabled (max 3 failures)\n");
    
    // ========================================================================
    // SUCCESS
    // ========================================================================
    
    SX1262_LOG_INFO("=== SX1262 Initialization Complete ===\n");
    SX1262_LOG_INFO("Radio ready for TX/RX operations\n");
    
    return SX1262_OK;
}

// ============================================================================
// PUBLIC API: sx1262_init_simple()
// ============================================================================

sx1262_result_t sx1262_init_simple(uint32_t frequency_hz, int8_t tx_power_dbm) {
    // Use defaults: SF7, BW125 - balanced configuration for most applications
    return _init_with_config(
        frequency_hz,
        tx_power_dbm,
        SX1262_LORA_SF7,
        SX1262_LORA_BW_125
    );
}

// ============================================================================
// PUBLIC API: sx1262_init_extended()
// ============================================================================

sx1262_result_t sx1262_init_extended(
    uint32_t frequency_hz, 
    int8_t tx_power_dbm,
    sx1262_lora_sf_t sf,
    sx1262_lora_bw_t bw
) {
    return _init_with_config(frequency_hz, tx_power_dbm, sf, bw);
}

// ============================================================================
// PUBLIC API: sx1262_get_init_error_help()
// ============================================================================

void sx1262_get_init_error_help(sx1262_result_t error, char* buffer, size_t len) {
    if (buffer == NULL || len == 0) {
        return;
    }
    
    const char* help_text;
    
    switch (error) {
        case SX1262_OK:
            help_text = "No error - initialization successful";
            break;
            
        case SX1262_ERROR_INVALID_PARAM:
            help_text = "Invalid parameter. Check: frequency (150-960 MHz), "
                        "power (-9 to +22 dBm), SF (5-12), BW (valid enum)";
            break;
            
        case SX1262_ERROR_INVALID_STATE:
            help_text = "Driver in invalid state. Try calling sx1262_driver_deinit() first, "
                        "or power cycle the module";
            break;
            
        case SX1262_ERROR_TIMEOUT:
            help_text = "Operation timed out. Check SPI wiring and clock speed. "
                        "Verify BUSY pin is connected correctly";
            break;
            
        case SX1262_ERROR_SPI:
            help_text = "SPI communication error. Verify wiring: MOSI, MISO, SCK, CS pins. "
                        "Check SPI mode (should be Mode 0). Try lower SPI clock speed";
            break;
            
        case SX1262_ERROR_BUSY_TIMEOUT:
            help_text = "BUSY pin timeout. Check BUSY pin connection (should be GPIO input). "
                        "Verify module has power. Try hardware reset via NRST pin";
            break;
            
        case SX1262_ERROR_NOT_INITIALIZED:
            help_text = "Driver not initialized. Call sx1262_init_simple() or "
                        "sx1262_driver_init() first";
            break;
            
        case SX1262_ERROR_MUTEX:
            help_text = "Mutex error (FreeRTOS). Check heap memory is sufficient for "
                        "semaphore allocation. Verify FreeRTOS is running";
            break;
            
        case SX1262_ERROR_WATCHDOG:
            help_text = "Watchdog triggered due to repeated failures. Hardware may be "
                        "malfunctioning. Power cycle the module and try again";
            break;
            
        case SX1262_ERROR_HARDWARE:
            help_text = "Hardware error. Module may be damaged or not responding. "
                        "Verify power supply (3.3V), check antenna connection, "
                        "try a different module if available";
            break;
            
        default:
            help_text = "Unknown error code. Check driver logs for details";
            break;
    }
    
    // Copy to buffer with truncation protection
    strncpy(buffer, help_text, len - 1);
    buffer[len - 1] = '\0';
}