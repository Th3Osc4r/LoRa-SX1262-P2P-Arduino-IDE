// sx1262_spi_protocol.cpp
// 
// VERSION 2 CHANGES:
// - spi_set_tx(): Modified to NOT wait for BUSY LOW after sending SetTx command.
//   BUSY stays HIGH during entire TX operation (Time-on-Air). The old behavior
//   caused ~ToA ms of redundant delay since the caller also waits for TX_DONE ISR.
//   This fix reduces TX Duration from ~2x ToA to ~1x ToA.
//
// VERIFIED WORKING: Diagnostic testing confirmed spi_set_tx returns in 1ms (not 30ms)
//
#include "sx1262_spi_protocol.h"
#include "sx1262_hal.h"
#include "config.h"
#include "sx1262_regs.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// ============ STATIC VARIABLES ============
static spi_command_diag_t last_diag = {0};
static const uint32_t default_busy_timeout_us = TIMING_BUSY_POLL_TIMEOUT_MS * 1000;

// ============ INITIALIZATION ============
spi_result_t spi_init(uint32_t spi_freq_hz) {
    hal_result_t res = hal_spi_init(spi_freq_hz, PIN_SX1262_CS);
    return (res == HAL_OK) ? SPI_OK : SPI_ERROR_HAL;
}

void spi_deinit() {
    hal_spi_deinit();
}

// ============ CORE SPI COMMAND WITH BUSY HANDLING (FIXED FOR SX1262 GET_STATUS) ============
// ============ CORE SPI COMMAND WITH BUSY HANDLING ============
spi_result_t spi_cmd(
    uint8_t opcode,
    const uint8_t *tx_params,
    uint16_t tx_len,
    uint8_t *rx_data,
    uint16_t rx_len,
    uint32_t busy_timeout_us,
    spi_command_diag_t *diag
) {
    uint32_t start_total = hal_get_time_us();
    if (busy_timeout_us == 0) busy_timeout_us = default_busy_timeout_us;

    if (diag) {
        memset(diag, 0, sizeof(spi_command_diag_t));
        diag->opcode = opcode;
        diag->busy_timeout_used_us = busy_timeout_us;
    }
    memset(&last_diag, 0, sizeof(last_diag));
    last_diag.opcode = opcode;
    last_diag.busy_timeout_used_us = busy_timeout_us;

    // ============================================================================
    // ENHANCED LOGGING: Step 1 - Wait for BUSY LOW before command
    // ============================================================================
        SX1262_LOG_SPI(">>> CMD 0x%02X start, BUSY=%d\n", 
                     opcode, hal_gpio_read(PIN_SX1262_BUSY));

    uint32_t busy_start = hal_get_time_us();
    bool initial_busy_was_high = hal_gpio_read(PIN_SX1262_BUSY);
    
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_us() - busy_start) > busy_timeout_us) {
                SX1262_LOG_ERROR("[SPI] TIMEOUT waiting for initial BUSY LOW (cmd 0x%02X)\n", opcode);
            if (diag) diag->error_flag = SPI_ERROR_BUSY_TIMEOUT;
            last_diag.error_flag = SPI_ERROR_BUSY_TIMEOUT;
            return SPI_ERROR_BUSY_TIMEOUT;
        }
    }
    
    if (initial_busy_was_high && DEBUG_VERBOSE_SPI) {
        uint32_t wait_time = hal_get_time_us() - busy_start;
        SX1262_LOG_SPI("    Initial BUSY cleared after %lu us\n", wait_time);
    }

    // ============================================================================
    // Step 2-5: Send command
    // ============================================================================
    hal_spi_cs_assert(1);  // CS LOW - start transaction
    
        SX1262_LOG_SPI("    CS LOW, sending %d bytes\n", 1 + tx_len);

        if (rx_len > 0 && rx_data) {
        // READ OPERATION
        uint16_t total_tx = 1 + tx_len;
        uint16_t total_rx = total_tx + rx_len;
        
        SX1262_LOG_DEBUG("[SPI] spi_cmd READ: opcode=0x%02X, tx_len=%d, rx_len=%d\n", 
                        opcode, tx_len, rx_len);
        SX1262_LOG_DEBUG("[SPI] spi_cmd READ: total_tx=%d, total_rx=%d\n", 
                        total_tx, total_rx);
        
        if (total_rx > SPI_CMD_MAX_BUFFER_SIZE) {
            hal_spi_cs_assert(0);
            if (diag) diag->error_flag = SPI_ERROR_BUFFER_TOO_LARGE;
            last_diag.error_flag = SPI_ERROR_BUFFER_TOO_LARGE;
            return SPI_ERROR_BUFFER_TOO_LARGE;
        }

        uint8_t tx_buf[SPI_CMD_MAX_BUFFER_SIZE];
        uint8_t rx_buf[SPI_CMD_MAX_BUFFER_SIZE];
        
        tx_buf[0] = opcode;
        if (tx_len > 0) {
            memcpy(tx_buf + 1, tx_params, tx_len);
        }
        for (uint16_t i = total_tx; i < total_rx; i++) {
            tx_buf[i] = 0x00;
        }
        
        hal_result_t res = hal_spi_transfer(tx_buf, rx_buf, total_rx);
        
        hal_spi_cs_assert(0);  // CS HIGH - end transaction
        
            SX1262_LOG_SPI("    CS HIGH, transfer result=%d\n", res);
        
            if (res != HAL_OK) {
                last_diag.error_flag = SPI_ERROR_SPI_READ;
                return SPI_ERROR_SPI_READ;
            }
        
        hal_delay_us(20);
        
        // Diagnostic: Show what we received
        SX1262_LOG_DEBUG("[SPI] spi_cmd READ: rx_buf after transfer = ");
        for (uint16_t i = 0; i < total_rx; i++) {
            SX1262_LOG_DEBUG("0x%02X ", rx_buf[i]);
        }
        SX1262_LOG_DEBUG("\n");
        
        last_diag.status_byte = rx_buf[0];
        last_diag.irq_bits = (rx_buf[0] & STATUS_IRQ_BITS_MASK) >> 4;
        memcpy(rx_data, rx_buf + total_tx, rx_len);
        
        SX1262_LOG_DEBUG("[SPI] spi_cmd READ: Copied %d bytes from rx_buf[%d] to rx_data\n", 
                         rx_len, total_tx);

        } else {
        // WRITE OPERATION
        uint16_t total_tx = 1 + tx_len; 
        
        if (total_tx > SPI_CMD_MAX_BUFFER_SIZE) {
             hal_spi_cs_assert(0);
             last_diag.error_flag = SPI_ERROR_BUFFER_TOO_LARGE;
             return SPI_ERROR_BUFFER_TOO_LARGE;
        }

        uint8_t tx_buf[SPI_CMD_MAX_BUFFER_SIZE];
        uint8_t rx_buf_temp[SPI_CMD_MAX_BUFFER_SIZE]; 
        
        tx_buf[0] = opcode;
        if (tx_len > 0) {
            memcpy(tx_buf + 1, tx_params, tx_len);
        }

        hal_result_t res = hal_spi_transfer(tx_buf, rx_buf_temp, total_tx); 
        
        hal_spi_cs_assert(0);  // CS HIGH - end transaction
        
            SX1262_LOG_SPI("    CS HIGH, transfer result=%d\n", res);
        
        if (res != HAL_OK) {
            last_diag.error_flag = SPI_ERROR_SPI_WRITE;
            return SPI_ERROR_SPI_WRITE;
        }

        hal_delay_us(20);

        last_diag.status_byte = rx_buf_temp[0]; 
        last_diag.irq_bits = (rx_buf_temp[0] & STATUS_IRQ_BITS_MASK) >> 4;
    }
    
    

    

    // ============================================================================
    // ENHANCED LOGGING: Step 6 - Wait for BUSY LOW after command
    // ============================================================================
        SX1262_LOG_SPI("    After 20us delay, BUSY=%d\n", hal_gpio_read(PIN_SX1262_BUSY));
    
    busy_start = hal_get_time_us();
    bool busy_went_high = false;
    uint32_t busy_high_time = 0;
    
    // First, detect if BUSY goes HIGH
    uint32_t detect_start = hal_get_time_us();
    while ((hal_get_time_us() - detect_start) < 1000) {  // 1ms to detect HIGH
        if (hal_gpio_read(PIN_SX1262_BUSY)) {
            busy_went_high = true;
            busy_high_time = hal_get_time_us();
                SX1262_LOG_SPI("    BUSY went HIGH after %lu us\n", 
                             busy_high_time - detect_start);
            break;
        }
    }
    
    // Now wait for BUSY to go LOW
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_us() - busy_start) > busy_timeout_us) {
                SX1262_LOG_ERROR("[SPI] TIMEOUT waiting for BUSY LOW after cmd 0x%02X\n", opcode);
                SX1262_LOG_SPI("    BUSY was HIGH for > %lu us\n", busy_timeout_us);
            last_diag.error_flag = SPI_ERROR_BUSY_TIMEOUT;
            last_diag.busy_time_us = busy_timeout_us;
            return SPI_ERROR_BUSY_TIMEOUT;
        }
    }
    
    last_diag.busy_time_us = hal_get_time_us() - busy_start;
    
        SX1262_LOG_SPI("    BUSY LOW, total processing %lu us\n", last_diag.busy_time_us);
        SX1262_LOG_SPI("<<< CMD 0x%02X complete (total %lu us)\n", 
                     opcode, hal_get_time_us() - start_total);

    last_diag.total_time_us = hal_get_time_us() - start_total;
    if (diag) *diag = last_diag;
    return SPI_OK;
}

// ============ CONVENIENCE WRAPPERS ============

    // Regulator Mode: 0x01 (DCDC) or 0x00 (LDO)
spi_result_t spi_set_regulator_mode(uint8_t mode) {
    // Opcode 0x96, 1 parameter
    uint8_t params[1] = {mode & 0x01}; 
    return spi_cmd(SX1262_OP_SET_REGULATOR_MODE, params, 1, NULL, 0, 0, NULL);
}

    // delay_us: TCXO stabilization time in microseconds (e.g., 5000 for 5ms)
    spi_result_t spi_set_tcxo_mode(uint32_t delay_us) {
    // DIO3_CTRL: Set to 0x07 for 1.8V (standard TCXO control)
    uint8_t dio3_ctrl = 0x07; 
    
    // Calculate 3-byte TCXO timeout in 15.625 us steps:
    // (delay_us / 15.625) rounded up
    // Using integer math: (delay_us * 1000) / 15625 = delay_us / 15.625
    uint32_t delay_steps = (delay_us * 1000 + 15624) / 15625; 
    
    // Max 3 bytes for timeout (24-bit value)
    if (delay_steps > 0xFFFFFF) {
        delay_steps = 0xFFFFFF; 
    }
    
    uint8_t params[4] = {
        dio3_ctrl,
        (uint8_t)((delay_steps >> 16) & 0xFF),
        (uint8_t)((delay_steps >> 8) & 0xFF),
        (uint8_t)(delay_steps & 0xFF)
    };
    
    // Opcode 0x97, 4 parameters
    return spi_cmd(SX1262_OP_SET_TCXO_MODE, params, 4, NULL, 0, 0, NULL);
}

spi_result_t spi_cmd_simple(uint8_t opcode, uint32_t busy_timeout_us) {
    return spi_cmd(opcode, NULL, 0, NULL, 0, busy_timeout_us, NULL);
}

spi_result_t spi_set_standby(uint8_t mode) {
    uint8_t param = mode;
    return spi_cmd(SX1262_OP_SET_STANDBY, &param, 1, NULL, 0, 0, NULL);
}

spi_result_t spi_write_register(uint16_t addr, const uint8_t *data, uint16_t len) {
    uint8_t params[2 + len];
    params[0] = addr >> 8;
    params[1] = addr & 0xFF;
    memcpy(params + 2, data, len);
    return spi_cmd(SX1262_OP_WRITE_REGISTER, params, 2 + len, NULL, 0, 0, NULL);
}

spi_result_t spi_read_register(uint16_t addr, uint8_t *data, uint16_t len) {
    // SPI Protocol for ReadRegister:
    // TX: [opcode] [addr_hi] [addr_lo] [NOP] [dummy...dummy]
    // RX: [status] [status]   [status]  [status] [data...data]
    // 
    // The chip returns status bytes echoing each transmitted byte,
    // then returns the actual register data
    
    uint8_t params[3] = { 
        (uint8_t)(addr >> 8),     // Address high byte
        (uint8_t)(addr & 0xFF),   // Address low byte
        0x00                       // NOP byte
    };
    
    // We'll receive: 4 status echoes (for opcode + 3 params) + actual data
    uint16_t total_rx = 4 + len;
    uint8_t rx_buf[total_rx];
    memset(rx_buf, 0xFF, sizeof(rx_buf));
    
    // CRITICAL: Pass total_rx as rx_len so spi_cmd doesn't add more bytes
    // and doesn't try to copy data (we'll extract it ourselves)
    spi_result_t res = spi_cmd(SX1262_OP_READ_REGISTER, params, 3, rx_buf, total_rx, 0, NULL);
    
    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_DEBUG("[SPI] spi_read_register full RX buffer: ");
    for (uint16_t i = 0; i < total_rx; i++) {
        SX1262_LOG_DEBUG("0x%02X ", rx_buf[i]);
    }
    SX1262_LOG_SPI("\n");
    #endif
    
    if (res == SPI_OK) {
        // Data starts at index 0 in rx_buf because spi_cmd() doesn't copy anymore
        // when rx_data == rx_buf (same pointer), it leaves data in place
        memcpy(data, rx_buf, len);  // Ã¢â€ Â Changed from rx_buf + 4 to rx_buf
        
        #if DEBUG_VERBOSE_SPI
        SX1262_LOG_DEBUG("[SPI] Extracted %d bytes from index 0: ", len);
        for (uint16_t i = 0; i < len; i++) {
            SX1262_LOG_DEBUG("0x%02X ", data[i]);
        }
        SX1262_LOG_SPI("\n");
        #endif
    }
    
    return res;
}

spi_result_t spi_write_buffer(uint8_t offset, const uint8_t *data, uint8_t len) {
    uint8_t params[1 + len];
    params[0] = offset;
    memcpy(params + 1, data, len);
    return spi_cmd(SX1262_OP_WRITE_BUFFER, params, 1 + len, NULL, 0, 0, NULL);
}

spi_result_t spi_read_buffer(uint8_t offset, uint8_t *data, uint8_t len) {
    // ReadBuffer command (0x1E) - per datasheet Table 13-27:
    //
    // Byte:      0       1       2        3       4        ... n
    // TX:      [0x1E] [offset] [NOP]    [NOP]   [NOP]     ... [NOP]
    // RX:      [RFU]  [Status] [Status] [D0]    [D1]      ... [D(n-3)]
    //
    // Key insight: Data starts at byte 3 (after opcode, offset, and TWO status bytes)
    // The "NOP" at byte 2 is required per datasheet: "An NOP must be sent after 
    // sending the offset" (Section 13.2.4)
    //
    // spi_cmd with tx_len=1 sends: [opcode][offset][NOPs...]
    // spi_cmd copies from position total_tx (=2), but data actually starts at position 3
    // So we need to skip 1 additional byte
    
    // Send offset as parameter, request (len + 1) bytes to account for extra status byte
    uint8_t param = offset;
    uint8_t rx_buf[1 + len];  // Extra byte for the status we need to skip
    
    spi_result_t res = spi_cmd(SX1262_OP_READ_BUFFER, &param, 1, rx_buf, 1 + len, 0, NULL);
    
    if (res == SPI_OK) {
        // Skip the first byte (extra status), data starts at rx_buf[1]
        memcpy(data, rx_buf + 1, len);
    }
    
    return res;
}

spi_result_t spi_get_rx_buffer_status(uint8_t* payload_length, uint8_t* rx_start_offset) {
    // GetRxBufferStatus command: Opcode 0x13
    // CRITICAL: Uses direct HAL SPI transfer to handle the SX1262 "Double Status" quirk.
    //
    // Transaction structure (5 bytes):
    //   Send: [0x13] [NOP] [NOP] [NOP] [NOP]
    //   Recv: [Stat] [Stat] [Len] [Ptr] [Garbage]
    //
    // Datasheet: Section 13.5.2 [cite: 2540]

    if (!payload_length || !rx_start_offset) {
        return SPI_ERROR_BUFFER_TOO_LARGE; // Reusing error code for null pointer
    }

    // 1. Wait for BUSY to go LOW before starting
    uint32_t busy_start = hal_get_time_ms();
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_ms() - busy_start) > TIMING_BUSY_POLL_TIMEOUT_MS) {
            return SPI_ERROR_BUSY_TIMEOUT;
        }
        hal_delay_us(10);
    }

    // 2. Prepare Buffers (Opcode + 4 NOPs)
    uint8_t tx_buf[5] = {SX1262_OP_GET_RX_BUFFER_STATUS, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[5] = {0};

    // 3. Perform Atomic SPI Transaction
    hal_spi_cs_assert(1); // CS LOW
    hal_spi_transfer(tx_buf, rx_buf, 5);
    hal_spi_cs_assert(0); // CS HIGH

    // 4. Wait for BUSY to go LOW after transaction
    busy_start = hal_get_time_ms();
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_ms() - busy_start) > TIMING_BUSY_POLL_TIMEOUT_MS) {
            return SPI_ERROR_BUSY_TIMEOUT;
        }
        hal_delay_us(10);
    }

    // 5. Extract Data (Skipping the two status bytes at index 0 and 1)
    *payload_length = rx_buf[2];
    *rx_start_offset = rx_buf[3];

    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_SPI("GetRxBufferStatus: len=%d, offset=%d (raw: %02X %02X %02X %02X %02X)\n",
                   *payload_length, *rx_start_offset,
                   rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
    #endif

    return SPI_OK;
}

spi_result_t spi_set_buffer_base_address(uint8_t tx_base_addr, uint8_t rx_base_addr) {
    // SetBufferBaseAddress command: Opcode 0x8F
    // Parameters: 2 bytes [TX base address, RX base address]
    // 
    // This sets where in the 256-byte buffer the TX and RX data starts.
    // Typical usage: TX at 0, RX at 128 (splits buffer 50/50)
    
    uint8_t params[2] = {
        tx_base_addr,
        rx_base_addr
    };
    
    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_SPI("spi_set_buffer_base_address: TX=0x%02X, RX=0x%02X\n", 
                   tx_base_addr, rx_base_addr);
    #endif
    
    return spi_cmd(SX1262_OP_SET_BUFFER_BASE_ADDR, params, 2, NULL, 0, 0, NULL);
}

spi_result_t spi_set_dio_irq_params(
    uint16_t irq_mask,
    uint16_t dio1_mask,
    uint16_t dio2_mask,
    uint16_t dio3_mask
) {
    // SetDioIrqParams command: Opcode 0x08
    // Parameters: 8 bytes (4x 16-bit values in MSB-first order)
    //   - irq_mask:  Which interrupts are enabled globally
    //   - dio1_mask: Which interrupts route to DIO1 pin
    //   - dio2_mask: Which interrupts route to DIO2 pin
    //   - dio3_mask: Which interrupts route to DIO3 pin
    //
    // Datasheet: Table 11-49, page 82
    
    uint8_t params[8] = {
        (uint8_t)((irq_mask >> 8) & 0xFF),    // IRQ mask MSB
        (uint8_t)(irq_mask & 0xFF),           // IRQ mask LSB
        (uint8_t)((dio1_mask >> 8) & 0xFF),   // DIO1 mask MSB
        (uint8_t)(dio1_mask & 0xFF),          // DIO1 mask LSB
        (uint8_t)((dio2_mask >> 8) & 0xFF),   // DIO2 mask MSB
        (uint8_t)(dio2_mask & 0xFF),          // DIO2 mask LSB
        (uint8_t)((dio3_mask >> 8) & 0xFF),   // DIO3 mask MSB
        (uint8_t)(dio3_mask & 0xFF)           // DIO3 mask LSB
    };
    
    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_SPI("spi_set_dio_irq_params: IRQ=0x%04X, DIO1=0x%04X, DIO2=0x%04X, DIO3=0x%04X\n",
                   irq_mask, dio1_mask, dio2_mask, dio3_mask);
    #endif
    
    return spi_cmd(SX1262_OP_SET_DIO_IRQ_PARAMS, params, 8, NULL, 0, 0, NULL);
}

spi_result_t spi_set_tx(uint32_t timeout_us) {
    // SetTx command: Opcode 0x83
    // Parameters: 3 bytes (24-bit timeout value)
    //
    // The timeout is specified in steps of 15.625 microseconds.
    // Conversion: timeout_steps = timeout_us / 15.625
    //            = (timeout_us * 1000) / 15625
    //            = (timeout_us * 64) / 1000  (simplified integer math)
    //
    // Special values:
    //   0x000000 = No timeout (TX completes when packet sent)
    //   0xFFFFFF = Maximum timeout (~262 seconds)
    //
    // Datasheet: Section 13.5.1, page 95
    
    uint32_t timeout_steps;
    
    if (timeout_us == 0) {
        // No timeout - radio stays in TX until packet sent
        timeout_steps = 0x000000;
    } else {
        // Convert microseconds to 15.625Ã‚Âµs steps using integer math
        // Formula: steps = (timeout_us * 64 + 999) / 1000
        // The +999 ensures rounding up
        timeout_steps = ((uint64_t)timeout_us * 64 + 999) / 1000;
        
        // Clamp to 24-bit maximum
        if (timeout_steps > 0xFFFFFF) {
            timeout_steps = 0xFFFFFF;
        }
    }
    
    uint8_t cmd[4] = {
        SX1262_OP_SET_TX,                          // Opcode 0x83
        (uint8_t)((timeout_steps >> 16) & 0xFF),   // Timeout MSB
        (uint8_t)((timeout_steps >> 8) & 0xFF),    // Timeout MID
        (uint8_t)(timeout_steps & 0xFF)            // Timeout LSB
    };
    
    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_SPI("spi_set_tx: timeout_us=%lu, timeout_steps=0x%06lX\n",
                   timeout_us, timeout_steps);
    #endif
    
    // =========================================================================
    // CRITICAL FIX (v2): Unlike other commands, SetTx causes BUSY to stay HIGH
    // for the ENTIRE duration of the transmission (Time-on-Air).
    // The standard spi_cmd() waits for BUSY LOW after every command, which
    // would block for ~ToA milliseconds. This is redundant because the caller
    // (sx1262_transmit) uses ISR-based notification to detect TX_DONE.
    //
    // Solution: Send SetTx manually without post-command BUSY wait.
    // =========================================================================
    
    // Step 1: Wait for initial BUSY LOW (device must be ready)
    uint32_t busy_start = hal_get_time_us();
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_us() - busy_start) > default_busy_timeout_us) {
            SX1262_LOG_ERROR("[SPI] spi_set_tx: BUSY timeout before SetTx command\n");
            return SPI_ERROR_BUSY_TIMEOUT;
        }
    }
    
    // Step 2: Send SetTx command via SPI
    hal_spi_cs_assert(1);  // CS LOW - start transaction
    hal_result_t res = hal_spi_transfer(cmd, NULL, 4);
    hal_spi_cs_assert(0);  // CS HIGH - end transaction
    
    if (res != HAL_OK) {
        SX1262_LOG_ERROR("[SPI] spi_set_tx: SPI transfer failed\n");
        return SPI_ERROR_SPI_WRITE;
    }
    
    // Step 3: Return immediately - DO NOT wait for BUSY LOW
    // BUSY will go HIGH and stay HIGH for entire TX duration.
    // Caller uses DIO1 interrupt to detect TX_DONE.
    
    return SPI_OK;
}

spi_result_t spi_get_device_id(uint8_t* device_id) {
    // Get Device ID by reading ASCII string from register 0x0320
    // 
    // The SX1262 stores a device identification string at address 0x0320:
    //   "SX1262 V2D 2D02\0" (or similar revision)
    //
    // This is undocumented in the datasheet but confirmed in:
    // - Semtech forums (thread #1508)
    // - RadioLib implementation
    // - Multiple production drivers
    //
    // We read 16 bytes to capture the full string, then parse it
    // to identify chip type (SX1261, SX1262, or SX1268).
    //
    // Expected strings:
    //   SX1261: "SX1261 V2D 2D02\0"
    //   SX1262: "SX1262 V2D 2D02\0"
    //   SX1268: "SX1268 V2D 2D02\0"
    
    if (!device_id) {
        return SPI_ERROR_BUFFER_TOO_LARGE;  // Reusing error code for NULL pointer
    }
    
    // Read 16 bytes from register 0x0320 to get full ASCII string
    uint8_t id_string[16];
    memset(id_string, 0, sizeof(id_string));
    
    spi_result_t res = spi_read_register(0x031C, id_string, 16);
    
    if (res == SPI_OK) {
        // Copy first 2 bytes to device_id (for backward compatibility)
        device_id[0] = id_string[0];
        device_id[1] = id_string[1];
        
        #if DEBUG_VERBOSE_SPI
        // Print full string (ensure null termination for safety)
        char safe_string[17];
        memcpy(safe_string, id_string, 16);
        safe_string[16] = '\0';
        SX1262_LOG_SPI("spi_get_device_id: \"%s\"\n", safe_string);
        
        // Parse chip type from string
        if (memcmp(id_string, "SX1261", 6) == 0) {
            SX1262_LOG_SPI("  -> Chip Type: SX1261 (Low Power)\n");
        } else if (memcmp(id_string, "SX1262", 6) == 0) {
            SX1262_LOG_SPI("  -> Chip Type: SX1262 (High Power) Ã¢Å“â€œ\n");
        } else if (memcmp(id_string, "SX1268", 6) == 0) {
            SX1262_LOG_SPI("  -> Chip Type: SX1268 (High Power)\n");
        } else {
            SX1262_LOG_SPI("  -> Unknown chip or corrupted ID\n");
        }
        
        // Extract revision info (characters 7-15)
        if (id_string[7] != 0) {
            SX1262_LOG_SPI("  -> Revision: ");
            for (int i = 7; i < 16 && id_string[i] != 0; i++) {
                if (id_string[i] >= 0x20 && id_string[i] <= 0x7E) {  // Printable ASCII
                    SX1262_LOG_SPI("%c", id_string[i]);
                }
            }
            SX1262_LOG_SPI("\n");
        }
        #endif
    }
    
    return res;
}

spi_result_t spi_get_packet_status(
    int16_t* rssi_pkt,
    int8_t* snr_pkt,
    int16_t* signal_rssi_pkt
) {
    // GetPacketStatus command: Opcode 0x14
    // Returns packet quality metrics (RSSI, SNR, Signal RSSI)
    //
    // Response format (LoRa mode):
    //   Byte 0: Status (echo)
    //   Byte 1: RssiPkt (packet RSSI, raw value)
    //   Byte 2: SnrPkt (packet SNR, raw value - signed)
    //   Byte 3: SignalRssiPkt (signal RSSI, raw value)
    //
    // Conversion formulas (from datasheet Section 5.8.3.15):
    //   RSSI = -RssiPkt / 2 (dBm)
    //   SNR = SnrPkt / 4 (dB)
    //   Signal RSSI = -SignalRssiPkt / 2 (dBm)
    //
    // Datasheet: Section 13.6.2, page 98
    
    if (!rssi_pkt || !snr_pkt || !signal_rssi_pkt) {
        return SPI_ERROR_BUFFER_TOO_LARGE;  // Reusing for NULL pointer check
    }
    
    // GetPacketStatus returns 3 data bytes after status echo
    // Total RX: 1 status + 3 data = 4 bytes
    uint8_t rx_buf[4];
    memset(rx_buf, 0, sizeof(rx_buf));
    
    spi_result_t res = spi_cmd(SX1262_OP_GET_PACKET_STATUS, NULL, 0, rx_buf, 4, 0, NULL);
    
    if (res == SPI_OK) {
        // Extract raw values from response
        // Data starts at index 1 (after status echo)
        uint8_t rssi_raw = rx_buf[1];
        int8_t snr_raw = (int8_t)rx_buf[2];  // SNR is signed
        uint8_t signal_rssi_raw = rx_buf[3];
        
        // Convert to actual values (raw values stored for caller to convert if needed)
        // Note: We return raw values, caller can apply conversion formulas
        *rssi_pkt = -(int16_t)rssi_raw;  // Negative value (will be / 2 for dBm)
        *snr_pkt = snr_raw;              // Signed value (will be / 4 for dB)
        *signal_rssi_pkt = -(int16_t)signal_rssi_raw;  // Negative value
        
        #if DEBUG_VERBOSE_SPI
        SX1262_LOG_SPI("spi_get_packet_status: RSSI=%d (%.1f dBm), SNR=%d (%.2f dB), SignalRSSI=%d (%.1f dBm)\n",
                       *rssi_pkt, (float)*rssi_pkt / 2.0f,
                       *snr_pkt, (float)*snr_pkt / 4.0f,
                       *signal_rssi_pkt, (float)*signal_rssi_pkt / 2.0f);
        #endif
    }
    
    return res;
}

spi_result_t spi_set_rx(uint32_t timeout_us) {
    // SetRx command: Opcode 0x82
    // Parameters: 3 bytes (24-bit timeout value)
    //
    // Similar to SetTx, but enters receive mode.
    // The timeout is specified in steps of 15.625 microseconds.
    //
    // Conversion: timeout_steps = timeout_us / 15.625
    //            = (timeout_us * 1000) / 15625
    //            = (timeout_us * 64) / 1000  (simplified integer math)
    //
    // Special values:
    //   0x000000 = No timeout (continuous RX, stays in RX until stopped)
    //   0xFFFFFF = Maximum timeout (~262 seconds)
    //
    // Datasheet: Section 13.5.2, page 96
    
    uint32_t timeout_steps;
    
    if (timeout_us == 0) {
        // No timeout - continuous RX mode
        timeout_steps = 0x000000;
    } else {
        // Convert microseconds to 15.625Ã‚Âµs steps using integer math
        // Formula: steps = (timeout_us * 64 + 999) / 1000
        // The +999 ensures rounding up
        timeout_steps = ((uint64_t)timeout_us * 64 + 999) / 1000;
        
        // Clamp to 24-bit maximum
        if (timeout_steps > 0xFFFFFF) {
            timeout_steps = 0xFFFFFF;
        }
    }
    
    uint8_t params[3] = {
        (uint8_t)((timeout_steps >> 16) & 0xFF),  // Timeout MSB
        (uint8_t)((timeout_steps >> 8) & 0xFF),   // Timeout MID
        (uint8_t)(timeout_steps & 0xFF)           // Timeout LSB
    };
    
    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_SPI("spi_set_rx: timeout_us=%lu, timeout_steps=0x%06lX\n",
                   timeout_us, timeout_steps);
    #endif
    
    return spi_cmd(SX1262_OP_SET_RX, params, 3, NULL, 0, 0, NULL);
}

spi_result_t spi_clear_device_errors() {
    // ClearDeviceErrors command: Opcode 0x07
    // Parameters: 2 bytes [0x00, 0x00]
    //
    // The SX1262 maintains internal error flags that can prevent
    // proper operation if not cleared. These include:
    // - RC64k calibration error
    // - RC13M calibration error
    // - PLL calibration error
    // - ADC calibration error
    // - Image calibration error
    // - XOSC start error
    // - PLL lock error
    // - PA ramp error
    //
    // This command resets all these error flags.
    // Should be called:
    // - After initialization
    // - After any error condition
    // - Before critical operations (TX/RX)
    //
    // Datasheet: Section 13.3.2, page 93
    
    uint8_t params[2] = {0x00, 0x00};
    
    #if DEBUG_VERBOSE_SPI
    SX1262_LOG_SPI("spi_clear_device_errors: Clearing all device error flags\n");
    #endif
    
    return spi_cmd(SX1262_OP_CLEAR_DEVICE_ERRORS, params, 2, NULL, 0, 0, NULL);
}

spi_result_t spi_get_status(uint8_t *status) {
    uint8_t rx_buf[2];
    memset(rx_buf, 0xAA, sizeof(rx_buf));
    
    SX1262_LOG_DEBUG("[SPI] spi_get_status: BEFORE spi_cmd, rx_buf = [0x%02X, 0x%02X]\n", 
                     rx_buf[0], rx_buf[1]);
    
    spi_result_t res = spi_cmd(SX1262_OP_GET_STATUS, NULL, 0, rx_buf, 2, 0, NULL);
    
    SX1262_LOG_DEBUG("[SPI] spi_get_status: AFTER spi_cmd, res=%d, rx_buf = [0x%02X, 0x%02X]\n", 
                     res, rx_buf[0], rx_buf[1]);
    
    if (res == SPI_OK) {
        // Get status from last_diag, not from rx_buf (which got overwritten)
        *status = rx_buf[0];
        SX1262_LOG_DEBUG("[SPI] spi_get_status: Returning status = 0x%02X (from last_diag)\n", *status);
    } else {
        SX1262_LOG_DEBUG("[SPI] spi_get_status: ERROR - spi_cmd returned %d\n", res);
    }
    
    return res;
}

spi_result_t spi_get_irq_status(uint16_t *irq_status) {
    // GetIrqStatus returns 2 bytes
    // RX will be: [status] [irq_hi] [irq_lo]
    uint8_t rx_buf[3];
    spi_result_t res = spi_cmd(SX1262_OP_GET_IRQ_STATUS, NULL, 0, rx_buf, 3, 0, NULL);
    
    if (res == SPI_OK) {
        // Data starts at index 1 (after status echo)
        *irq_status = (rx_buf[1] << 8) | rx_buf[2];
    }
    
    return res;
}

spi_result_t spi_clear_irq_status(uint16_t irq_mask) {
    uint8_t params[2] = { irq_mask >> 8, irq_mask & 0xFF };
    return spi_cmd(SX1262_OP_CLEAR_IRQ_STATUS, params, 2, NULL, 0, 0, NULL);
}

uint32_t spi_poll_busy_timing(uint32_t timeout_us) {
    uint32_t start = hal_get_time_us();
    while (hal_gpio_read(PIN_SX1262_BUSY)) {
        if ((hal_get_time_us() - start) >= timeout_us) return timeout_us;
    }
    return hal_get_time_us() - start;
}

uint8_t spi_is_busy() {
    return hal_gpio_read(PIN_SX1262_BUSY);
}

spi_command_diag_t* spi_get_last_diag() {
    return &last_diag;
}

void spi_print_last_diag() {
    SX1262_LOG_SPI("\n[SPI Diag] Opcode:0x%02X Status:0x%02X (Cmd:0x%X IRQ:0x%X) Busy:%luus Total:%luus%s\n",
        last_diag.opcode,
        last_diag.status_byte,
        last_diag.status_byte & 0x0F,
        last_diag.irq_bits,
        last_diag.busy_time_us,
        last_diag.total_time_us,
        last_diag.error_flag ? " ERROR" : "");
}