#ifndef SX1262_SPI_PROTOCOL_H
#define SX1262_SPI_PROTOCOL_H

#include <stdint.h>
#include "sx1262_regs.h"

#define SPI_CMD_MAX_BUFFER_SIZE 32
#define SX1262_OP_SET_REGULATOR_MODE    0x96
#define SX1262_OP_SET_TCXO_MODE         0x97

// ============ RETURN CODES ============
typedef enum {
    SPI_OK = 0,
    SPI_ERROR_BUSY_TIMEOUT = -1,
    SPI_ERROR_SPI_WRITE = -2,
    SPI_ERROR_SPI_READ = -3,
    SPI_ERROR_HAL = -4,
    SPI_ERROR_BUFFER_TOO_LARGE = -5,
} spi_result_t;

// ============ DIAGNOSTICS STRUCTURE ============
/**
 * Captured diagnostics for a single SPI command
 * Stores timing, status, and error information for post-analysis
 */
typedef struct {
    uint32_t opcode;              // Command opcode sent (8-bit, stored as uint32)
    uint8_t  status_byte;         // Status byte returned on MISO (lower 4 bits = command status)
    uint8_t  irq_bits;            // IRQ bits from status byte (upper 4 bits)
    uint32_t busy_time_us;        // Duration BUSY was high (microseconds)
    uint32_t total_time_us;       // Total command execution time including BUSY poll
    uint32_t busy_timeout_used_us; // Timeout value used for BUSY poll
    uint8_t  error_flag;          // Error occurred (0=no error, non-zero=error code)
    char     description[80];     // Human-readable command description
} spi_command_diag_t;

// ============ INITIALIZATION ============
/**
 * Initialize SPI protocol layer
 * Must be called after HAL is initialized
 * @param spi_freq_hz: SPI clock frequency in Hz (typically 10000000 for 10 MHz)
 * @return SPI_OK or error code
 */
spi_result_t spi_init(uint32_t spi_freq_hz);

/**
 * Deinitialize SPI protocol layer
 * Cleanup and resource release
 */
void spi_deinit();

// ============ CORE SPI COMMAND (with BUSY polling) ============
/**
 * Send a generic SPI command with automatic BUSY polling
 * 
 * This is the core function for all SPI communication.
 * Sequence:
 *   1. If BUSY already high, wait for it to clear (from previous command)
 *   2. Send: [opcode] [tx_params...]
 *   3. Capture status byte from MISO
 *   4. Poll BUSY pin until it goes low (with timeout)
 *   5. If rx_len > 0, read response data
 *   6. Fill diagnostics struct with measurements
 *
 * @param opcode: Command opcode (8-bit, defines command to SX1262)
 * @param tx_params: Pointer to parameter bytes (can be NULL if no params)
 * @param tx_len: Number of parameter bytes to send (0 if no params)
 * @param rx_data: Buffer to store response data (can be NULL if no response)
 * @param rx_len: Number of response bytes expected (0 if no response)
 * @param busy_timeout_us: Timeout for BUSY poll in microseconds
 *                         (0 = use default from config)
 * @param diag: Optional pointer to diagnostic struct to fill
 *              (can be NULL if diagnostics not needed)
 * 
 * @return SPI_OK on success, or error code on failure
 */
spi_result_t spi_cmd(
    uint8_t opcode,
    const uint8_t *tx_params,
    uint16_t tx_len,
    uint8_t *rx_data,
    uint16_t rx_len,
    uint32_t busy_timeout_us,
    spi_command_diag_t *diag
);

// ============ CONVENIENCE WRAPPERS ============
/**
 * Send command with no parameters or response
 * Useful for simple mode-change commands (e.g., SetFs, SetCAD)
 * @param opcode: Command opcode
 * @param busy_timeout_us: BUSY poll timeout (0=default)
 * @return SPI_OK or error code
 */
spi_result_t spi_cmd_simple(uint8_t opcode, uint32_t busy_timeout_us);

/**
 * Set standby mode
 * @param mode: STDBY_RC (0x00) or STDBY_XOSC (0x01)
 * @return SPI_OK or error code
 */
spi_result_t spi_set_standby(uint8_t mode);

/**
 * Read register (16-bit address, auto-increment on multi-byte read)
 * @param addr: 16-bit register address
 * @param data: Buffer to store read data
 * @param len: Number of bytes to read
 * @return SPI_OK or error code
 */
spi_result_t spi_read_register(uint16_t addr, uint8_t *data, uint16_t len);

/**
 * Write register (16-bit address, auto-increment on multi-byte write)
 * @param addr: 16-bit register address
 * @param data: Data bytes to write
 * @param len: Number of bytes to write
 * @return SPI_OK or error code
 */
spi_result_t spi_write_register(uint16_t addr, const uint8_t *data, uint16_t len);

/**
 * Write TX buffer
 * Opcode: 0x0E
 * @param offset: Offset in buffer (0-255)
 * @param data: Payload to write
 * @param len: Payload length
 * @return SPI_OK or error code
 */
spi_result_t spi_write_buffer(uint8_t offset, const uint8_t *data, uint8_t len);

/**
 * Read RX buffer
 * Opcode: 0x1E
 * @param offset: Offset in buffer (0-255)
 * @param data: Buffer to store received data
 * @param len: Number of bytes to read
 * @return SPI_OK or error code
 */
spi_result_t spi_read_buffer(uint8_t offset, uint8_t *data, uint8_t len);

/**
 * Set buffer base addresses for TX and RX
 * The SX1262 has 256 bytes of buffer memory that can be split between TX and RX.
 * Typical usage: TX at offset 0, RX at offset 128 (50/50 split)
 * 
 * Opcode: 0x8F
 * Parameters: 2 bytes [tx_base_addr, rx_base_addr]
 * 
 * @param tx_base_addr Starting address for TX buffer (0-255)
 * @param rx_base_addr Starting address for RX buffer (0-255)
 * @return SPI_OK or error code
 */
spi_result_t spi_set_buffer_base_address(uint8_t tx_base_addr, uint8_t rx_base_addr);

/**
 * Configure DIO interrupt routing and IRQ masks
 * 
 * This configures which interrupts are enabled globally and which ones
 * are routed to which DIO pins. For TX, we typically enable TX_DONE and
 * route it to DIO1.
 * 
 * Opcode: 0x08
 * Parameters: 8 bytes (4x 16-bit masks in MSB-first order)
 * 
 * Datasheet Reference: Table 11-49, page 82
 * 
 * @param irq_mask Global IRQ enable mask (16-bit, which IRQs are active)
 * @param dio1_mask IRQs routed to DIO1 pin (16-bit)
 * @param dio2_mask IRQs routed to DIO2 pin (16-bit)
 * @param dio3_mask IRQs routed to DIO3 pin (16-bit)
 * @return SPI_OK or error code
 * 
 * Example for TX:
 *   spi_set_dio_irq_params(
 *       IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,  // Enable these IRQs
 *       IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,  // Route to DIO1
 *       0x0000,                            // DIO2 not used
 *       0x0000                             // DIO3 not used
 *   );
 */
spi_result_t spi_set_dio_irq_params(
    uint16_t irq_mask,
    uint16_t dio1_mask,
    uint16_t dio2_mask,
    uint16_t dio3_mask
);

/**
 * Enter TX mode with optional timeout
 * 
 * The radio will transmit the packet from the TX buffer and then either:
 * - Return to STANDBY_RC (if no timeout specified)
 * - Trigger RX_TX_TIMEOUT IRQ (if timeout expires)
 * 
 * Opcode: 0x83
 * Parameters: 3 bytes (24-bit timeout value)
 * 
 * Timeout is specified in steps of 15.625 microseconds (64 µs resolution):
 * - 0x000000 = No timeout (infinite)
 * - 0xFFFFFF = Maximum timeout (~262 seconds)
 * 
 * Datasheet Reference: Section 13.5.1, page 95
 * 
 * @param timeout_us Timeout in microseconds (0 = no timeout)
 * @return SPI_OK or error code
 * 
 * Note: The actual timeout resolution is 15.625µs, so timeout_us
 *       will be rounded to the nearest multiple of this value.
 */
spi_result_t spi_set_tx(uint32_t timeout_us);

/**
 * Get device ID to verify chip type
 * 
 * Returns a 2-byte device identifier:
 * - SX1261: 0x01, 0x24
 * - SX1262: 0x01, 0x33
 * - SX1268: 0x01, 0x68
 * 
 * This is useful for verifying correct chip communication and
 * detecting counterfeit or wrong chips.
 * 
 * Opcode: 0x1D (Read register at address 0x0320)
 * 
 * @param device_id Pointer to 2-byte array to store device ID
 * @return SPI_OK or error code
 */
spi_result_t spi_get_device_id(uint8_t* device_id);

/**
 * Get packet status (RSSI, SNR, Signal RSSI)
 * 
 * Returns reception quality metrics for the last received packet.
 * Can also be called after TX to verify transmission power.
 * 
 * Opcode: 0x14
 * Returns: 3 bytes of packet status
 * 
 * Datasheet Reference: Section 13.6.2, page 98
 * 
 * @param rssi_pkt Pointer to store packet RSSI (in 0.5 dBm steps, add offset)
 * @param snr_pkt Pointer to store packet SNR (in 0.25 dB steps)
 * @param signal_rssi_pkt Pointer to store signal RSSI (in 0.5 dBm steps)
 * @return SPI_OK or error code
 * 
 * Note: RSSI/SNR values are raw from chip. Use helper functions to convert.
 */
spi_result_t spi_get_packet_status(
    int16_t* rssi_pkt,
    int8_t* snr_pkt,
    int16_t* signal_rssi_pkt
);

/**
 * Get RX buffer status (for future RX operations)
 * 
 * Returns information about received data in the RX buffer:
 * - Payload length received
 * - Starting offset in RX buffer
 * 
 * Opcode: 0x13
 * Returns: 2 bytes [payload_length, rx_start_buffer_pointer]
 * 
 * Datasheet Reference: Section 13.6.1, page 98
 * 
 * @param payload_length Pointer to store received payload length
 * @param rx_start_offset Pointer to store RX buffer start offset
 * @return SPI_OK or error code
 */
spi_result_t spi_get_rx_buffer_status(
    uint8_t* payload_length,
    uint8_t* rx_start_offset
);

/**
 * Set RX mode with timeout (for future RX operations)
 * 
 * Similar to SetTx, but enters receive mode.
 * 
 * Opcode: 0x82
 * Parameters: 3 bytes (24-bit timeout value)
 * 
 * Timeout in steps of 15.625 microseconds:
 * - 0x000000 = No timeout (continuous RX)
 * - 0xFFFFFF = Maximum timeout (~262 seconds)
 * 
 * Datasheet Reference: Section 13.5.2, page 96
 * 
 * @param timeout_us Timeout in microseconds (0 = continuous RX)
 * @return SPI_OK or error code
 */
spi_result_t spi_set_rx(uint32_t timeout_us);

/**
 * Clear device errors (reset error flags)
 * 
 * The SX1262 maintains internal error flags that can prevent
 * operation if not cleared. This command resets them.
 * 
 * Opcode: 0x07
 * Parameters: 2 bytes [0x00, 0x00]
 * 
 * Should be called after errors or on initialization.
 * 
 * @return SPI_OK or error code
 */
spi_result_t spi_clear_device_errors();

/**
 * Get chip status (returns 2 status bytes)
 * Opcode: 0xC0
 * Status byte format:
 *   [7:4] = IRQ bits (if interrupt condition exists)
 *   [3:0] = Command status (0x0F=OK, 0x0E=timeout)
 * 
 * @param status: Pointer to 2-byte array to store [byte0, byte1]
 * @return SPI_OK or error code
 */
spi_result_t spi_get_status(uint8_t *status);

/**
 * Get IRQ status (returns 2-byte 16-bit IRQ flags)
 * Opcode: 0x12
 * @param irq_status: Pointer to store 16-bit IRQ status
 * @return SPI_OK or error code
 */
spi_result_t spi_get_irq_status(uint16_t *irq_status);

/**
 * Clear IRQ status (clears selected IRQ flags)
 * Opcode: 0x02
 * @param irq_mask: 16-bit mask of IRQ flags to clear
 * @return SPI_OK or error code
 */
spi_result_t spi_clear_irq_status(uint16_t irq_mask);

// ============ ADVANCED POLLING & DIAGNOSTICS ============
/**
 * Poll BUSY pin with timing measurement (for low-level testing)
 * Manually measure how long BUSY stays high
 * @param timeout_us: Maximum time to wait (microseconds)
 * @return Time BUSY was high (microseconds), or timeout_us if timeout occurred
 */
uint32_t spi_poll_busy_timing(uint32_t timeout_us);

/**
 * Get last command diagnostics
 * Returns the diagnostics struct from the most recent spi_cmd call
 * @return Pointer to last diagnostic data (valid until next spi_cmd call)
 */
spi_command_diag_t* spi_get_last_diag();

/**
 * Print last command diagnostics to Serial
 * Formatted human-readable output for debugging
 */
void spi_print_last_diag();

/**
 * Check if BUSY pin is currently high
 * For polling without waiting
 * @return 1 if BUSY high, 0 if BUSY low
 */
uint8_t spi_is_busy();

#endif // SX1262_SPI_PROTOCOL_H