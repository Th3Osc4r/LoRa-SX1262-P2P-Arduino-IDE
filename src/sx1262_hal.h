/**
 * sx1262_hal.h - Hardware Abstraction Layer Interface for SX1262
 * 
 * VERSION 2 CHANGES:
 * - Added hal_board_power_init() and hal_board_power_deinit() for VEXT control
 * - Used with sx1262_hal_v2.cpp which reads all pins from config.h
 */

#ifndef SX1262_HAL_H
#define SX1262_HAL_H

#include <stdint.h>

// ============ RETURN CODES ============
typedef enum {
    HAL_OK = 0,
    HAL_ERROR_SPI = -1,
    HAL_ERROR_GPIO = -2,
    HAL_ERROR_TIMEOUT = -3,
} hal_result_t;

// ============ GPIO OPERATIONS ============
/**
 * Initialize GPIO pin as input
 * @param pin: GPIO pin number
 * @param pull_mode: 0=no pull, 1=pull-up, 2=pull-down
 * @return HAL_OK or error code
 */
hal_result_t hal_gpio_init_input(uint8_t pin, int pull_mode);

// CRITICAL FIX: Added this prototype so the driver can see it
hal_result_t hal_gpio_attach_interrupt(uint8_t pin, void (*callback)(void*), void* arg);

/**
 * Initialize GPIO pin as output (push-pull)
 * @param pin: GPIO pin number
 * @return HAL_OK or error code
 */
hal_result_t hal_gpio_init_output(uint8_t pin);

/**
 * Read GPIO pin state
 * @param pin: GPIO pin number
 * @return 0 (LOW) or 1 (HIGH)
 */
uint8_t hal_gpio_read(uint8_t pin);

/**
 * Write GPIO pin state
 * @param pin: GPIO pin number
 * @param value: 0=LOW, 1=HIGH
 */
void hal_gpio_write(uint8_t pin, uint8_t value);

// DIO operations
hal_result_t hal_dio_init(void);
void hal_dio_deinit(void);

// ============ BOARD POWER CONTROL (Optional) ============
/**
 * Initialize and enable power to external peripherals
 * For boards with VEXT pin (e.g., Heltec V3), this enables power to the SX1262.
 * For boards without VEXT, this is a no-op stub.
 * Call BEFORE sx1262_driver_init().
 * @return HAL_OK or error code
 */
hal_result_t hal_board_power_init(void);

/**
 * Disable power to external peripherals
 */
void hal_board_power_deinit(void);

// Reset operations
hal_result_t hal_reset_init(void);
void hal_reset_deinit(void);  // â† ADD THIS LINE if it's missing!
void hal_reset_pin_state(uint8_t assert);
void hal_reset_pulse(void);
hal_result_t hal_reset(void);

// ============ SPI OPERATIONS ============
/**
 * Initialize SPI hardware and pins
 * @param spi_freq_hz: SPI clock frequency
 * @param cs_pin: Chip Select pin (will be controlled manually)
 * @return HAL_OK or error code
 */
hal_result_t hal_spi_init(uint32_t spi_freq_hz, uint8_t cs_pin);

/**
 * Deinitialize SPI hardware
 */
void hal_spi_deinit();

/**
 * Perform an SPI transfer (blocking)
 * @param tx_data: Pointer to transmit buffer
 * @param rx_data: Pointer to receive buffer
 * @param len: number of bytes
 * @return HAL_OK or error code
 */
hal_result_t hal_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

hal_result_t hal_spi_read(uint8_t *rx_data, uint16_t len);

/**
 * Assert/deassert chip select manually
 * Used for multi-part SPI transactions where NSS must stay low
 * @param assert: 1=NSS low (asserted), 0=NSS high (deasserted)
 */
void hal_spi_cs_assert(uint8_t assert);

// ============ TIMING OPERATIONS ============
/**
 * Initialize timing system (call once at startup)
 * Must be called before any timing functions
 */
void hal_timing_init();

/**
 * Get current time in microseconds (32-bit, wraps)
 * @return current time in Î¼s
 */
uint32_t hal_get_time_us();

/**
 * Get current time in milliseconds (32-bit, wraps)
 * @return current time in ms
 */
uint32_t hal_get_time_ms();

/**
 * Blocking delay in milliseconds
 * @param ms: milliseconds to delay
 */
void hal_delay_ms(uint32_t ms);

/**
 * Blocking delay in microseconds (busy-wait)
 * Avoid for long delays; use for precise short delays only
 * @param us: microseconds to delay
 */
void hal_delay_us(uint32_t us);

// ============ RESET OPERATIONS ============
/**
 * Initialize NRESET pin and perform the hardware reset sequence
 * @return HAL_OK or error code
 */
hal_result_t hal_reset_init();

/**
 * Perform a hard reset sequence (toggle NRESET pin)
 * This is the function the compiler complained was missing!
 * @return HAL_OK or error code
 */
hal_result_t hal_reset();

void hal_reset_pulse();

#endif // SX1262_HAL_H