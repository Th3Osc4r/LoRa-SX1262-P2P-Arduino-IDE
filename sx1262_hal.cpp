#include "sx1262_hal.h"
#include "config.h"

#include <Arduino.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ============ STATIC VARIABLES ============
static spi_device_handle_t spi_device = NULL;
static uint8_t hal_cs_pin = 255;
static uint32_t hal_spi_freq_hz = 0;
static bool isr_service_installed = false;

// ============ GPIO OPERATIONS ============

hal_result_t hal_gpio_init_input(uint8_t pin, int pull_mode) {
    if (pin > 48) return HAL_ERROR_GPIO;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (pull_mode == 1) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (pull_mode == 2) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return (gpio_config(&io_conf) == ESP_OK) ? HAL_OK : HAL_ERROR_GPIO;
}

hal_result_t hal_gpio_init_output(uint8_t pin) {
    if (pin > 48) return HAL_ERROR_GPIO;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return (gpio_config(&io_conf) == ESP_OK) ? HAL_OK : HAL_ERROR_GPIO;
}

// CRITICAL FIX: Implementation of interrupt attachment
hal_result_t hal_gpio_attach_interrupt(uint8_t pin, void (*callback)(void*), void* arg) {
    if (pin > 48) return HAL_ERROR_GPIO;

    if (gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_POSEDGE) != ESP_OK) return HAL_ERROR_GPIO;

    if (!isr_service_installed) {
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return HAL_ERROR_GPIO;
        isr_service_installed = true;
    }

    return (gpio_isr_handler_add((gpio_num_t)pin, callback, arg) == ESP_OK) ? HAL_OK : HAL_ERROR_GPIO;
}

uint8_t hal_gpio_read(uint8_t pin) { return (uint8_t)gpio_get_level((gpio_num_t)pin); }
void hal_gpio_write(uint8_t pin, uint8_t value) { gpio_set_level((gpio_num_t)pin, value); }

// ============ DIO PIN OPERATIONS ============
hal_result_t hal_dio_init() {
    return hal_gpio_init_input(PIN_SX1262_DIO1, 0);
}

void hal_dio_deinit() {
    gpio_reset_pin((gpio_num_t)PIN_SX1262_DIO1);
    if (isr_service_installed) {
        gpio_isr_handler_remove((gpio_num_t)PIN_SX1262_DIO1);
    }
}

// ============ SPI OPERATIONS ============

hal_result_t hal_spi_init(uint32_t freq_hz, uint8_t cs_pin) {
    hal_cs_pin = cs_pin;
    hal_spi_freq_hz = freq_hz;

    // Default pins for SPI2 on ESP32-S3: MISO (GPIO 13), MOSI (GPIO 11), SCLK (GPIO 12)
    // Corrected logic based on typical S3 config; ensure these match your board.
    spi_bus_config_t buscfg = {
        .mosi_io_num = 11,
        .miso_io_num = 13,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    // Initialize the SPI Bus
    esp_err_t err = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return HAL_ERROR_SPI;
    }

    // Configure the SPI Device (SX1262)
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0, // SX1262 uses SPI Mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = (int)freq_hz,
        .spics_io_num = -1, // Use -1 for manual CS control
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    // Attach the device to the SPI bus
    err = spi_bus_add_device(SPI_HOST, &devcfg, &spi_device);
    if (err != ESP_OK) {
        return HAL_ERROR_SPI;
    }

    // Configure the manual CS pin
    hal_result_t res = hal_gpio_init_output(cs_pin);
    if (res != HAL_OK) {
        return res;
    }
    hal_gpio_write(cs_pin, 1); // NSS high (deasserted) initially

    return HAL_OK;
}

void hal_spi_deinit() {
    if (spi_device != NULL) {
        spi_bus_remove_device(spi_device);
        spi_device = NULL;
    }
    spi_bus_free(SPI_HOST);
}

hal_result_t hal_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len) {
    if (spi_device == NULL || len == 0) {
        return HAL_ERROR_SPI;
    }

    spi_transaction_t t = {
        .flags = 0,
        .length = (size_t)len * 8, // length in bits
        .rxlength = (size_t)len * 8, // rxlength in bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t err = spi_device_polling_transmit(spi_device, &t);

    return (err == ESP_OK) ? HAL_OK : HAL_ERROR_SPI;
}

hal_result_t hal_spi_read(uint8_t *rx_data, uint16_t len) {
    if (spi_device == NULL || len == 0) {
        return HAL_ERROR_SPI;
    }
    
    // Create a dummy TX buffer for the read operation
    // Using heap allocation for large buffers might be safer if len is huge, 
    // but for SX1262 len is usually small (< 256). Stack VLA is risky in some standards,
    // but fine for small known max sizes.
    // Better safely: use a small static buffer or memset a heap pointer if needed.
    // Given SX1262 max packet 256, stack is likely fine on ESP32 S3 (large stack).
    uint8_t dummy_tx_buf[256]; 
    if (len > 256) return HAL_ERROR_SPI; // Safety check
    
    memset(dummy_tx_buf, 0x00, len); 

    spi_transaction_t t = {
        .flags = 0,
        .length = (size_t)len * 8,
        .rxlength = (size_t)len * 8,
        .tx_buffer = dummy_tx_buf,
        .rx_buffer = rx_data,
    };

    esp_err_t err = spi_device_polling_transmit(spi_device, &t);
    return (err == ESP_OK) ? HAL_OK : HAL_ERROR_SPI;
}

void hal_spi_cs_assert(uint8_t assert) {
    // Asserted state is LOW (0)
    hal_gpio_write(hal_cs_pin, assert ? 0 : 1);
}

// ============ TIMING OPERATIONS ============

void hal_timing_init() {
    // ESP32 timer is already initialized by default
}

uint32_t hal_get_time_us() {
    return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
}

uint32_t hal_get_time_ms() {
    return (uint32_t)((esp_timer_get_time() / 1000) & 0xFFFFFFFF);
}

void hal_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void hal_delay_us(uint32_t us) {
    // Busy-wait for short microsecond delays
    uint32_t start = hal_get_time_us();
    while ((hal_get_time_us() - start) < us) {
        // Busy wait
    }
}

// ============ RESET OPERATIONS ============

hal_result_t hal_reset_init() {
    // Initialize NRESET pin as output
    hal_result_t res = hal_gpio_init_output(PIN_SX1262_NRESET);
    if (res != HAL_OK) {
        return res;
    }
    // Set to HIGH (default state)
    hal_gpio_write(PIN_SX1262_NRESET, 1);
    return HAL_OK;
}

void hal_reset_deinit() {
    gpio_reset_pin((gpio_num_t)PIN_SX1262_NRESET);
}

void hal_reset_pin_state(uint8_t assert) {
    // Asserted state is LOW
    hal_gpio_write(PIN_SX1262_NRESET, assert ? 0 : 1);
}

void hal_reset_pulse() {
    hal_reset_pin_state(1); // NRESET LOW
    hal_delay_us(TIMING_RESET_LOW_US);
    hal_reset_pin_state(0); // NRESET HIGH
    hal_delay_ms(TIMING_RESET_WAIT_MS);
}

hal_result_t hal_reset() {
    hal_reset_pulse();
    return HAL_OK;
}