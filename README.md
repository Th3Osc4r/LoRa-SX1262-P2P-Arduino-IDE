# LoRa-SX1262-P2P-
Fully featured Arduino/ C++ driver Suite for P2P SX1662 Radio

SX1262 LoRa Driver Suite
Technical Documentation
Production-Grade LoRa Communication for ESP32-S3

Version 1.0.0
December 2024

Target Application: Railway Monitoring Systems
15-Minute Sensor Intervals with Real-Time Alarm Capability
 
Table of Contents
1. Introduction
2. Architecture Overview
3. Quick Start Guide
4. Core Driver API
5. Radio Configuration
6. TX/RX Operations
7. Power Management
8. Optional Modules
    8.1 Security Module (AES-128-CTR)
    8.2 Duty Cycle Manager (ETSI Compliance)
    8.3 Link Health Monitor
9. Error Handling
10. Hardware Configuration
11. Troubleshooting
12. API Reference Summary
 
1. Introduction
The SX1262 LoRa Driver Suite is a comprehensive, production-grade driver for the Semtech SX1262 LoRa transceiver, specifically designed for ESP32-S3 microcontrollers. This driver follows a  philosophy that prioritizes reliability and robustness over performance optimization.
1.1 Key Features
•	Complete LoRa transmission and reception with hardware interrupts
•	Thread-safe FreeRTOS integration with mutex protection
•	Comprehensive power management (WARM/COLD sleep modes)
•	Silicon errata workarounds (including critical Errata 15.2)
•	Platform-agnostic architecture with clean HAL separation
•	Optional security module with AES-128-CTR encryption
•	ETSI duty cycle compliance manager
•	Multi-node link health monitoring
1.2 Target Application
This driver is conceived to support efficient and reliable bidirectional communication.
1.3 Design Philosophy
•	Reliability over performance: Every function prioritizes correct operation
•	Defensive programming: Comprehensive error checking and recovery
•	Modularity: Optional features don't affect core driver stability
•	Testability: Extensive logging and diagnostic capabilities
 
2. Architecture Overview
The driver follows a 4-layer architecture with clear separation of concerns:
Layer	Description
Application Layer	User code: sensors, gateways, application logic
Optional Modules	Security (AES-128-CTR), Duty Cycle, Link Health
Driver Layer	State machine, TX/RX operations, power management
SPI Protocol Layer	SX1262 command encoding, BUSY handling, diagnostics
HAL Layer	ESP32-S3 hardware abstraction (GPIO, SPI, timing)
2.1 File Structure
File	Purpose
sx1262_driver.h/cpp	Main driver: state machine, TX/RX, power management
sx1262_config.cpp	Radio configuration: frequency, power, modulation
sx1262_spi_protocol.h/cpp	SPI command layer with BUSY handling
sx1262_hal.h/cpp	Hardware abstraction (ESP32-S3 specific)
sx1262_errata.h/cpp	Silicon bug workarounds
sx1262_regs.h	Register and command definitions
config.h	Pin mappings and compile-time options
sx1262_security.h/cpp	Optional: AES-128-CTR encryption module
sx1262_duty_cycle.h/cpp	Optional: ETSI duty cycle compliance
sx1262_link_health.h/cpp	Optional: Multi-node health monitoring
 
3. Quick Start Guide
Get up and running with minimal code using the simplified initialization API.
3.1 Minimal Transmitter Example
#include "sx1262_driver.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize at 868.1 MHz, +14 dBm
    if (sx1262_init_simple(868100000, 14) != SX1262_OK) {
        Serial.println("Init failed!");
        while(1);
    }
    Serial.println("Radio ready!");
}

void loop() {
    uint8_t data[] = "Hello LoRa!";
    sx1262_transmit(data, sizeof(data), 0, NULL);
    delay(5000);
}
3.2 Minimal Receiver Example
void loop() {
    uint8_t rx_buffer[255];
    sx1262_rx_result_t result;
    
    if (sx1262_receive(rx_buffer, 255, 10000, &result) == SX1262_OK) {
        Serial.printf("Received %d bytes, RSSI: %d dBm\n",
            result.payload_length, result.rssi_pkt / 2);
    }
}
3.3 Bidirectional Communication (Sensor with ACK)
// Transmit sensor data
sx1262_transmit(sensor_data, len, 0, NULL);

// Wait for gateway ACK (500ms window)
sx1262_rx_result_t rx_result;
if (sx1262_turnaround_tx_to_rx(ack_buf, 32, 500, &rx_result) == SX1262_OK) {
    // ACK received
}

// Enter sleep until next cycle
sx1262_sleep(SX1262_SLEEP_WARM);
 
4. Core Driver API
4.1 Initialization Functions
Function	Description
sx1262_init_simple(freq, power)	Simplified init with frequency and TX power
sx1262_init_extended(...)	Extended init with SF and BW control
sx1262_driver_init()	Low-level driver initialization (no radio config)
sx1262_driver_deinit()	Shutdown driver and release resources
sx1262_driver_is_initialized()	Check if driver is ready
4.2 Return Codes
Code	Meaning
SX1262_OK (0)	Operation successful
SX1262_ERROR_INVALID_STATE (-1)	Invalid state for operation
SX1262_ERROR_INVALID_PARAM (-2)	Invalid parameter value
SX1262_ERROR_TIMEOUT (-3)	Operation timed out
SX1262_ERROR_SPI (-4)	SPI communication error
SX1262_ERROR_BUSY_TIMEOUT (-5)	BUSY pin stuck high
SX1262_ERROR_NOT_INITIALIZED (-6)	Driver not initialized
SX1262_ERROR_MUTEX (-7)	Failed to acquire mutex
SX1262_ERROR_HARDWARE (-9)	Hardware error (CRC, IRQ)
4.3 Radio States
State	Description
SX1262_STATE_UNINITIALIZED	Driver not initialized
SX1262_STATE_SLEEP	Low-power sleep mode
SX1262_STATE_STANDBY_RC	Standby with RC oscillator
SX1262_STATE_STANDBY_XOSC	Standby with crystal oscillator
SX1262_STATE_TX	Transmitting
SX1262_STATE_RX	Receiving
SX1262_STATE_ERROR	Error state (needs reset)
 
5. Radio Configuration
5.1 LoRa Parameters
Parameter	Range	Default
Frequency	150-960 MHz	868.1 MHz (EU868)
TX Power	-9 to +22 dBm	+22 dBm
Spreading Factor	SF5-SF12	SF7 (balanced)
Bandwidth	7.8-500 kHz	125 kHz (standard)
Coding Rate	4/5 to 4/8	4/5
Preamble	6-65535 symbols	8 symbols
Sync Word	16-bit	0x1424 (private)
5.2 Configuration Structure
typedef struct {
    uint32_t frequency_hz;           // RF frequency
    int8_t tx_power_dbm;             // TX power (-9 to +22)
    sx1262_ramp_time_t ramp_time;    // PA ramp time
    sx1262_lora_sf_t spreading_factor;
    sx1262_lora_bw_t bandwidth;
    sx1262_lora_cr_t coding_rate;
    bool low_data_rate_optimize;     // Auto-enabled for long symbols
    uint16_t preamble_length;
    sx1262_lora_header_t header_type;
    uint8_t payload_length;
    sx1262_lora_crc_t crc_type;
    sx1262_lora_iq_t invert_iq;
    uint16_t sync_word;
} sx1262_lora_config_t;
5.3 Configuration Functions
// Initialize with defaults
sx1262_config_init_defaults(&config);

// Modify as needed
config.frequency_hz = 868300000;
config.spreading_factor = SX1262_LORA_SF9;

// Apply configuration
sx1262_config_lora(&config);
 
6. TX/RX Operations
6.1 Transmission
sx1262_result_t sx1262_transmit(
    const uint8_t* payload,    // Data to transmit (1-255 bytes)
    uint8_t length,            // Payload length
    uint32_t timeout_ms,       // Timeout (0 = auto from ToA)
    sx1262_tx_result_t* result // Optional result structure
);
The TX result structure provides detailed transmission information:
typedef struct {
    bool success;              // TX completed successfully
    uint32_t tx_start_ms;      // Start timestamp
    uint32_t tx_done_ms;       // Completion timestamp
    uint32_t tx_duration_ms;   // Actual duration
    uint32_t calculated_toa_ms;// Calculated Time-on-Air
    bool timed_out;            // Timeout occurred
    uint16_t irq_flags;        // IRQ status at completion
} sx1262_tx_result_t;
6.2 Reception
sx1262_result_t sx1262_receive(
    uint8_t* payload,          // Buffer for received data
    uint8_t max_length,        // Buffer size
    uint32_t timeout_ms,       // RX window duration
    sx1262_rx_result_t* result // Optional result structure
);
The RX result structure provides signal quality information:
typedef struct {
    bool success;              // Packet received successfully
    uint32_t rx_duration_ms;   // RX duration
    bool timed_out;            // Timeout occurred
    bool crc_error;            // CRC error detected
    uint8_t payload_length;    // Received payload length
    int16_t rssi_pkt;          // RSSI (divide by 2 for dBm)
    int8_t snr_pkt;            // SNR in dB
} sx1262_rx_result_t;
6.3 TX/RX Turnaround Functions
For bidirectional communication, use the turnaround functions:
// After TX, wait for response
sx1262_turnaround_tx_to_rx(rx_buf, 255, 500, &rx_result);

// After RX, send response
sx1262_turnaround_rx_to_tx(tx_buf, len, 0, &tx_result);
 
7. Power Management
7.1 Sleep Modes
The SX1262 supports two sleep modes with different power/wake tradeoffs:
Mode	Current	Wake Time	Config
WARM	~600 nA	~340 µs	Retained
COLD	~160 nA	~3.5 ms	Lost (auto-restored)
7.2 Sleep/Wake API
// Enter sleep mode
sx1262_sleep(SX1262_SLEEP_WARM);  // Config retained
sx1262_sleep(SX1262_SLEEP_COLD);  // Lowest power

// Wake from sleep
sx1262_wake();  // Auto-restores config for COLD sleep

// Check sleep state
if (sx1262_is_sleeping()) { ... }

// Get power statistics
uint32_t total_sleep_ms, wake_count;
sx1262_get_power_stats(&total_sleep_ms, &wake_count);
7.3 Typical Sensor Duty Cycle
void loop() {
    // Wake from sleep
    if (sx1262_is_sleeping()) {
        sx1262_wake();
    }
    
    // Read sensor and transmit
    uint8_t data[16];
    read_sensor(data);
    sx1262_transmit(data, sizeof(data), 0, NULL);
    
    // Wait for ACK
    sx1262_turnaround_tx_to_rx(ack_buf, 32, 500, NULL);
    
    // Enter sleep for 15 minutes
    sx1262_sleep(SX1262_SLEEP_WARM);
    esp_deep_sleep(15 * 60 * 1000000ULL);
}
 
8. Optional Modules
The driver suite includes three optional modules that can be enabled independently without modifying core driver files.
8.1 Security Module (AES-128-CTR)
Provides payload encryption with replay protection for secure communication.
Features
•	AES-128-CTR stream cipher encryption
•	4-byte counter for replay protection
•	Automatic counter persistence via callbacks
•	Platform-agnostic crypto HAL
Usage
// Initialize with 128-bit key
uint8_t key[16] = { 0x00, 0x11, 0x22, ... };
sx1262_security_context_t sec_ctx;
sx1262_security_init(&sec_ctx, key, 0);

// Encrypt before transmission
uint8_t buffer[64];
memcpy(buffer, payload, payload_len);
uint8_t secure_len;
sx1262_secure_pack(&sec_ctx, buffer, payload_len, &secure_len);
sx1262_transmit(buffer, secure_len, 0, NULL);

// Decrypt after reception
uint8_t decrypted_len;
sx1262_secure_unpack(&sec_ctx, rx_buffer, rx_len, &decrypted_len);
Counter Persistence
For security to survive reboots, the TX counter must be saved to non-volatile storage:
// With automatic storage callbacks (recommended)
sx1262_security_storage_t storage = {
    .save = my_nvs_save,
    .load = my_nvs_load,
    .save_interval = 0,     // Save every TX
    .safety_margin = 100    // Skip 100 on load
};
sx1262_security_init_with_storage(&sec_ctx, key, &storage, 0);
 
8.2 Duty Cycle Manager (ETSI Compliance)
Tracks transmission airtime and enforces regulatory duty cycle limits.
ETSI Sub-bands
Sub-band	Frequency	Duty Cycle
g1	868.0-868.6 MHz	1% (36 sec/hour)
g2	868.7-869.2 MHz	0.1% (3.6 sec/hour)
g3	869.4-869.65 MHz	10% (360 sec/hour)
Usage
// Initialize for ETSI g1 (1% duty cycle)
sx1262_dc_init(SX1262_DC_REGION_ETSI_G1);

// Before each transmission
uint32_t wait_ms;
uint32_t toa = sx1262_get_time_on_air_ms(&config, payload_len);

if (sx1262_dc_can_transmit(toa, &wait_ms)) {
    sx1262_transmit(payload, len, 0, &result);
    sx1262_dc_record_transmission(result.tx_duration_ms);
} else {
    Serial.printf("Must wait %lu ms\n", wait_ms);
}
8.3 Link Health Monitor
Tracks communication quality with remote nodes for gateway applications.
Alarm Types
•	SILENT: Node hasn't transmitted in expected window
•	IRREGULAR: Transmission timing varies significantly
•	WEAK_SIGNAL: RSSI below threshold
•	RECOVERED: Previously alarmed node recovered
Usage
// Initialize with callback
void on_alarm(const sx1262_link_alarm_t* alarm) {
    Serial.printf("Node %d: %s\n", alarm->node_id,
        sx1262_link_alarm_type_to_string(alarm->type));
}
sx1262_link_health_init(20, on_alarm);

// Register expected nodes (15-min interval = 900000 ms)
sx1262_link_health_register_node(0x01, 900000);
sx1262_link_health_register_node(0x02, 900000);

// Record receptions
sx1262_link_health_record_rx(node_id, rssi, snr);

// Periodic check (in main loop)
sx1262_link_health_check_timeouts();
 
9. Error Handling
9.1 Error Checking Pattern
sx1262_result_t res = sx1262_transmit(data, len, 0, NULL);
if (res != SX1262_OK) {
    Serial.printf("TX failed: %s\n", sx1262_error_to_string(res));
    
    // Get detailed error info
    sx1262_result_t err;
    const char* func;
    uint32_t timestamp;
    sx1262_get_last_error(&err, &func, &timestamp);
}
9.2 Watchdog Protection
The driver includes a watchdog that triggers emergency reset after consecutive failures:
// Enable watchdog with 3 failure threshold
sx1262_watchdog_init(3);
sx1262_watchdog_enable(true);

// Feed watchdog after successful operations
if (res == SX1262_OK) {
    sx1262_watchdog_feed();
}
9.3 Emergency Reset
If the radio enters an unrecoverable state, the driver automatically performs hardware reset and re-initialization.
 
10. Hardware Configuration
10.1 Default Pin Mapping (ESP32-S3)
Function	GPIO	Notes
NRESET	16	Hardware reset control
BUSY	5	Ready/busy status indicator
DIO1	2	Interrupt output (TX/RX done)
CS (NSS)	10	SPI chip select
MOSI	11	SPI Master Out
MISO	13	SPI Master In
SCLK	12	SPI Clock
10.2 Modifying Pin Configuration
Edit config.h to change pin assignments:
#define PIN_SX1262_NRESET   16
#define PIN_SX1262_BUSY     5
#define PIN_SX1262_DIO1     2
#define PIN_SX1262_CS       10
10.3 SPI Configuration
#define SPI_FREQUENCY_HZ    10000000  // 10 MHz
#define PIN_SPI_MOSI        11
#define PIN_SPI_MISO        13
#define PIN_SPI_SCLK        12
 
11. Troubleshooting
11.1 Common Issues
BUSY Timeout on Init
•	Check wiring to BUSY pin
•	Verify power supply (3.3V, >200mA capability)
•	Try hardware reset before init
TX Timeout
•	Verify antenna connection
•	Check frequency is within valid range
•	Increase timeout value
No RX Packets
•	Verify TX and RX use same frequency/SF/BW
•	Check sync word matches on both ends
•	Ensure CRC setting matches
Configuration Not Retained After Sleep
•	Use WARM sleep for config retention
•	COLD sleep requires full reconfig (handled automatically)
11.2 Debug Logging
Enable verbose logging in config.h:
#define DEBUG_ENABLED       1
#define DEBUG_VERBOSE_SPI   1
#define DEBUG_VERBOSE_STATE 1
11.3 Silicon Errata
The driver includes workarounds for known SX1262 silicon bugs:
•	Errata 15.2: IRQ config cleared after SetRx - automatically re-applied
•	Sensitivity degradation fix applied after wake
•	PA clamp voltage optimization for high power
 
12. API Reference Summary
12.1 Core Functions
// Initialization
sx1262_init_simple(freq_hz, power_dbm)
sx1262_driver_init()
sx1262_driver_deinit()

// TX/RX Operations
sx1262_transmit(payload, length, timeout_ms, result)
sx1262_receive(buffer, max_len, timeout_ms, result)
sx1262_turnaround_tx_to_rx(...)
sx1262_turnaround_rx_to_tx(...)

// Power Management
sx1262_sleep(mode)
sx1262_wake()
sx1262_is_sleeping()
sx1262_get_power_stats(...)

// Configuration
sx1262_config_lora(config)
sx1262_set_frequency(freq_hz)
sx1262_set_tx_power(power_dbm, ramp_time)
sx1262_get_time_on_air_ms(config, payload_len)
12.2 Security Module
sx1262_security_init(ctx, key, initial_counter)
sx1262_security_init_with_storage(ctx, key, storage, fallback)
sx1262_secure_pack(ctx, buffer, len, out_len)
sx1262_secure_unpack(ctx, buffer, len, out_len)
sx1262_security_force_save(ctx)
12.3 Duty Cycle Manager
sx1262_dc_init(region)
sx1262_dc_can_transmit(airtime_ms, wait_ms)
sx1262_dc_record_transmission(airtime_ms)
sx1262_dc_get_budget_remaining_ms()
12.4 Link Health Monitor
sx1262_link_health_init(max_nodes, callback)
sx1262_link_health_register_node(node_id, interval_ms)
sx1262_link_health_record_rx(node_id, rssi, snr)
sx1262_link_health_check_timeouts()
