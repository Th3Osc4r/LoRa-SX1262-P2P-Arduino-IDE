# SX1262 Driver Suite: User's Manual & Integration Guide
**Version:** 2.0.0  
**Date:** January 2025  
**Target Platform:** ESP32 (S3/WROOM/Heltec V3)  
**Architecture:** Single-Radio per MCU

---

## 1. System Overview

This software suite is a professional-grade driver for the Semtech SX1262 LoRa transceiver. Unlike standard hobbyist libraries, it is architected for **high-reliability industrial applications**. It enforces regulatory compliance, data security, and autonomous fault recovery.

### 1.1 Architecture
The system follows a layered architecture to ensure separation of concerns:

| Layer | Module | Responsibility |
| :--- | :--- | :--- |
| **Application** | `main.cpp` | User logic, sensor reading |
| **Middleware** | `sx1262_security` | AES-128 Encryption & Replay Protection |
| | `sx1262_duty_cycle` | ETSI/FCC Airtime Enforcement |
| | `sx1262_link_health`| Network Monitoring & Silent Node Alarms |
| **Core Driver** | `sx1262_driver` | State Machine, TX/RX Operations, Power Management |
| | `sx1262_config` | Radio Configuration & Validation |
| | `sx1262_errata` | Silicon Bug Workarounds |
| **Protocol** | `sx1262_spi_protocol` | SX126x Command Set Implementation |
| **HAL** | `sx1262_hal` | GPIO, SPI, Platform Abstraction |
| | `sx1262_crypto_hal` | AES Hardware/Software Abstraction |

---

## 2. Hardware Setup

### 2.1 Supported Board Presets

The driver includes pre-configured pin mappings for common hardware combinations. Select your board in `config.h`:

| Board Preset | Define | Description |
|:-------------|:-------|:------------|
| Heltec WiFi LoRa 32 V3 | `BOARD_HELTEC_WIFI_LORA_32_V3` | Integrated SX1262, OLED, VEXT power control |
| ESP32-S3 + Waveshare | `BOARD_ESP32S3_WAVESHARE` | diymore N16R8 + Waveshare SX1262 HF module |
| ESP32 WROOM + Waveshare | `BOARD_ESP32_WROOM_WAVESHARE` | Standard ESP32 + Waveshare SX1262 module |
| Custom | `BOARD_CUSTOM` | Manual pin configuration |

### 2.2 Board Configuration

Edit `config.h` and uncomment **exactly one** board preset:

```cpp
// ============================================================================
// BOARD SELECTION - Uncomment exactly ONE
// ============================================================================
//#define BOARD_HELTEC_WIFI_LORA_32_V3      // Heltec V3 with integrated SX1262
#define BOARD_ESP32S3_WAVESHARE             // ESP32-S3 + Waveshare module
//#define BOARD_ESP32_WROOM_WAVESHARE       // ESP32 WROOM + Waveshare module
//#define BOARD_CUSTOM                       // Manual configuration below
```

### 2.3 Pin Mappings by Board

#### Heltec WiFi LoRa 32 V3
| Function | GPIO | Notes |
|:---------|:-----|:------|
| MOSI | 10 | SPI2_HOST |
| MISO | 11 | |
| SCK | 9 | |
| CS (NSS) | 8 | |
| RESET | 12 | |
| BUSY | 13 | Input |
| DIO1 | 14 | Interrupt |
| VEXT | 36 | Power control (active LOW) |
| LED | 35 | Onboard LED |

#### ESP32-S3 + Waveshare
| Function | GPIO | Notes |
|:---------|:-----|:------|
| MOSI | 11 | SPI2_HOST |
| MISO | 13 | |
| SCK | 12 | |
| CS (NSS) | 10 | |
| RESET | 16 | |
| BUSY | 5 | Input |
| DIO1 | 2 | Interrupt |

#### ESP32 WROOM + Waveshare
| Function | GPIO | Notes |
|:---------|:-----|:------|
| MOSI | 23 | VSPI_HOST |
| MISO | 19 | |
| SCK | 18 | |
| CS (NSS) | 5 | |
| RESET | 16 | |
| BUSY | 4 | Input |
| DIO1 | 17 | Interrupt |

### 2.4 Custom Board Configuration

If using `BOARD_CUSTOM`, define all pins manually:

```cpp
#ifdef BOARD_CUSTOM
    #define BOARD_NAME "Custom Board"
    
    // SPI Pins
    #define PIN_SX1262_MOSI     23
    #define PIN_SX1262_MISO     19
    #define PIN_SX1262_SCK      18
    #define PIN_SX1262_CS       5
    
    // Control Pins
    #define PIN_SX1262_RESET    16
    #define PIN_SX1262_BUSY     4
    #define PIN_SX1262_DIO1     17
    
    // SPI Host
    #define SPI_HOST_ID         VSPI_HOST
    
    // Optional: Power control (uncomment if needed)
    // #define PIN_VEXT          36
#endif
```

### 2.5 Hardware Self-Test

Before initializing the driver, run the self-test to verify connections:

```cpp
#include "sx1262_driver.h"

void setup() {
    Serial.begin(115200);
    
    // For Heltec boards: Enable VEXT power first
    hal_board_power_init();
    
    sx1262_self_test_result_t test;
    if (sx1262_self_test(&test) != SX1262_OK) {
        Serial.printf("Hardware test FAILED: %s\n", test.failure_reason);
        while(1);
    }
    Serial.println("Hardware OK!");
}
```

---

## 3. Quick Start Guide

### 3.1 Basic Setup

```cpp
#include <Arduino.h>
#include "config.h"
#include "sx1262_driver.h"
#include "sx1262_hal.h"

void setup() {
    Serial.begin(SERIAL_BAUD);  // Uses baud rate from config.h
    
    // Step 1: Enable board power (required for Heltec, no-op for others)
    hal_board_power_init();
    
    // Step 2: Initialize driver with EU868 at +14dBm
    sx1262_result_t res = sx1262_init_simple(868100000, 14);
    if (res != SX1262_OK) {
        char help[128];
        sx1262_get_init_error_help(res, help, sizeof(help));
        Serial.printf("Init failed: %s\n", help);
        while(1);
    }
    
    Serial.println("Radio ready!");
}
```

### 3.2 Transmitting a Packet

```cpp
uint8_t payload[] = "Hello World";
sx1262_tx_result_t tx_result;

if (sx1262_transmit(payload, sizeof(payload), 0, &tx_result) == SX1262_OK) {
    Serial.printf("TX OK - Duration: %lu ms, ToA: %lu ms\n", 
                  tx_result.tx_duration_ms, tx_result.calculated_toa_ms);
}
```

### 3.3 Receiving a Packet

```cpp
uint8_t rx_buf[255];
sx1262_rx_result_t rx_result;

if (sx1262_receive(rx_buf, sizeof(rx_buf), 5000, &rx_result) == SX1262_OK) {
    // RSSI conversion: divide by 2 for dBm
    int rssi_dbm = rx_result.rssi_pkt / 2;
    Serial.printf("RX OK - %d bytes, RSSI: %d dBm, SNR: %d dB\n", 
                  rx_result.payload_length, rssi_dbm, rx_result.snr_pkt);
}
```

### 3.4 TX/RX Turnaround (Ping-Pong)

For applications requiring fast TX→RX or RX→TX transitions:

```cpp
// After transmitting, wait for ACK
sx1262_rx_result_t rx_result;
if (sx1262_turnaround_tx_to_rx(ack_buf, 32, 500, &rx_result) == SX1262_OK) {
    Serial.println("ACK received!");
}

// After receiving, send response
sx1262_tx_result_t tx_result;
sx1262_turnaround_rx_to_tx(response, len, 0, &tx_result);
```

---

## 4. Power Management

### 4.1 Sleep Modes

The SX1262 supports two sleep modes:

| Mode | Current | Wake Time | Config | Use Case |
|:-----|:--------|:----------|:-------|:---------|
| WARM | ~600 nA | ~340 µs | Retained | Frequent wake (seconds/minutes) |
| COLD | ~160 nA | ~3.5 ms | Lost | Long sleep (hours/days) |

### 4.2 Sleep/Wake Cycle

```cpp
// Enter sleep mode
sx1262_sleep(SX1262_SLEEP_WARM);  // or SX1262_SLEEP_COLD

// ... MCU sleep or delay ...

// Wake from sleep
if (sx1262_is_sleeping()) {
    sx1262_wake();
    // Radio is immediately ready for TX/RX
    // Driver automatically restores config after COLD wake
}
```

**Important:** After calling `sx1262_wake()`, the driver automatically handles:
- WARM wake: Radio ready immediately (config retained in chip)
- COLD wake: Driver re-applies stored LoRa configuration automatically

**Do NOT** call `sx1262_init_simple()` after wake - this will fail with "SPI bus already initialized".

### 4.3 Complete Duty Cycle Example

```cpp
void loop() {
    // Wake if sleeping
    if (sx1262_is_sleeping()) {
        sx1262_wake();
    }
    
    // Transmit sensor data
    sx1262_transmit(sensor_data, len, 0, &tx_result);
    
    // Wait for ACK
    sx1262_turnaround_tx_to_rx(ack_buf, 32, 500, &rx_result);
    
    // Enter sleep
    sx1262_sleep(SX1262_SLEEP_WARM);
    
    // MCU sleep (in production, use esp_deep_sleep)
    delay(15 * 60 * 1000);  // 15 minutes
}
```

### 4.4 Power Statistics

```cpp
uint32_t total_sleep_ms, wake_count;
sx1262_get_power_stats(&total_sleep_ms, &wake_count);

if (wake_count > 0) {
    float avg_sleep = (float)total_sleep_ms / wake_count;
    Serial.printf("Average sleep: %.1f ms over %lu cycles\n", avg_sleep, wake_count);
}
```

---

## 5. Security Layer (AES-128)

### 5.1 Overview

The security module provides:
- **AES-128-CTR encryption** for payload confidentiality
- **Replay protection** using monotonic counters
- **Automatic counter persistence** via storage callbacks

### 5.2 Basic Usage (Manual Counter Management)

```cpp
#include "sx1262_security.h"

sx1262_security_context_t sec_ctx;
uint8_t key[16] = { /* your 128-bit key */ };

// Initialize with counter from NVS
uint32_t saved_counter = nvs_read_counter();
sx1262_security_init(&sec_ctx, key, saved_counter + 100);  // +100 safety margin

// Encrypt before TX
uint8_t buffer[64];
memcpy(buffer, payload, payload_len);
uint8_t secure_len;
sx1262_secure_pack(&sec_ctx, buffer, payload_len, &secure_len);
sx1262_transmit(buffer, secure_len, 0, NULL);

// Save counter periodically
nvs_write_counter(sx1262_security_get_tx_counter(&sec_ctx));
```

### 5.3 Recommended: Automatic Counter Persistence

For production use, configure storage callbacks for automatic counter management:

```cpp
// Define storage callbacks
bool save_counter(uint32_t counter, void* ctx) {
    Preferences prefs;
    prefs.begin("lora", false);
    prefs.putUInt("tx_ctr", counter);
    prefs.end();
    return true;
}

bool load_counter(uint32_t* counter, void* ctx) {
    Preferences prefs;
    prefs.begin("lora", true);
    *counter = prefs.getUInt("tx_ctr", 0);
    prefs.end();
    return true;
}

// Initialize with storage
sx1262_security_storage_t storage = {
    .save = save_counter,
    .load = load_counter,
    .user_ctx = NULL,
    .save_interval = 0,     // Save every TX (safest)
    .safety_margin = 100    // Skip 100 counters on load (handles crash recovery)
};

sx1262_security_init_with_storage(&sec_ctx, key, &storage, 0);
```

### 5.4 Decryption with Replay Protection

```cpp
// After receiving
uint8_t payload_len;
sx1262_security_result_t res = sx1262_secure_unpack(&sec_ctx, rx_buf, rx_len, &payload_len);

if (res == SX1262_SEC_OK) {
    // rx_buf now contains decrypted payload
} else if (res == SX1262_SEC_ERROR_REPLAY) {
    Serial.println("Replay attack detected!");
}
```

### 5.5 Force Save Before Sleep

```cpp
// Before entering deep sleep or shutdown
sx1262_security_force_save(&sec_ctx);
```

---

## 6. Regulatory Compliance (Duty Cycle)

### 6.1 Regional Presets

| Region | Preset | Duty Cycle Limit |
|:-------|:-------|:-----------------|
| Europe (G1) | `SX1262_DC_REGION_ETSI_G1` | 1% |
| Europe (G2) | `SX1262_DC_REGION_ETSI_G2` | 0.1% |
| Europe (G3) | `SX1262_DC_REGION_ETSI_G3` | 10% |
| USA 915 MHz | `SX1262_DC_REGION_FCC_915` | FCC rules |

### 6.2 Usage

```cpp
#include "sx1262_duty_cycle.h"

// Initialize for EU868 G1 band (1% duty cycle)
sx1262_dc_init(SX1262_DC_REGION_ETSI_G1);

// Before transmitting
uint32_t wait_ms;
if (sx1262_dc_can_transmit(tx_duration_ms, &wait_ms)) {
    sx1262_transmit(payload, len, 0, &tx_result);
    sx1262_dc_record_transmission(tx_result.tx_duration_ms);
} else {
    Serial.printf("Duty cycle exceeded - wait %lu ms\n", wait_ms);
}
```

---

## 7. Link Health Monitor

### 7.1 Setup

```cpp
#include "sx1262_link_health.h"

void alarm_callback(uint32_t node_id, sx1262_link_alarm_t alarm) {
    switch (alarm) {
        case SX1262_LINK_ALARM_SILENT:
            Serial.printf("Node %lu: No signal!\n", node_id);
            break;
        case SX1262_LINK_ALARM_WEAK_SIGNAL:
            Serial.printf("Node %lu: Weak signal\n", node_id);
            break;
        case SX1262_LINK_ALARM_IRREGULAR:
            Serial.printf("Node %lu: Irregular timing\n", node_id);
            break;
    }
}

sx1262_link_health_init(32, alarm_callback);  // Track up to 32 nodes
```

### 7.2 Recording Packets

```cpp
// After each successful RX
sx1262_link_health_record_rx(node_id, rx_result.rssi_pkt, rx_result.snr_pkt);
```

---

## 8. Troubleshooting

### 8.1 Error Codes

| Error Code | Meaning | Solution |
|:-----------|:--------|:---------|
| `SX1262_ERROR_SPI` | SPI communication failed | Check wiring, run `sx1262_self_test()` |
| `SX1262_ERROR_BUSY_TIMEOUT` | Chip unresponsive | Check 3.3V supply, BUSY pin wiring |
| `SX1262_ERROR_TIMEOUT` | TX/RX timeout | Check DIO1 wiring, increase timeout |
| `SX1262_ERROR_NOT_INITIALIZED` | Driver not initialized | Call `sx1262_init_simple()` first |
| `SX1262_ERROR_HARDWARE` | Hardware error/CRC fail | Check antenna, RF environment |
| `SX1262_SEC_ERROR_REPLAY` | Replay attack detected | Re-sync counters between devices |

### 8.2 Common Issues

**"SPI bus already initialized" after wake:**
- Do NOT call `sx1262_init_simple()` after `sx1262_wake()`
- The driver handles reconfiguration automatically

**TX Duration is 2× expected:**
- Ensure you're using driver v2.0+ with the spi_set_tx fix
- Check that `spi_set_tx()` returns immediately (doesn't wait for BUSY)

**Heltec board not responding:**
- Call `hal_board_power_init()` before driver initialization
- This enables VEXT power to the radio module

**RSSI values look wrong:**
- Raw `rssi_pkt` must be divided by 2 for dBm: `int rssi_dbm = rx_result.rssi_pkt / 2;`

### 8.3 Self-Test Failures

| Test | Failure Reason | Check |
|:-----|:---------------|:------|
| BUSY pin | "BUSY pin not readable" | GPIO wiring, pullup resistor |
| SPI | "SPI communication failed" | MOSI/MISO/SCK/CS wiring |
| Chip ID | "Invalid chip response" | Power supply, module compatibility |
| Reset | "Reset not working" | NRST pin wiring |
| DIO1 | "DIO1 not readable" | DIO1 GPIO wiring |

---

## 9. Project Structure

```text
MyLoRaProject/
├── platformio.ini
├── src/
│   ├── main.cpp                    # Your application
│   ├── config.h                    # Board selection & pin definitions
│   ├── sx1262_driver.cpp           # Core driver
│   ├── sx1262_driver.h
│   ├── sx1262_hal.cpp              # Hardware abstraction
│   ├── sx1262_hal.h
│   ├── sx1262_config.cpp           # Radio configuration
│   ├── sx1262_spi_protocol.cpp     # SPI commands
│   ├── sx1262_spi_protocol.h
│   ├── sx1262_errata.cpp           # Silicon bug fixes
│   ├── sx1262_errata.h
│   ├── sx1262_regs.h               # Register definitions
│   ├── sx1262_mutex_guard.h        # RAII mutex wrapper
│   ├── sx1262_duty_cycle.cpp       # Regulatory compliance
│   ├── sx1262_duty_cycle.h
│   ├── sx1262_link_health.cpp      # Network monitoring
│   ├── sx1262_link_health.h
│   ├── sx1262_security.cpp         # Encryption
│   ├── sx1262_security.h
│   ├── sx1262_crypto_hal.h         # Crypto abstraction
│   └── sx1262_crypto_hal_esp32.cpp # ESP32 hardware AES
```

---

## 10. Version History

### v2.0.0 (January 2025)
- **Board Preset System:** Unified `config.h` with pre-configured board presets
- **VEXT Power Control:** `hal_board_power_init()` for Heltec boards
- **Cold Wake Fix:** Driver automatically re-applies config after cold sleep
- **TX Timing Fix:** `spi_set_tx()` no longer blocks during transmission
- **Security Storage Callbacks:** Automatic counter persistence with `sx1262_security_init_with_storage()`

### v1.0.0 (December 2024)
- Initial release
- Core driver with TX/RX operations
- Security module (AES-128-CTR)
- Duty cycle enforcement
- Link health monitoring

---

## Appendix A: PlatformIO Configuration

```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200

build_flags = 
    -D CORE_DEBUG_LEVEL=3
    -D SX1262_LOG_LEVEL=3  ; 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG
```

---

## Appendix B: API Quick Reference

### Initialization
| Function | Description |
|:---------|:------------|
| `hal_board_power_init()` | Enable board power (VEXT for Heltec) |
| `sx1262_self_test(&result)` | Hardware connectivity test |
| `sx1262_init_simple(freq, power)` | Simple init with defaults |
| `sx1262_init_extended(freq, power, sf, bw)` | Extended init with SF/BW |

### TX/RX Operations
| Function | Description |
|:---------|:------------|
| `sx1262_transmit(data, len, timeout, &result)` | Blocking transmit |
| `sx1262_receive(buf, max, timeout, &result)` | Blocking receive |
| `sx1262_turnaround_tx_to_rx(...)` | Fast TX→RX transition |
| `sx1262_turnaround_rx_to_tx(...)` | Fast RX→TX transition |

### Power Management
| Function | Description |
|:---------|:------------|
| `sx1262_sleep(mode)` | Enter WARM or COLD sleep |
| `sx1262_wake()` | Wake from sleep (auto-reconfigures) |
| `sx1262_is_sleeping()` | Check sleep state |
| `sx1262_get_power_stats(&sleep_ms, &wakes)` | Get power statistics |

### Security
| Function | Description |
|:---------|:------------|
| `sx1262_security_init(&ctx, key, counter)` | Basic init |
| `sx1262_security_init_with_storage(&ctx, key, &storage, fallback)` | Init with auto-persistence |
| `sx1262_secure_pack(&ctx, buf, len, &out_len)` | Encrypt payload |
| `sx1262_secure_unpack(&ctx, buf, len, &out_len)` | Decrypt payload |
| `sx1262_security_force_save(&ctx)` | Force counter save |

### Diagnostics
| Function | Description |
|:---------|:------------|
| `sx1262_error_to_string(error)` | Error code to string |
| `sx1262_get_init_error_help(error, buf, len)` | Detailed error help |
| `sx1262_print_status()` | Print driver state |
