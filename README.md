# SX1262 LoRa P2P Driver Suite

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![ESP32](https://img.shields.io/badge/ESP32-Supported-green.svg)](https://www.espressif.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

LoRa SX1262 P2P Driver (Arduino / ESP32)

This repository provides a fully-featured, production-grade LoRa P2P driver for the Semtech SX1262, designed for deterministic behavior, robustness, and low-power operation on ESP32-class MCUs using the Arduino framework.

Unlike many Arduino-centric LoRa libraries, this driver is built around an explicit radio state machine, strict BUSY/IRQ handling, and predictable timing, making it suitable for long-running, unattended, and reliability-critical applications.

Scope note:
This project is P2P only. LoRaWAN is intentionally out of scope.

Why this driver?

The table below summarizes the key design differences compared to commonly used Arduino LoRa libraries (e.g. RadioLib-based approaches):

Feature / Property	This Driver	Typical Arduino LoRa Libraries
Radio scope	SX1262 P2P only	Multi-radio, mixed scope
Radio state machine	Explicit, enforced	Implicit / fragmented
BUSY line handling	Strict, blocking-safe	Often partial or optimistic
IRQ-driven RX/TX	Yes (deterministic)	Mixed polling / IRQ
Thread / task safety (ESP32)	Designed for it	Generally not
Dynamic memory allocation	None	Common
Power modes	Full sleep / warm start	Limited or incomplete
AES encryption support	Integrated	Usually external / none
Replay protection	Built-in	Rare
Duty-cycle enforcement (EU/FCC)	Integrated logic	User responsibility
Failure recovery paths	Explicit	Often undefined
Target user	Embedded / systems engineers	Rapid prototyping
Design philosophy

This driver prioritizes:

Determinism over convenience

Correct radio sequencing over abstraction

Predictable behavior over feature breadth

Long-term stability over quick demos

If you are building:

unattended nodes,

low-power systems,

safety-adjacent or infrastructure-like devices,

or systems where “it usually works” is not acceptable,

this driver is designed for that class of problem.

## ✨ Features

### Core Driver
- **State Machine Architecture** - Enforced valid state transitions
- **Thread Safety** - FreeRTOS mutex protection (RAII guards)
- **Hardware Self-Test** - Verify wiring before initialization
- **Blocking TX/RX** - Simple polling-based operations
- **Fast Turnaround** - Quick TX↔RX transitions for ACK protocols
- **Semtech Errata Handling** - BW500 sensitivity fix, SPI quirks

### Power Management
- **Warm Sleep** - 600 nA, 340 µs wake, config retained
- **Cold Sleep** - 160 nA, 3.5 ms wake, auto-reconfigure
- **Power Statistics** - Track sleep time and wake cycles

### Security Module
- **AES-128-CTR Encryption** - Hardware-accelerated on ESP32
- **Replay Protection** - Monotonic counter with configurable window
- **Counter Persistence** - Automatic save/load with crash recovery

### Regulatory Compliance
- **EU868 Duty Cycle** - ETSI G1 (1%), G2 (0.1%), G3 (10%)
- **FCC 915 MHz** - US regulations support
- **Multi-Channel Tracking** - Independent airtime budgets

### Link Monitoring
- **Multi-Node Tracking** - Monitor up to 32 remote nodes
- **Signal Quality Alarms** - Silent node, weak signal, irregular timing
- **Statistics Collection** - RSSI/SNR history and trends

## ⚡ Quick Start — TX/RX in 20 Lines

**1. Set your board in `config.h`** — uncomment your board or define custom pins:

```c
// config.h — uncomment ONE:
//#define BOARD_HELTEC_WIFI_LORA_32_V3
#define BOARD_ESP32S3_WAVESHARE
//#define BOARD_ESP32_WROOM_WAVESHARE
//#define BOARD_CUSTOM                  // ← define your own pins below
```

**2. Transmitter sketch:**

```cpp
#include "sx1262_driver.h"

void setup() {
    Serial.begin(115200);
    if (sx1262_init_simple(868100000, 14) != SX1262_OK) {   // 868.1 MHz, +14 dBm
        Serial.println("Radio init failed!");
        while (1);
    }
}

void loop() {
    uint8_t msg[] = "Hello LoRa!";
    sx1262_tx_result_t tx_res;
    if (sx1262_transmit(msg, sizeof(msg), 0, &tx_res) == SX1262_OK) {
        Serial.printf("TX OK  %lu ms\n", tx_res.tx_duration_ms);
    }
    // Immediately listen for an ACK
    uint8_t buf[255];
    sx1262_rx_result_t rx_res;
    if (sx1262_turnaround_tx_to_rx(buf, sizeof(buf), 1000, &rx_res) == SX1262_OK) {
        Serial.printf("ACK!   RSSI %d dBm\n", rx_res.rssi_pkt / 2);
    }
    delay(3000);
}
```

**3. Receiver sketch:**

```cpp
#include "sx1262_driver.h"

void setup() {
    Serial.begin(115200);
    if (sx1262_init_simple(868100000, 14) != SX1262_OK) {
        Serial.println("Radio init failed!");
        while (1);
    }
}

void loop() {
    uint8_t buf[255];
    sx1262_rx_result_t rx_res;
    if (sx1262_receive(buf, sizeof(buf), 5000, &rx_res) == SX1262_OK) {
        Serial.printf("RX [%d B] RSSI %d dBm : %.*s\n",
                       rx_res.payload_length, rx_res.rssi_pkt / 2,
                       rx_res.payload_length, buf);
        // Respond immediately
        uint8_t ack[] = "ACK";
        sx1262_turnaround_rx_to_tx(ack, sizeof(ack), 0, NULL);
    }
}
```

> **That's it.** `sx1262_init_simple()` handles all SPI, IRQ, modulation, and errata configuration.  
> Both nodes must share the same frequency, SF, and BW — the defaults (`SF7 / 125 kHz / CR 4/5`) work out of the box.  
> Need more range? Use `sx1262_init_extended()` to bump to SF10/SF12.

## 📦 Installation

### Method 1: Arduino Library Manager (Not available yet)

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries...**
3. Search for "**LoRa-SX1262-P2P**"
4. Click **Install**

### Method 2: Manual Installation (ZIP)

1. Download the latest release from [GitHub Releases](https://github.com/Th3Osc4r/LoRa-SX1262-P2P-Arduino-IDE/releases)
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library...**
3. Select the downloaded ZIP file

### Method 3: Git Clone

```bash
cd ~/Arduino/libraries/
git clone https://github.com/Th3Osc4r/LoRa-SX1262-P2P-Arduino-IDE.git LoRa-SX1262-P2P
```

### Method 4: PlatformIO

Add to `platformio.ini`:

```ini
lib_deps = 
    https://github.com/Th3Osc4r/LoRa-SX1262-P2P-Arduino-IDE.git
```

Or use the library name once published:
```ini
lib_deps = 
    Th3Osc4r/LoRa-SX1262-P2P
```

## 🔧 Configuration

### Step 1: Select Your Board

Edit `config.h` in the library's `src/` folder and uncomment **exactly one** board preset:

```cpp
// ============================================================================
// BOARD SELECTION - Uncomment exactly ONE
// ============================================================================
#define BOARD_HELTEC_WIFI_LORA_32_V3      // Heltec V3 with integrated SX1262
//#define BOARD_ESP32S3_WAVESHARE           // ESP32-S3 + Waveshare module
//#define BOARD_ESP32_WROOM_WAVESHARE       // ESP32 WROOM + Waveshare module
//#define BOARD_CUSTOM                       // Manual pin configuration
```

### Step 2: Custom Board Configuration

If using `BOARD_CUSTOM`, define your pin mappings:

```cpp
#define BOARD_CUSTOM
#define BOARD_NAME "My Custom Board"

// SPI Pins
#define PIN_SX1262_MOSI     23
#define PIN_SX1262_MISO     19
#define PIN_SX1262_SCK      18
#define PIN_SX1262_CS       5

// Control Pins
#define PIN_SX1262_NRESET   16
#define PIN_SX1262_BUSY     4
#define PIN_SX1262_DIO1     17

// SPI Host (ESP32: VSPI_HOST, ESP32-S3: SPI2_HOST)
#define SPI_HOST_ID         VSPI_HOST
```

### Supported Boards

| Board | Preset Define | MCU |
|:------|:--------------|:----|
| Heltec WiFi LoRa 32 V3 | `BOARD_HELTEC_WIFI_LORA_32_V3` | ESP32-S3 |
| ESP32-S3 + Waveshare SX1262 | `BOARD_ESP32S3_WAVESHARE` | ESP32-S3 |
| ESP32 WROOM + Waveshare SX1262 | `BOARD_ESP32_WROOM_WAVESHARE` | ESP32 |
| Custom | `BOARD_CUSTOM` | Any ESP32 |

## 🚀 Quick Start

### Basic Transmitter

```cpp
#include <SX1262_LoRa.h>

void setup() {
    Serial.begin(115200);
    
    // Enable board power (required for Heltec)
    hal_board_power_init();
    
    // Initialize radio: 868.1 MHz, +14 dBm
    if (sx1262_init_simple(868100000, 14) != SX1262_OK) {
        Serial.println("Radio init failed!");
        while(1);
    }
    Serial.println("Radio ready!");
}

void loop() {
    uint8_t data[] = "Hello LoRa!";
    sx1262_tx_result_t result;
    
    if (sx1262_transmit(data, sizeof(data), 0, &result) == SX1262_OK) {
        Serial.printf("TX OK - %lu ms\n", result.tx_duration_ms);
    }
    delay(5000);
}
```

### Basic Receiver

```cpp
#include <SX1262_LoRa.h>

void setup() {
    Serial.begin(115200);
    hal_board_power_init();
    sx1262_init_simple(868100000, 14);
}

void loop() {
    uint8_t buffer[255];
    sx1262_rx_result_t result;
    
    if (sx1262_receive(buffer, sizeof(buffer), 5000, &result) == SX1262_OK) {
        int rssi_dbm = result.rssi_pkt / 2;  // Convert to dBm
        Serial.printf("RX: %d bytes, RSSI: %d dBm, SNR: %d dB\n",
                     result.payload_length, rssi_dbm, result.snr_pkt);
    }
}
```

## 📖 Examples

The library includes several example sketches:

| Example | Description |
|:--------|:------------|
| **SelfTest** | Hardware connectivity verification |
| **BasicTransmit** | Simple packet transmission |
| **BasicReceive** | Simple packet reception with signal quality |
| **PingPong** | Bidirectional communication demo |
| **SignalMeter** | Professional link quality analyzer |

Access via: **File → Examples → LoRa-SX1262-P2P**

## 📚 API Reference

### Initialization

```cpp
sx1262_result_t sx1262_init_simple(uint32_t frequency_hz, int8_t tx_power_dbm);
sx1262_result_t sx1262_init_extended(uint32_t freq, int8_t power, sx1262_lora_sf_t sf, sx1262_lora_bw_t bw);
sx1262_result_t sx1262_self_test(sx1262_self_test_result_t* result);
void hal_board_power_init(void);  // Call before init on Heltec boards
```

### TX/RX Operations

```cpp
sx1262_result_t sx1262_transmit(const uint8_t* payload, uint8_t length, 
                                 uint32_t timeout_ms, sx1262_tx_result_t* result);
sx1262_result_t sx1262_receive(uint8_t* payload, uint8_t max_length,
                                uint32_t timeout_ms, sx1262_rx_result_t* result);
sx1262_result_t sx1262_turnaround_tx_to_rx(...);  // Fast TX→RX
sx1262_result_t sx1262_turnaround_rx_to_tx(...);  // Fast RX→TX
```

### Power Management

```cpp
sx1262_result_t sx1262_sleep(sx1262_sleep_mode_t mode);  // WARM or COLD
sx1262_result_t sx1262_wake(void);
bool sx1262_is_sleeping(void);
```

### Configuration

```cpp
sx1262_result_t sx1262_set_frequency(uint32_t frequency_hz);
sx1262_result_t sx1262_set_tx_power(int8_t power_dbm, sx1262_ramp_time_t ramp);
sx1262_result_t sx1262_set_modulation_params(sf, bw, cr, ldro);
```

## 🔍 Troubleshooting

### Common Issues

| Problem | Solution |
|:--------|:---------|
| "SPI bus already initialized" after wake | Don't call `sx1262_init_simple()` after `sx1262_wake()` |
| Heltec board not responding | Call `hal_board_power_init()` first |
| TX duration 2× expected | Update to v2.0+ (TX timing fix) |
| RSSI values look wrong | Divide `rssi_pkt` by 2 for dBm |

### Hardware Self-Test

Always run `SelfTest.ino` first to verify wiring:

```cpp
sx1262_self_test_result_t test;
if (sx1262_self_test(&test) != SX1262_OK) {
    Serial.printf("FAILED: %s\n", test.failure_reason);
}
```

### Debug Logging

Enable verbose logging in `config.h`:

```cpp
#define SX1262_LOG_LEVEL SX1262_LOG_LEVEL_DEBUG  // 0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG
#define DEBUG_VERBOSE_SPI   1  // SPI transaction details
#define DEBUG_VERBOSE_STATE 1  // State machine transitions
```

## 📋 Requirements

- **MCU:** ESP32, ESP32-S3, ESP32-C3, Heltec LoRa V3, Heltec LoRa V4
- **Framework:** Arduino or ESP-IDF
- **Radio:** Semtech SX1262 (or SX1261/SX1268)
- **Arduino IDE:** 1.8.x or 2.x
- **ESP32 Board Package:** 2.0.0+

## 📄 License

GPL 2.0 License - see [LICENSE](LICENSE) file.

## 🙏 Acknowledgments

- Semtech for the SX1262 datasheet and reference implementations
- ESP32 Arduino Core maintainers
- The LoRa community for testing and feedback

## 📬 Support

- **Issues:** [GitHub Issues](https://github.com/Th3Osc4r/LoRa-SX1262-P2P-Arduino-IDE/issues)
- **Discussions:** [GitHub Discussions](https://github.com/Th3Osc4r/LoRa-SX1262-P2P-Arduino-IDE/discussions)

---

**Made with ❤️ for the LoRa community**
