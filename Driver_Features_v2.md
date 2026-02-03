# SX1262 Driver Suite: Feature Reference
**Version:** 2.0.0  
**Date:** January 2025

---

## Complete Feature List

### 1. Core Radio Driver (`sx1262_driver`)

#### Initialization & Configuration
| Feature | Description | API |
|:--------|:------------|:----|
| Simple Init | Single-function setup with SF7/BW125 defaults | `sx1262_init_simple()` |
| Extended Init | Custom SF/BW configuration | `sx1262_init_extended()` |
| Hardware Self-Test | Verify SPI, BUSY, DIO1, chip ID before init | `sx1262_self_test()` |
| Dynamic Reconfiguration | Change frequency/power at runtime | `sx1262_set_frequency()`, `sx1262_set_tx_power()` |
| Init Error Help | Human-readable diagnostics on failure | `sx1262_get_init_error_help()` |

#### Transmit & Receive Operations
| Feature | Description | API |
|:--------|:------------|:----|
| Blocking Transmit | Send with auto ToA calculation & timeout | `sx1262_transmit()` |
| Blocking Receive | Listen with timeout, CRC check, RSSI/SNR | `sx1262_receive()` |
| TX→RX Turnaround | Fast mode switch for ping-pong | `sx1262_turnaround_tx_to_rx()` |
| RX→TX Turnaround | Fast mode switch for gateway ACKs | `sx1262_turnaround_rx_to_tx()` |
| Prepare for TX | Manual state preparation | `sx1262_prepare_for_tx()` |
| Prepare for RX | Manual state preparation | `sx1262_prepare_for_rx()` |

#### Power Management
| Feature | Description | API |
|:--------|:------------|:----|
| Warm Sleep | 600 nA, 340 µs wake, config retained | `sx1262_sleep(SX1262_SLEEP_WARM)` |
| Cold Sleep | 160 nA, 3.5 ms wake, config lost | `sx1262_sleep(SX1262_SLEEP_COLD)` |
| Auto Wake Config | Driver restores config after cold wake | `sx1262_wake()` |
| Sleep Detection | Check if radio is sleeping | `sx1262_is_sleeping()` |
| Power Statistics | Cumulative sleep time & wake count | `sx1262_get_power_stats()` |

#### System Reliability
| Feature | Description | API |
|:--------|:------------|:----|
| State Machine | Enforces valid state transitions | Internal |
| Watchdog Timer | Auto-recovery after consecutive failures | `sx1262_watchdog_init()` |
| Emergency Reset | Hard reset with config restore | `sx1262_emergency_reset()` |
| Thread Safety | RAII mutex guards for RTOS | `SX1262MutexGuard` |
| Errata Handling | BW500 sensitivity fix, SPI quirks | `sx1262_errata` module |

#### Diagnostics
| Feature | Description | API |
|:--------|:------------|:----|
| Error Strings | Human-readable error messages | `sx1262_error_to_string()` |
| Status Printing | Debug output of driver state | `sx1262_print_status()` |
| Transition History | Circular buffer of state changes | `sx1262_print_transition_history()` |
| Statistics | Uptime, transitions, errors | `sx1262_get_statistics()` |

---

### 2. Hardware Abstraction Layer (`sx1262_hal`)

#### Platform Support
| Feature | Description | API |
|:--------|:------------|:----|
| Board Presets | Pre-configured pin mappings | `config.h` defines |
| SPI Abstraction | Transfer, CS control | `hal_spi_transfer()` |
| GPIO Abstraction | Read/write, interrupt attach | `hal_gpio_read()`, `hal_gpio_write()` |
| Timing | Millisecond delays & timestamps | `hal_delay_ms()`, `hal_get_time_ms()` |

#### Board Power Control (v2.0)
| Feature | Description | API |
|:--------|:------------|:----|
| VEXT Power Init | Enable power on Heltec boards | `hal_board_power_init()` |
| VEXT Power Deinit | Disable power (for deep sleep) | `hal_board_power_deinit()` |

#### Supported Boards
| Board | Preset Define | Notes |
|:------|:--------------|:------|
| Heltec WiFi LoRa 32 V3 | `BOARD_HELTEC_WIFI_LORA_32_V3` | Integrated SX1262, VEXT |
| ESP32-S3 + Waveshare | `BOARD_ESP32S3_WAVESHARE` | External module |
| ESP32 WROOM + Waveshare | `BOARD_ESP32_WROOM_WAVESHARE` | External module |
| Custom | `BOARD_CUSTOM` | Manual pin config |

---

### 3. Security Module (`sx1262_security`)

#### Encryption
| Feature | Description | API |
|:--------|:------------|:----|
| AES-128-CTR | Stream cipher encryption | `sx1262_secure_pack()` |
| In-place Operation | Encrypts/decrypts in buffer | Both pack/unpack |
| 4-byte Counter Header | Prepended to encrypted payload | Automatic |

#### Replay Protection
| Feature | Description | API |
|:--------|:------------|:----|
| Monotonic Counter | Rejects old/repeated counters | `sx1262_secure_unpack()` |
| Configurable Window | Tolerance for packet loss | `sx1262_security_set_replay_window()` |

#### Counter Persistence (v2.0)
| Feature | Description | API |
|:--------|:------------|:----|
| Storage Callbacks | User-provided save/load functions | `sx1262_security_init_with_storage()` |
| Auto-Save | Save after every TX (or interval) | `save_interval` config |
| Safety Margin | Skip counters on load (crash recovery) | `safety_margin` config |
| Force Save | Manual save before sleep/shutdown | `sx1262_security_force_save()` |
| Storage Check | Verify callbacks are configured | `sx1262_security_has_storage()` |

#### Statistics
| Feature | Description | API |
|:--------|:------------|:----|
| Packet Counts | Encrypted/decrypted totals | `sx1262_security_get_stats()` |
| Replay Rejections | Counter of blocked replays | Part of stats |
| Storage Metrics | Save count, failure count | Part of stats |

#### Crypto HAL
| Feature | Description | API |
|:--------|:------------|:----|
| Hardware AES | ESP32 mbedTLS acceleration | `sx1262_crypto_hal_esp32.cpp` |
| Software AES | Portable fallback (tiny-AES-c) | Optional |

---

### 4. Duty Cycle Manager (`sx1262_duty_cycle`)

#### Regulatory Compliance
| Feature | Description | API |
|:--------|:------------|:----|
| ETSI G1 (1%) | Europe 868 MHz band | `SX1262_DC_REGION_ETSI_G1` |
| ETSI G2 (0.1%) | Europe restricted | `SX1262_DC_REGION_ETSI_G2` |
| ETSI G3 (10%) | Europe high-power | `SX1262_DC_REGION_ETSI_G3` |
| FCC 915 MHz | USA regulations | `SX1262_DC_REGION_FCC_915` |

#### Airtime Management
| Feature | Description | API |
|:--------|:------------|:----|
| Transmit Check | Query if TX is allowed | `sx1262_dc_can_transmit()` |
| Wait Time | Get remaining cooldown | Part of can_transmit |
| Record TX | Log transmission duration | `sx1262_dc_record_transmission()` |
| Multi-Channel | Track 8 channels independently | Channel parameter |

#### Algorithm
| Feature | Description |
|:--------|:------------|
| Sliding Window | 1-hour rolling airtime budget |
| Burst Allowance | Configurable burst tolerance |
| Real-time Tracking | Millisecond precision |

---

### 5. Link Health Monitor (`sx1262_link_health`)

#### Node Tracking
| Feature | Description | API |
|:--------|:------------|:----|
| Multi-Node | Track up to 32 nodes (configurable) | `sx1262_link_health_init()` |
| Per-Node Stats | RSSI, SNR, packet timing | Internal |
| History Buffer | Rolling window of metrics | Internal |

#### Alarm Types
| Alarm | Trigger | Default Threshold |
|:------|:--------|:------------------|
| Silent Node | No packets for 3× expected interval | Configurable |
| Weak Signal | RSSI below threshold | -110 dBm |
| Irregular Timing | Jitter > 50% | Configurable |
| Node Recovery | Previously alarmed node responds | Auto-cleared |

#### Notification
| Feature | Description | API |
|:--------|:------------|:----|
| Callback System | Async notification on alarm | Init callback parameter |
| Alarm History | Query past alarms | `sx1262_link_health_get_alarms()` |

---

### 6. SPI Protocol (`sx1262_spi_protocol`)

#### Command Implementation
| Command | Description | API |
|:--------|:------------|:----|
| SetStandby | Enter standby mode | `spi_set_standby()` |
| SetTx | Start transmission | `spi_set_tx()` |
| SetRx | Start reception | `spi_set_rx()` |
| SetSleep | Enter sleep mode | `spi_set_sleep()` |
| GetStatus | Read chip status | `spi_get_status()` |
| GetPacketStatus | Read RSSI/SNR | `spi_get_packet_status()` |
| WriteBuffer | Write TX data | `spi_write_buffer()` |
| ReadBuffer | Read RX data | `spi_read_buffer()` |
| SetFrequency | Configure RF frequency | `spi_set_rf_frequency()` |
| SetTxParams | Configure TX power | `spi_set_tx_params()` |
| SetModulationParams | Configure SF/BW/CR | `spi_set_modulation_params_lora()` |
| SetPacketParams | Configure preamble/header/CRC | `spi_set_packet_params_lora()` |

#### BUSY Handling
| Feature | Description |
|:--------|:------------|
| Pre-command Wait | Wait for BUSY LOW before commands |
| TX/RX Immediate Return | SetTx/SetRx return without blocking (v2.0 fix) |
| Timeout Protection | Configurable BUSY timeout |

---

### 7. Configuration (`config.h`)

#### Board Selection
```cpp
// Uncomment exactly ONE:
#define BOARD_HELTEC_WIFI_LORA_32_V3
//#define BOARD_ESP32S3_WAVESHARE
//#define BOARD_ESP32_WROOM_WAVESHARE
//#define BOARD_CUSTOM
```

#### Compile-Time Validation
- Missing pin definitions trigger `#error`
- Ensures all required GPIOs are defined before compile

#### Logging Configuration
| Level | Define | Output |
|:------|:-------|:-------|
| 0 | `SX1262_LOG_LEVEL=0` | Silent |
| 1 | `SX1262_LOG_LEVEL=1` | Errors only |
| 2 | `SX1262_LOG_LEVEL=2` | + Warnings |
| 3 | `SX1262_LOG_LEVEL=3` | + Info |
| 4 | `SX1262_LOG_LEVEL=4` | + Debug |

#### Module-Specific Debug
| Define | Effect |
|:-------|:-------|
| `DEBUG_VERBOSE_SPI` | Detailed SPI transaction logs |
| `DEBUG_VERBOSE_STATE` | State machine transitions |
| `DEBUG_VERBOSE_HAL` | HAL operations |
| `DEBUG_VERBOSE_SECURITY` | Crypto operations |

---

## Version 2.0 Changes Summary

### New Features
1. **Unified Board Preset System** - Single `config.h` with board selection
2. **VEXT Power Control** - `hal_board_power_init()` for Heltec boards
3. **Security Storage Callbacks** - Automatic counter persistence
4. **Cold Wake Auto-Config** - Driver restores settings after cold sleep

### Bug Fixes
1. **TX Timing Fix** - `spi_set_tx()` returns immediately (was blocking for ToA)
2. **RSSI Calculation** - Documented correct conversion (divide by 2)

### Breaking Changes
- Separate `config_heltec_v3.h` / `config_wroom_waveshare.h` are deprecated
- Use unified `config.h` with board preset selection instead

---

## Hardware Requirements

| Component | Requirement |
|:----------|:------------|
| MCU | ESP32, ESP32-S3, ESP32-C3 |
| Radio | Semtech SX1262 |
| Framework | Arduino or ESP-IDF |
| RTOS | FreeRTOS (standard on ESP32) |
| Crypto | mbedTLS (standard on ESP32) |

---

## Memory Footprint

| Module | Flash | RAM |
|:-------|:------|:----|
| Core Driver | ~50 KB | ~2 KB |
| Security | ~22 KB | ~1 KB |
| Duty Cycle | ~18 KB | ~0.5 KB |
| Link Health | ~28 KB | ~2 KB (32 nodes) |
| **Total** | ~120 KB | ~5.5 KB |

*Approximate values, varies with optimization level and debug settings.*
