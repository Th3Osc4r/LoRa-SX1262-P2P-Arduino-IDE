# SX1262 LoRa P2P Library - Source Files

Copy all your driver source files (.cpp and .h) to this `src/` folder.

## Required Files

From your repository root, copy these files here:

### Core Driver
- `config.h`
- `sx1262_driver.h`
- `sx1262_driver.cpp`
- `sx1262_hal.h`
- `sx1262_hal.cpp`
- `sx1262_spi_protocol.h`
- `sx1262_spi_protocol.cpp`
- `sx1262_regs.h`
- `sx1262_config.cpp`

### Errata & Utilities
- `sx1262_errata.h`
- `sx1262_errata.cpp`
- `sx1262_mutex_guard.h`

### Optional Modules
- `sx1262_security.h`
- `sx1262_security.cpp`
- `sx1262_crypto_hal.h`
- `sx1262_crypto_hal_esp32.cpp`
- `sx1262_duty_cycle.h`
- `sx1262_duty_cycle.cpp`
- `sx1262_link_health.h`
- `sx1262_link_health.cpp`

### Board Configs (Deprecated - use unified config.h)
These are optional for backward compatibility:
- `config_heltec_v3.h`
- `config_wroom_waveshare.h`

## File Already Present
- `SX1262_LoRa.h` - Unified include header (already created)

## After Copying

Your `src/` folder should contain approximately 20+ files.
The `SX1262_LoRa.h` header will include the essential files automatically.
