/**
 * ============================================================================
 * SX1262_LoRa.h - Unified Include Header
 * ============================================================================
 * SX1262 LoRa P2P Driver Suite v2.0.0
 * 
 * This is a convenience header that includes all essential driver components.
 * For most applications, simply include this file:
 * 
 *   #include <SX1262_LoRa.h>
 * 
 * This automatically includes:
 *   - config.h           (Board configuration & pin mappings)
 *   - sx1262_driver.h    (Core driver API)
 *   - sx1262_hal.h       (Hardware abstraction layer)
 * 
 * Optional modules (include separately if needed):
 *   - sx1262_security.h     (AES-128 encryption & replay protection)
 *   - sx1262_duty_cycle.h   (ETSI/FCC regulatory compliance)
 *   - sx1262_link_health.h  (Network monitoring & alarms)
 * 
 * QUICK START:
 * ============
 * 
 * 1. Edit config.h and uncomment your board preset:
 *    #define BOARD_HELTEC_WIFI_LORA_32_V3
 *    // or
 *    #define BOARD_ESP32S3_WAVESHARE
 *    // or
 *    #define BOARD_CUSTOM  (then define pins manually)
 * 
 * 2. In your sketch:
 * 
 *    #include <SX1262_LoRa.h>
 *    
 *    void setup() {
 *        Serial.begin(115200);
 *        hal_board_power_init();  // Required for Heltec boards
 *        
 *        if (sx1262_init_simple(868100000, 14) == SX1262_OK) {
 *            Serial.println("Radio ready!");
 *        }
 *    }
 *    
 *    void loop() {
 *        uint8_t data[] = "Hello LoRa!";
 *        sx1262_transmit(data, sizeof(data), 0, NULL);
 *        delay(5000);
 *    }
 * 
 * ============================================================================
 * https://github.com/Th3Osc4r/LoRa-SX1262-P2P-Arduino-IDE
 * ============================================================================
 */

#ifndef SX1262_LORA_H
#define SX1262_LORA_H

// Core components (always needed)
#include "config.h"
#include "sx1262_driver.h"
#include "sx1262_hal.h"

// Version information
#define SX1262_LORA_VERSION_MAJOR   2
#define SX1262_LORA_VERSION_MINOR   0
#define SX1262_LORA_VERSION_PATCH   0
#define SX1262_LORA_VERSION_STRING  "2.0.0"

#endif // SX1262_LORA_H
