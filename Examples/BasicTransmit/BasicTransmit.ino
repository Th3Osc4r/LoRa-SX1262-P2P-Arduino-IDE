/**
 * ============================================================================
 * BasicTransmit.ino - Simple LoRa Transmitter
 * ============================================================================
 * SX1262 LoRa P2P Driver Suite
 * 
 * Demonstrates basic packet transmission with the SX1262 driver.
 * Sends a counter packet every 5 seconds with transmission statistics.
 * 
 * SETUP:
 *   1. Edit config.h and uncomment your board preset
 *   2. Adjust FREQUENCY and TX_POWER for your region
 *   3. Upload to transmitter board
 *   4. Run BasicReceive.ino on another board
 * 
 * EU868 (Europe):  868100000 Hz, max +14 dBm (ETSI)
 * US915 (USA):     915000000 Hz, max +22 dBm (FCC)
 * 
 * ============================================================================
 */

#include <SX1262_LoRa.h>

// ============================================================================
// CONFIGURATION - Adjust for your region
// ============================================================================
#define FREQUENCY       868100000   // 868.1 MHz (EU868)
#define TX_POWER        14          // +14 dBm (ETSI limit for EU)
#define TX_INTERVAL_MS  5000        // Transmit every 5 seconds

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
uint32_t packetCounter = 0;
uint32_t lastTxTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println();
    Serial.println("================================================");
    Serial.println("  SX1262 Basic Transmitter");
    Serial.println("================================================");
    Serial.printf("Board: %s\n", BOARD_NAME);
    Serial.printf("Frequency: %.2f MHz\n", FREQUENCY / 1e6);
    Serial.printf("TX Power: %d dBm\n", TX_POWER);
    Serial.println();
    
    // Step 1: Enable board power (required for Heltec)
    hal_board_power_init();
    
    // Step 2: Initialize radio
    Serial.print("Initializing radio... ");
    sx1262_result_t result = sx1262_init_simple(FREQUENCY, TX_POWER);
    
    if (result != SX1262_OK) {
        Serial.println("FAILED!");
        char help[128];
        sx1262_get_init_error_help(result, help, sizeof(help));
        Serial.printf("Error: %s\n", help);
        Serial.println("\nRun SelfTest.ino to diagnose hardware issues.");
        while (1) delay(1000);
    }
    
    Serial.println("OK!");
    Serial.println();
    Serial.println("Starting transmission loop...");
    Serial.println("─────────────────────────────────────────────────");
}

void loop() {
    uint32_t now = millis();
    
    if (now - lastTxTime >= TX_INTERVAL_MS) {
        lastTxTime = now;
        packetCounter++;
        
        // Build payload
        char payload[64];
        int len = snprintf(payload, sizeof(payload), "Hello #%lu from %s", 
                          packetCounter, BOARD_NAME);
        
        // Transmit
        Serial.printf("[TX #%lu] Sending %d bytes... ", packetCounter, len);
        
        sx1262_tx_result_t txResult;
        sx1262_result_t result = sx1262_transmit(
            (uint8_t*)payload, 
            len, 
            0,          // Auto timeout based on ToA
            &txResult
        );
        
        if (result == SX1262_OK) {
            Serial.printf("OK! (Duration: %lu ms, ToA: %lu ms)\n", 
                         txResult.tx_duration_ms, 
                         txResult.calculated_toa_ms);
        } else {
            Serial.printf("FAILED! Error: %s\n", sx1262_error_to_string(result));
        }
    }
}
