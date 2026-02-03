/**
 * ============================================================================
 * BasicReceive.ino - Simple LoRa Receiver
 * ============================================================================
 * SX1262 LoRa P2P Driver Suite
 * 
 * Demonstrates basic packet reception with the SX1262 driver.
 * Continuously listens for packets and displays signal quality metrics.
 * 
 * SETUP:
 *   1. Edit config.h and uncomment your board preset
 *   2. Match FREQUENCY with the transmitter
 *   3. Upload to receiver board
 *   4. Run BasicTransmit.ino on another board
 * 
 * Signal Quality Guide:
 *   RSSI: -30 to -120 dBm (higher = stronger signal)
 *         > -70 dBm  = Excellent
 *         -70 to -90 = Good
 *         -90 to -110 = Weak
 *         < -110 = Poor (near noise floor)
 * 
 *   SNR: -20 to +10 dB (higher = cleaner signal)
 *        > 5 dB   = Excellent
 *        0 to 5   = Good
 *        -5 to 0  = Acceptable
 *        < -5     = Poor (signal near noise)
 * 
 * ============================================================================
 */

#include <SX1262_LoRa.h>

// ============================================================================
// CONFIGURATION - Must match transmitter!
// ============================================================================
#define FREQUENCY       868100000   // 868.1 MHz (EU868)
#define RX_TIMEOUT_MS   10000       // 10 second receive window

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
uint32_t packetsReceived = 0;
uint32_t packetsError = 0;
uint32_t packetsTimeout = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println();
    Serial.println("================================================");
    Serial.println("  SX1262 Basic Receiver");
    Serial.println("================================================");
    Serial.printf("Board: %s\n", BOARD_NAME);
    Serial.printf("Frequency: %.2f MHz\n", FREQUENCY / 1e6);
    Serial.printf("RX Timeout: %d ms\n", RX_TIMEOUT_MS);
    Serial.println();
    
    // Step 1: Enable board power (required for Heltec)
    hal_board_power_init();
    
    // Step 2: Initialize radio
    Serial.print("Initializing radio... ");
    sx1262_result_t result = sx1262_init_simple(FREQUENCY, 14);  // TX power doesn't matter for RX
    
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
    Serial.println("Listening for packets...");
    Serial.println("─────────────────────────────────────────────────");
}

void loop() {
    uint8_t rxBuffer[255];
    sx1262_rx_result_t rxResult;
    
    // Start receiving
    sx1262_result_t result = sx1262_receive(
        rxBuffer, 
        sizeof(rxBuffer), 
        RX_TIMEOUT_MS, 
        &rxResult
    );
    
    if (result == SX1262_OK) {
        packetsReceived++;
        
        // Convert RSSI (raw value must be divided by 2 for dBm)
        int rssi_dbm = rxResult.rssi_pkt / 2;
        
        // Null-terminate if it looks like text
        if (rxResult.payload_length < sizeof(rxBuffer)) {
            rxBuffer[rxResult.payload_length] = '\0';
        }
        
        // Determine signal quality
        const char* quality;
        if (rssi_dbm > -70 && rxResult.snr_pkt > 5) {
            quality = "Excellent";
        } else if (rssi_dbm > -90 && rxResult.snr_pkt > 0) {
            quality = "Good";
        } else if (rssi_dbm > -110 && rxResult.snr_pkt > -5) {
            quality = "Weak";
        } else {
            quality = "Poor";
        }
        
        Serial.println();
        Serial.printf("[RX #%lu] Received %d bytes\n", packetsReceived, rxResult.payload_length);
        Serial.printf("  Payload: \"%s\"\n", (char*)rxBuffer);
        Serial.printf("  RSSI: %d dBm | SNR: %d dB | Quality: %s\n", 
                     rssi_dbm, rxResult.snr_pkt, quality);
        Serial.printf("  RX Duration: %lu ms\n", rxResult.rx_duration_ms);
        
    } else if (result == SX1262_ERROR_TIMEOUT) {
        packetsTimeout++;
        Serial.print(".");  // Show activity during timeout
        
    } else if (result == SX1262_ERROR_HARDWARE && rxResult.crc_error) {
        packetsError++;
        Serial.printf("\n[CRC ERROR] Corrupted packet received (RSSI: %d dBm)\n", 
                     rxResult.rssi_pkt / 2);
        
    } else {
        Serial.printf("\n[ERROR] %s\n", sx1262_error_to_string(result));
    }
    
    // Periodic statistics
    static uint32_t lastStats = 0;
    if (millis() - lastStats > 30000) {
        lastStats = millis();
        Serial.println();
        Serial.println("─────────────────────────────────────────────────");
        Serial.printf("Stats: %lu received | %lu CRC errors | %lu timeouts\n",
                     packetsReceived, packetsError, packetsTimeout);
        Serial.println("─────────────────────────────────────────────────");
    }
}
