/**
 * ============================================================================
 * PingPong.ino - Bidirectional LoRa Communication
 * ============================================================================
 * SX1262 LoRa P2P Driver Suite
 * 
 * Demonstrates bidirectional communication using fast TX/RX turnaround.
 * Upload to TWO boards - one starts as "initiator", the other responds.
 * 
 * How it works:
 *   - Initiator sends PING, waits for PONG response
 *   - Responder listens, receives PING, sends PONG back
 *   - Roles can switch automatically or be fixed
 * 
 * SETUP:
 *   1. Edit config.h on both boards
 *   2. Set IS_INITIATOR to true on ONE board, false on the other
 *   3. Upload to both boards
 *   4. Watch the ping-pong exchange!
 * 
 * ============================================================================
 */

#include <SX1262_LoRa.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define FREQUENCY           868100000   // Must match on both devices
#define TX_POWER            14          // dBm

#define IS_INITIATOR        true        // Set to 'true' on ONE board only!

#define PING_INTERVAL_MS    3000        // Time between pings (initiator)
#define ACK_TIMEOUT_MS      1000        // Time to wait for response
#define LISTEN_TIMEOUT_MS   5000        // Responder listen window

// ============================================================================
// PACKET DEFINITIONS
// ============================================================================
#define MSG_PING    0x01
#define MSG_PONG    0x02

typedef struct {
    uint8_t type;           // MSG_PING or MSG_PONG
    uint32_t sequence;      // Packet counter
    int16_t rssi;           // Sender's last received RSSI (for reporting)
    int8_t snr;             // Sender's last received SNR
} __attribute__((packed)) PingPongPacket;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t missedCount = 0;
int16_t lastRssi = 0;
int8_t lastSnr = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println();
    Serial.println("================================================");
    Serial.println("  SX1262 Ping-Pong Demo");
    Serial.println("================================================");
    Serial.printf("Board: %s\n", BOARD_NAME);
    Serial.printf("Role: %s\n", IS_INITIATOR ? "INITIATOR (sends first)" : "RESPONDER (listens first)");
    Serial.printf("Frequency: %.2f MHz\n", FREQUENCY / 1e6);
    Serial.println();
    
    // Initialize hardware
    hal_board_power_init();
    
    Serial.print("Initializing radio... ");
    sx1262_result_t result = sx1262_init_simple(FREQUENCY, TX_POWER);
    
    if (result != SX1262_OK) {
        Serial.println("FAILED!");
        char help[128];
        sx1262_get_init_error_help(result, help, sizeof(help));
        Serial.printf("Error: %s\n", help);
        while (1) delay(1000);
    }
    Serial.println("OK!\n");
    
    if (IS_INITIATOR) {
        Serial.println("Starting as INITIATOR - will send PING packets");
    } else {
        Serial.println("Starting as RESPONDER - waiting for PING packets");
    }
    Serial.println("─────────────────────────────────────────────────");
}

void loop() {
    if (IS_INITIATOR) {
        initiatorLoop();
    } else {
        responderLoop();
    }
}

// ============================================================================
// INITIATOR: Send PING, wait for PONG
// ============================================================================
void initiatorLoop() {
    static uint32_t lastPing = 0;
    
    if (millis() - lastPing >= PING_INTERVAL_MS) {
        lastPing = millis();
        txCount++;
        
        // Build PING packet
        PingPongPacket ping;
        ping.type = MSG_PING;
        ping.sequence = txCount;
        ping.rssi = lastRssi;
        ping.snr = lastSnr;
        
        Serial.printf("\n[PING #%lu] Sending... ", txCount);
        
        // Transmit PING
        sx1262_tx_result_t txResult;
        sx1262_result_t result = sx1262_transmit(
            (uint8_t*)&ping, sizeof(ping), 0, &txResult);
        
        if (result != SX1262_OK) {
            Serial.printf("TX Failed: %s\n", sx1262_error_to_string(result));
            return;
        }
        Serial.print("OK, waiting for PONG... ");
        
        // Fast turnaround to RX for PONG response
        uint8_t rxBuf[sizeof(PingPongPacket)];
        sx1262_rx_result_t rxResult;
        
        result = sx1262_turnaround_tx_to_rx(rxBuf, sizeof(rxBuf), ACK_TIMEOUT_MS, &rxResult);
        
        if (result == SX1262_OK) {
            PingPongPacket* pong = (PingPongPacket*)rxBuf;
            
            if (pong->type == MSG_PONG && pong->sequence == txCount) {
                rxCount++;
                lastRssi = rxResult.rssi_pkt;
                lastSnr = rxResult.snr_pkt;
                
                Serial.printf("PONG received!\n");
                Serial.printf("  Round-trip: %lu ms\n", rxResult.rx_done_ms - txResult.tx_start_ms);
                Serial.printf("  Local  RSSI: %d dBm, SNR: %d dB\n", 
                             rxResult.rssi_pkt / 2, rxResult.snr_pkt);
                Serial.printf("  Remote RSSI: %d dBm, SNR: %d dB\n", 
                             pong->rssi / 2, pong->snr);
            } else {
                Serial.println("Invalid response!");
                missedCount++;
            }
        } else if (result == SX1262_ERROR_TIMEOUT) {
            Serial.println("TIMEOUT - no response");
            missedCount++;
        } else {
            Serial.printf("RX Error: %s\n", sx1262_error_to_string(result));
            missedCount++;
        }
        
        // Print statistics periodically
        Serial.printf("  Stats: %lu sent, %lu received, %lu missed (%.1f%% success)\n",
                     txCount, rxCount, missedCount,
                     txCount > 0 ? (100.0 * rxCount / txCount) : 0.0);
    }
}

// ============================================================================
// RESPONDER: Wait for PING, send PONG back
// ============================================================================
void responderLoop() {
    uint8_t rxBuf[sizeof(PingPongPacket)];
    sx1262_rx_result_t rxResult;
    
    // Listen for PING
    sx1262_result_t result = sx1262_receive(rxBuf, sizeof(rxBuf), LISTEN_TIMEOUT_MS, &rxResult);
    
    if (result == SX1262_OK) {
        PingPongPacket* ping = (PingPongPacket*)rxBuf;
        
        if (ping->type == MSG_PING) {
            rxCount++;
            lastRssi = rxResult.rssi_pkt;
            lastSnr = rxResult.snr_pkt;
            
            Serial.printf("\n[PING #%lu] Received (RSSI: %d dBm, SNR: %d dB)\n", 
                         ping->sequence, rxResult.rssi_pkt / 2, rxResult.snr_pkt);
            Serial.printf("  Remote reports: RSSI %d dBm, SNR %d dB\n",
                         ping->rssi / 2, ping->snr);
            
            // Build PONG response
            PingPongPacket pong;
            pong.type = MSG_PONG;
            pong.sequence = ping->sequence;
            pong.rssi = lastRssi;
            pong.snr = lastSnr;
            
            // Fast turnaround to TX for PONG
            Serial.print("  Sending PONG... ");
            sx1262_tx_result_t txResult;
            result = sx1262_turnaround_rx_to_tx(
                (uint8_t*)&pong, sizeof(pong), 0, &txResult);
            
            if (result == SX1262_OK) {
                txCount++;
                Serial.printf("OK! (Stats: %lu RX, %lu TX)\n", rxCount, txCount);
            } else {
                Serial.printf("Failed: %s\n", sx1262_error_to_string(result));
            }
        }
    } else if (result == SX1262_ERROR_TIMEOUT) {
        Serial.print(".");  // Show we're still listening
    } else {
        Serial.printf("\n[ERROR] %s\n", sx1262_error_to_string(result));
    }
}
