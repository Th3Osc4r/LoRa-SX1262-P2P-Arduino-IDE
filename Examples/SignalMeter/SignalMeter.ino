/**
 * ============================================================================
 * SignalMeter.ino - LoRa Link Quality Analyzer
 * ============================================================================
 * SX1262 LoRa P2P Driver Suite
 * 
 * Professional signal strength meter for site surveys and link testing.
 * Displays real-time RSSI, SNR, packet statistics, and link margin.
 * 
 * Features:
 *   - Real-time signal quality metrics
 *   - Running averages and statistics
 *   - Link margin calculation
 *   - Packet loss tracking
 *   - CSV output for logging
 * 
 * Use with BasicTransmit.ino on the transmitter side.
 * 
 * ============================================================================
 */

#include <SX1262_LoRa.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define FREQUENCY           868100000   // Must match transmitter
#define RX_TIMEOUT_MS       5000        // Receive window

// Link margin thresholds (dB above sensitivity floor)
// SX1262 sensitivity at SF7/BW125: approximately -124 dBm
#define SENSITIVITY_FLOOR   -124        // dBm (SF7, BW125)
#define MARGIN_EXCELLENT    20          // >20 dB margin
#define MARGIN_GOOD         10          // 10-20 dB margin  
#define MARGIN_MARGINAL     5           // 5-10 dB margin
// Below 5 dB = Poor

// ============================================================================
// STATISTICS TRACKING
// ============================================================================
#define HISTORY_SIZE        100         // Rolling window size

struct Statistics {
    uint32_t packetsReceived;
    uint32_t packetsLost;               // Based on sequence gaps
    uint32_t crcErrors;
    
    // Signal quality history (circular buffer)
    int16_t rssiHistory[HISTORY_SIZE];
    int8_t snrHistory[HISTORY_SIZE];
    uint8_t historyIndex;
    uint8_t historyCount;
    
    // Running totals for averages
    int32_t rssiSum;
    int32_t snrSum;
    
    // Min/Max tracking
    int16_t rssiMin;
    int16_t rssiMax;
    int8_t snrMin;
    int8_t snrMax;
    
    // Sequence tracking for packet loss
    uint32_t lastSequence;
    bool firstPacket;
    
    // Timing
    uint32_t startTime;
    uint32_t lastPacketTime;
};

Statistics stats;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║       SX1262 LoRa Signal Meter v2.0              ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.printf("Board: %s\n", BOARD_NAME);
    Serial.printf("Frequency: %.3f MHz\n", FREQUENCY / 1e6);
    Serial.printf("Sensitivity Floor: %d dBm (SF7/BW125)\n", SENSITIVITY_FLOOR);
    Serial.println();
    
    // Initialize hardware
    hal_board_power_init();
    
    Serial.print("Initializing radio... ");
    sx1262_result_t result = sx1262_init_simple(FREQUENCY, 14);
    
    if (result != SX1262_OK) {
        Serial.println("FAILED!");
        char help[128];
        sx1262_get_init_error_help(result, help, sizeof(help));
        Serial.printf("Error: %s\n", help);
        while (1) delay(1000);
    }
    Serial.println("OK!\n");
    
    // Initialize statistics
    memset(&stats, 0, sizeof(stats));
    stats.rssiMin = 0;
    stats.rssiMax = -200;
    stats.snrMin = 127;
    stats.snrMax = -128;
    stats.firstPacket = true;
    stats.startTime = millis();
    
    Serial.println("Waiting for transmitter...");
    Serial.println("Use BasicTransmit.ino on the TX side.");
    Serial.println();
    Serial.println("CSV Format: timestamp,rssi_dbm,snr_db,margin_db,packets,loss_pct");
    Serial.println("────────────────────────────────────────────────────────────────");
}

void loop() {
    uint8_t rxBuffer[255];
    sx1262_rx_result_t rxResult;
    
    sx1262_result_t result = sx1262_receive(rxBuffer, sizeof(rxBuffer), RX_TIMEOUT_MS, &rxResult);
    
    if (result == SX1262_OK) {
        processPacket(rxBuffer, &rxResult);
    } else if (result == SX1262_ERROR_HARDWARE && rxResult.crc_error) {
        stats.crcErrors++;
        Serial.printf("[CRC ERROR] Count: %lu\n", stats.crcErrors);
    } else if (result != SX1262_ERROR_TIMEOUT) {
        Serial.printf("[ERROR] %s\n", sx1262_error_to_string(result));
    }
    
    // Periodic summary every 30 seconds
    static uint32_t lastSummary = 0;
    if (stats.packetsReceived > 0 && millis() - lastSummary > 30000) {
        lastSummary = millis();
        printSummary();
    }
}

void processPacket(uint8_t* payload, sx1262_rx_result_t* rx) {
    stats.packetsReceived++;
    stats.lastPacketTime = millis();
    
    // Convert RSSI (divide by 2 for dBm)
    int16_t rssi_dbm = rx->rssi_pkt / 2;
    int8_t snr_db = rx->snr_pkt;
    
    // Calculate link margin
    int linkMargin = rssi_dbm - SENSITIVITY_FLOOR;
    
    // Update history (circular buffer)
    stats.rssiHistory[stats.historyIndex] = rssi_dbm;
    stats.snrHistory[stats.historyIndex] = snr_db;
    stats.historyIndex = (stats.historyIndex + 1) % HISTORY_SIZE;
    if (stats.historyCount < HISTORY_SIZE) stats.historyCount++;
    
    // Update running totals
    stats.rssiSum += rssi_dbm;
    stats.snrSum += snr_db;
    
    // Update min/max
    if (rssi_dbm < stats.rssiMin) stats.rssiMin = rssi_dbm;
    if (rssi_dbm > stats.rssiMax) stats.rssiMax = rssi_dbm;
    if (snr_db < stats.snrMin) stats.snrMin = snr_db;
    if (snr_db > stats.snrMax) stats.snrMax = snr_db;
    
    // Check for packet loss (look for "Hello #X" pattern)
    uint32_t sequence = 0;
    if (sscanf((char*)payload, "Hello #%lu", &sequence) == 1) {
        if (!stats.firstPacket) {
            uint32_t expected = stats.lastSequence + 1;
            if (sequence > expected) {
                stats.packetsLost += (sequence - expected);
            }
        }
        stats.lastSequence = sequence;
        stats.firstPacket = false;
    }
    
    // Determine quality indicator
    const char* quality;
    const char* qualityBar;
    if (linkMargin >= MARGIN_EXCELLENT) {
        quality = "EXCELLENT";
        qualityBar = "████████████████████";
    } else if (linkMargin >= MARGIN_GOOD) {
        quality = "GOOD";
        qualityBar = "████████████████░░░░";
    } else if (linkMargin >= MARGIN_MARGINAL) {
        quality = "MARGINAL";
        qualityBar = "████████████░░░░░░░░";
    } else {
        quality = "POOR";
        qualityBar = "████████░░░░░░░░░░░░";
    }
    
    // Calculate packet loss percentage
    uint32_t totalExpected = stats.packetsReceived + stats.packetsLost;
    float lossPct = totalExpected > 0 ? (100.0 * stats.packetsLost / totalExpected) : 0.0;
    
    // Print real-time data
    Serial.println();
    Serial.printf("┌─ Packet #%lu ──────────────────────────────────────┐\n", stats.packetsReceived);
    Serial.printf("│ RSSI: %4d dBm  │  SNR: %3d dB  │  Margin: %2d dB │\n", 
                 rssi_dbm, snr_db, linkMargin);
    Serial.printf("│ Quality: %-9s  [%s] │\n", quality, qualityBar);
    Serial.printf("│ Packets: %lu received, %lu lost (%.1f%%)           │\n",
                 stats.packetsReceived, stats.packetsLost, lossPct);
    Serial.printf("└──────────────────────────────────────────────────┘\n");
    
    // CSV output for logging/plotting
    uint32_t elapsed = (millis() - stats.startTime) / 1000;
    Serial.printf("CSV,%lu,%d,%d,%d,%lu,%.1f\n", 
                 elapsed, rssi_dbm, snr_db, linkMargin, 
                 stats.packetsReceived, lossPct);
}

void printSummary() {
    if (stats.packetsReceived == 0) return;
    
    // Calculate averages
    float avgRssi = (float)stats.rssiSum / stats.packetsReceived;
    float avgSnr = (float)stats.snrSum / stats.packetsReceived;
    float avgMargin = avgRssi - SENSITIVITY_FLOOR;
    
    uint32_t totalExpected = stats.packetsReceived + stats.packetsLost;
    float lossPct = totalExpected > 0 ? (100.0 * stats.packetsLost / totalExpected) : 0.0;
    float reliability = 100.0 - lossPct;
    
    uint32_t elapsed = (millis() - stats.startTime) / 1000;
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║              LINK QUALITY SUMMARY                ║");
    Serial.println("╠══════════════════════════════════════════════════╣");
    Serial.printf("║ Duration:      %lu seconds                        \n", elapsed);
    Serial.printf("║ Packets:       %lu received, %lu lost              \n", 
                 stats.packetsReceived, stats.packetsLost);
    Serial.printf("║ CRC Errors:    %lu                                 \n", stats.crcErrors);
    Serial.printf("║ Reliability:   %.1f%%                              \n", reliability);
    Serial.println("╠══════════════════════════════════════════════════╣");
    Serial.printf("║ RSSI:   Avg: %.1f dBm  Min: %d  Max: %d           \n", 
                 avgRssi, stats.rssiMin, stats.rssiMax);
    Serial.printf("║ SNR:    Avg: %.1f dB   Min: %d  Max: %d           \n", 
                 avgSnr, stats.snrMin, stats.snrMax);
    Serial.printf("║ Margin: Avg: %.1f dB (above %d dBm floor)         \n", 
                 avgMargin, SENSITIVITY_FLOOR);
    Serial.println("╚══════════════════════════════════════════════════╝");
}
