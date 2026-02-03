/**
 * ============================================================================
 * SelfTest.ino - Hardware Connectivity Test
 * ============================================================================
 * SX1262 LoRa P2P Driver Suite
 * 
 * Run this sketch FIRST to verify your hardware wiring before attempting
 * radio operations. It tests:
 *   - BUSY pin readability
 *   - SPI communication
 *   - Chip identification
 *   - Reset functionality
 *   - DIO1 pin readability
 *   - Standby command execution
 * 
 * SETUP:
 *   1. Edit config.h and uncomment your board preset
 *   2. Upload this sketch
 *   3. Open Serial Monitor at 115200 baud
 *   4. Check test results
 * 
 * ============================================================================
 */

#include <SX1262_LoRa.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait for Serial (with timeout)
    
    Serial.println();
    Serial.println("================================================");
    Serial.println("  SX1262 Hardware Self-Test");
    Serial.println("================================================");
    Serial.printf("Board: %s\n", BOARD_NAME);
    Serial.println();
    
    // Step 1: Enable board power (required for Heltec, no-op for others)
    Serial.println("[1/2] Enabling board power...");
    hal_board_power_init();
    delay(100);  // Allow power to stabilize
    Serial.println("      Done.\n");
    
    // Step 2: Run hardware self-test
    Serial.println("[2/2] Running hardware self-test...\n");
    
    sx1262_self_test_result_t test;
    sx1262_result_t result = sx1262_self_test(&test);
    
    // Print detailed results
    Serial.println("Test Results:");
    Serial.println("─────────────────────────────────────────────────");
    Serial.printf("  BUSY pin readable:    %s\n", test.busy_pin_readable ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  BUSY pin responsive:  %s\n", test.busy_pin_responsive ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  SPI communication:    %s\n", test.spi_communication ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  Chip responding:      %s\n", test.chip_responding ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  Chip ID valid:        %s\n", test.chip_id_valid ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  Reset working:        %s\n", test.reset_working ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  DIO1 pin readable:    %s\n", test.dio1_pin_readable ? "✓ PASS" : "✗ FAIL");
    Serial.printf("  Standby command:      %s\n", test.standby_command ? "✓ PASS" : "✗ FAIL");
    Serial.println("─────────────────────────────────────────────────");
    
    if (test.spi_communication) {
        Serial.println("\nDiagnostic Data:");
        Serial.printf("  Status byte:        0x%02X\n", test.status_byte);
        Serial.printf("  Chip mode:          %d (2=STDBY_RC, 3=STDBY_XOSC)\n", test.chip_mode);
        Serial.printf("  Command status:     %d\n", test.command_status);
        Serial.printf("  BUSY response:      %lu µs\n", test.busy_response_us);
    }
    
    Serial.println();
    Serial.println("================================================");
    if (result == SX1262_OK) {
        Serial.println("  ✓ ALL TESTS PASSED - Hardware OK!");
        Serial.println("  You can now run the other examples.");
    } else {
        Serial.println("  ✗ TESTS FAILED");
        Serial.printf("  Reason: %s\n", test.failure_reason);
        Serial.println();
        Serial.println("  Troubleshooting:");
        if (!test.busy_pin_readable) {
            Serial.println("  - Check BUSY pin wiring");
            Serial.printf("    Expected GPIO: %d\n", PIN_SX1262_BUSY);
        }
        if (!test.spi_communication) {
            Serial.println("  - Check SPI wiring (MOSI, MISO, SCK, CS)");
            Serial.printf("    MOSI: GPIO %d\n", PIN_SX1262_MOSI);
            Serial.printf("    MISO: GPIO %d\n", PIN_SX1262_MISO);
            Serial.printf("    SCK:  GPIO %d\n", PIN_SX1262_SCK);
            Serial.printf("    CS:   GPIO %d\n", PIN_SX1262_CS);
        }
        if (!test.reset_working) {
            Serial.println("  - Check RESET pin wiring");
            Serial.printf("    Expected GPIO: %d\n", PIN_SX1262_NRESET);
        }
        if (!test.dio1_pin_readable) {
            Serial.println("  - Check DIO1 pin wiring");
            Serial.printf("    Expected GPIO: %d\n", PIN_SX1262_DIO1);
        }
    }
    Serial.println("================================================");
}

void loop() {
    // Nothing to do - self-test runs once
    delay(10000);
}
