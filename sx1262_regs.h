#ifndef SX1262_REGS_H
#define SX1262_REGS_H

#include <stdint.h>

// ============ SPI OPCODES ============
#define SX1262_OP_SET_SLEEP              0x84
#define SX1262_OP_SET_STANDBY            0x80
#define SX1262_OP_SET_FS                 0xC1
#define SX1262_OP_SET_TX                 0x83
#define SX1262_OP_SET_RX                 0x82
#define SX1262_OP_SET_CAD                0xC5
#define SX1262_OP_CALIBRATE              0x89
#define SX1262_OP_CALIBRATE_IMAGE        0x98
#define SX1262_OP_SET_PA_CONFIG          0x95
#define SX1262_OP_SET_REGULATOR_MODE     0x96
#define SX1262_OP_SET_TX_PARAMS          0x8E
#define SX1262_OP_SET_MODULATION_PARAMS  0x8B
#define SX1262_OP_SET_PACKET_TYPE        0x8A
#define SX1262_OP_SET_PACKET_PARAMS      0x8C
#define SX1262_OP_SET_RF_FREQUENCY       0x86
#define SX1262_OP_SET_DIO_IRQ_PARAMS     0x08
#define SX1262_OP_SET_BUFFER_BASE_ADDR   0x8F
#define SX1262_OP_WRITE_BUFFER           0x0E
#define SX1262_OP_READ_BUFFER            0x1E
#define SX1262_OP_WRITE_REGISTER         0x0D
#define SX1262_OP_READ_REGISTER          0x1D
#define SX1262_OP_GET_STATUS             0xC0
#define SX1262_OP_GET_IRQ_STATUS         0x12
#define SX1262_OP_CLEAR_IRQ_STATUS       0x02
#define SX1262_OP_CLEAR_DEVICE_ERRORS    0x07
#define SX1262_OP_GET_RX_BUFFER_STATUS   0x13
#define SX1262_OP_GET_PACKET_TYPE        0x03
#define SX1262_OP_GET_PACKET_STATUS      0x14
#define SX1262_SET_DIO3_AS_TCXO_CTRL     0x97
#define SX1262_STDBY_RC                  0x00
#define SX1262_STDBY_XOSC                0x01

// ============ REGISTER ADDRESSES (16-bit) ============
#define REG_CHIP_VERSION                 0x0320  // Should read 0x04 for SX1262
#define REG_IRQ_STATUS                   0x0600  // 16-bit IRQ status
#define REG_IRQ_MASK                     0x0604  // IRQ mask
#define REG_DIO1_IRQ_MASK                0x0608  // DIO1 IRQ mask
#define REG_DIO2_IRQ_MASK                0x0609  // DIO2 IRQ mask
#define REG_DIO3_IRQ_MASK                0x060A  // DIO3 IRQ mask
#define REG_PACKET_TYPE                  0x08A0  // Packet type (0x01=LoRa)
#define REG_RF_FREQUENCY_MSB             0x086B  // RF frequency MSB
#define REG_RF_FREQUENCY_MID             0x086C  // RF frequency MID
#define REG_RF_FREQUENCY_LSB             0x086D  // RF frequency LSB
// #define REG_TX_POWER                     0x08D0  // TX power (dBm) Does not exist. Use SetTxParams opcode instead
#define REG_TX_RAMP_TIME                 0x08D1  // TX ramp time
#define REG_PA_CONFIG                    0x0951  // PA configuration
#define REG_REGULATOR_MODE               0x08F0  // Regulator mode (LDO/DC-DC)
#define REG_LR_MODEM_CONFIG_1            0x0900  // LoRa modem config 1 (SF, BW)
#define REG_LR_MODEM_CONFIG_2            0x0901  // LoRa modem config 2 (CR)
#define REG_LR_MODEM_CONFIG_3            0x0902  // LoRa modem config 3 (AGC)
#define REG_RSSI_INSTANT                 0x08D0  // Instant RSSI
#define REG_SNR                          0x08D3  // SNR value
#define REG_PACKET_RSSI                  0x08D4  // Packet RSSI
#define REG_STANDBY_CONFIG               0x0902  // Standby config (RC/XOSC)
#define REG_BW500_OPTIMIZATION           0x0925

// ============ IRQ FLAGS (16-bit) ============
#define IRQ_TX_DONE                      (1 << 0)
#define IRQ_RX_DONE                      (1 << 1)
#define IRQ_PREAMBLE_DETECTED            (1 << 2)
#define IRQ_SYNCWORD_VALID               (1 << 3)
#define IRQ_HEADER_VALID                 (1 << 4)
#define IRQ_HEADER_ERROR                 (1 << 5)
#define IRQ_CRC_ERROR                    (1 << 6)
#define IRQ_CAD_DONE                     (1 << 7)
#define IRQ_CAD_DETECTED                 (1 << 8)
#define IRQ_RX_TX_TIMEOUT                (1 << 9)
#define IRQ_LR_FHSS_HOP                  (1 << 10)

// ============ PACKET TYPES ============
#define PACKET_TYPE_FSK                  0x00
#define PACKET_TYPE_LORA                 0x01

// ============ STANDBY MODES ============
#define STDBY_RC                         0x00  // RC13M oscillator
#define STDBY_XOSC                       0x01  // 32 MHz crystal

// ============ REGULATOR MODES ============
#define REGULATOR_MODE_LDO               0x00
#define REGULATOR_MODE_DC_DC             0x01

// ============ CALIBRATION BITS ============
#define CALIBRATE_IMAGE                  (1 << 0)
#define CALIBRATE_ADC_PULSE              (1 << 1)
#define CALIBRATE_ADC_BULK_P             (1 << 2)
#define CALIBRATE_ADC_BULK_N             (1 << 3)
#define CALIBRATE_PLL                    (1 << 4)
#define CALIBRATE_RC13M                  (1 << 5)
#define CALIBRATE_RC64K                  (1 << 6)
#define CALIBRATE_ALL                    0xFF

// ============ STATUS BYTE BIT LAYOUT ============
// Status byte returned on MISO for every command
// [7:4] IRQ status bits
// [3:0] Command status (0x0F = OK)
#define STATUS_COMMAND_OK                0x0F
#define STATUS_COMMAND_TIMEOUT           0x0E
#define STATUS_CMD_BITS_MASK             0x0F
#define STATUS_IRQ_BITS_MASK             0xF0

#endif // SX1262_REGS_H