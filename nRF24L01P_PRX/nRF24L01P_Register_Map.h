#ifndef NRF24L01P_REGISTER_MAP_H_
#define NRF24L01P_REGISTER_MAP_H_

// nRF24L01+ REGISTER MAP (Datasheet page 53 - ...): /////////////////////////////////////////////////////////////////////////////////////////////
//
#define REG_CONFIG 0x00 // Configuration register
#define CONFIG_MASK_RX_DR 6
#define CONFIG_MASK_TX_DS 5
#define CONFIG_MASK_MAX_RT 4
#define CONFIG_EN_CRC 3
#define CONFIG_CRCO 2
#define CONFIG_PWR_UP 1
#define CONFIG_PRIM_RX 0

#define REG_EN_AA 0x01 // Enable data pipes auto acknowledgement
#define EN_AA_P5 5
#define EN_AA_P4 4
#define EN_AA_P3 3
#define EN_AA_P2 2
#define EN_AA_P1 1
#define EN_AA_P0 0

#define REG_EN_RXADDR 0x02 // Enable data pipes RX address
#define EN_RXADDR_P5 5
#define EN_RXADDR_P4 4
#define EN_RXADDR_P3 3
#define EN_RXADDR_P2 2
#define EN_RXADDR_P1 1
#define EN_RXADDR_P0 0

#define REG_SETUP_AW 0x03     // Setup address width (common for all data pipes)
#define SETUP_AW_AW_REG_POS 0 // Bits[0-1]
#define AW_ILLEGAL 0x00       // Make sure that the Address Width is set because a value of 0x00 is illegal to use!
#define AW_3_BYTES 0x01
#define AW_4_BYTES 0x02
#define AW_5_BYTES 0x03
#define AW_MASK(aw_val) (((aw_val) & 0x03) << SETUP_AW_AW_REG_POS) // & the passed aw value with 0b0000 0011. This makes sure that only the first 2 bits can be set, even when a wrong value is passed,
                                                                   // however the result will be wrong in that case but all the other bits of the same register are protected from modification.

#define REG_SETUP_RETR 0x04 // Setup automatic retransmission
//
#define SETUP_RETR_ARD_REG_POS 4                                         // Bits[4-7]
#define ARD_WAIT_250us 0x00                                              // 0b0000 0000
#define ARD_WAIT_500us 0x10                                              // 0b0001 0000
#define ARD_WAIT_750us 0x20                                              // 0b0010 0000
#define ARD_WAIT_1000us 0x30                                             // 0b0011 0000
#define ARD_WAIT_1250us 0x40                                             // 0b0100 0000
#define ARD_WAIT_1500us 0x50                                             // 0b0101 0000
#define ARD_WAIT_1750us 0x60                                             // 0b0110 0000
#define ARD_WAIT_2000us 0x70                                             // 0b0111 0000
#define ARD_WAIT_2250us 0x80                                             // 0b1000 0000
#define ARD_WAIT_2500us 0x90                                             // 0b1001 0000
#define ARD_WAIT_2750us 0xA0                                             // 0b1010 0000
#define ARD_WAIT_3000us 0xB0                                             // 0b1011 0000
#define ARD_WAIT_3250us 0xC0                                             // 0b1100 0000
#define ARD_WAIT_3500us 0xD0                                             // 0b1101 0000
#define ARD_WAIT_3750us 0xE0                                             // 0b1110 0000
#define ARD_WAIT_4000us 0xF0                                             // 0b1111 0000
#define ARD_MASK(ard_val) (((ard_val) & 0xF0) << SETUP_RETR_ARD_REG_POS) // & the passed ard value with 0b1111 0000. This makes sure that only the last 4 bits can be set, even when a wrong value is passed.
//
#define SETUP_RETR_ARC_REG_POS 0                                         // Bits[0-3]
#define ARC_RETR_DISABLE 0x00                                            // 0b0000 0000
#define ARC_RETR_X1 0x01                                                 // 0b0000 0001
#define ARC_RETR_X2 0x02                                                 // 0b0000 0010
#define ARC_RETR_X3 0x03                                                 // 0b0000 0011
#define ARC_RETR_X4 0x04                                                 // 0b0000 0100
#define ARC_RETR_X5 0x05                                                 // 0b0000 0101
#define ARC_RETR_X6 0x06                                                 // 0b0000 0110
#define ARC_RETR_X7 0x07                                                 // 0b0000 0111
#define ARC_RETR_X8 0x08                                                 // 0b0000 1000
#define ARC_RETR_X9 0x09                                                 // 0b0000 1001
#define ARC_RETR_X10 0x0A                                                // 0b0000 1010
#define ARC_RETR_X11 0x0B                                                // 0b0000 1011
#define ARC_RETR_X12 0x0C                                                // 0b0000 1100
#define ARC_RETR_X13 0x0D                                                // 0b0000 1101
#define ARC_RETR_X14 0x0E                                                // 0b0000 1110
#define ARC_RETR_X15 0x0F                                                // 0b0000 1111
#define ARC_MASK(arc_val) (((arc_val) & 0x0F) << SETUP_RETR_ARC_REG_POS) // & the passed arc value with 0b0000 1111. This makes sure that only the first 4 bits can be set, even when a wrong value is passed.

#define REG_RF_CH 0x05 // Set the frequency channel that the nRF24L01+ operates on => RESEARCH: WHAT DOES THIS MEAN EXACTLY ???
// TO DO: RF_CH REQUIRES 7 BITS TO BE SET => FIRST RESEARCH HOW THIS FREQUENCY CHANNEL WORKS. IS THE CHANNEL ONE OF THE DIVISIONS OF THE TOTAL BANDWIDTH ?
#define RF_CH_RF_CH_REG_POS 0
#define RF_CH_RESET 0x02                                                    // THIS IS THE RESET VALUE FOR THE CHANNEL => OTHER CHANNELS NEED TO BE IMPLEMENTED
#define RF_CH_MASK(rf_ch_val) (((rf_ch_val) & 0x7F) << RF_CH_RF_CH_REG_POS) // & the passed rf channel value with 0b0111 1111. This makes sure that only the first 6 bits can be set, even when a wrong value is passed.

#define REG_RF_SETUP 0x06   // RF setup (antenna in/out power, low noise amplifier gain, ...)
#define RF_SETUP_PLL_LOCK 4 // Recommended to leave unchanged. The datasheet mentions it is only used in test.
#define RF_SETUP_RF_DR 3
#define RF_SETUP_RF_PWR_REG_POS 1 // Bits[1-2]
#define RF_PWR_MIN_18DBM 0x00
#define RF_PWR_MIN_12DBM 0x01
#define RF_PWR_MIN_6DBM 0x02
#define RF_PWR_MIN_0DBM 0x03
#define RF_PWR_MASK(rf_pwr_val) (((rf_pwr_val) & 0x06) << RF_SETUP_RF_PWR_REG_POS) // & the passed rf power value with 0b0000 0110. This makes sure that only bit 1 and 2 can be set, even when a wrong value is passed.
#define RF_SETUP_LNA_HCURR 0

#define REG_STATUS 0x07 // The status register is shifted out automaticaly on the MISO line in parallel with an SPI command sent over the MOSI line.
#define STATUS_RX_DR 6
#define STATUS_TX_DS 5
#define STATUS_MAX_RT 4
// TO DO / CREATE MACRO: RX_P_NO REQUIRES 3 BITS TO BE GET
#define STATUS_TX_FULL 0

#define REG_OBSERVE_TX 0x08 // Observe the lost packet count or the packet retransmission count
// TO DO / CREATE MACRO: PLOS_CNT REQUIRES 4 BITS TO BE GET
// TO DO / CREATE MACRO: ARC_CNT REQUIRES 4 BITS TO BE GET

#define REG_RPD 0x09 // Received Power Detector. This register is called CD (Carrier Detect) in the nRF24L01.
#define RPD_RPD 0

#define REG_RX_ADDR_P0 0x0A // Data pipe 0 receive address. Max 5byte length, depends on the value set in SETUP_AW. (LSB written first)
#define REG_RX_ADDR_P1 0x0B // Data pipe 1 receive address. Max 5byte length, depends on the value set in SETUP_AW. (LSB written first)
#define REG_RX_ADDR_P2 0x0C // Data pipe 2 receive address. Only the LSB can be defined, the other address bytes are shared with RX_ADDR_P1
#define REG_RX_ADDR_P3 0x0D // Data pipe 3 receive address. Only the LSB can be defined, the other address bytes are shared with RX_ADDR_P1
#define REG_RX_ADDR_P4 0x0E // Data pipe 4 receive address. Only the LSB can be defined, the other address bytes are shared with RX_ADDR_P1
#define REG_RX_ADDR_P5 0x0F // Data pipe 5 receive address. Only the LSB can be defined, the other address bytes are shared with RX_ADDR_P1

#define REG_TX_ADDR 0x10 // The primary transmitter (PTX) 5byte address (LSB written first).
                         // Set RX_ADDR_P0 of the PTX device to this address if auto acknowledgement is used to be able to receive the acknowledgment package from the primary receiver (PRX)

#define REG_RX_PW_P0 0x11 // The amount of bytes in the RX payload in data pipe 0. (0 - 32bytes => 0bytes = data pipe not used)
#define REG_RX_PW_P1 0x12 // The amount of bytes in the RX payload in data pipe 1. (0 - 32bytes => 0bytes = data pipe not used)
#define REG_RX_PW_P2 0x13 // The amount of bytes in the RX payload in data pipe 2. (0 - 32bytes => 0bytes = data pipe not used)
#define REG_RX_PW_P3 0x14 // The amount of bytes in the RX payload in data pipe 3. (0 - 32bytes => 0bytes = data pipe not used)
#define REG_RX_PW_P4 0x15 // The amount of bytes in the RX payload in data pipe 4. (0 - 32bytes => 0bytes = data pipe not used)
#define REG_RX_PW_P5 0x16 // The amount of bytes in the RX payload in data pipe 5. (0 - 32bytes => 0bytes = data pipe not used)

#define REG_FIFO_STATUS 0x17 // FIFO status flags. Reuse transmit packet and tx/rx fifo empty/full.
#define FIFO_STATUS_TX_REUSE 6
#define FIFO_STATUS_TX_FULL 5
#define FIFO_STATUS_TX_EMPTY 4
#define FIFO_STATUS_RX_FULL 1
#define FIFO_STATUS_RX_EMPTY 0

// IMPLEMENTING THIS LATER ON, THIS FUNCTIONALITY IS NOT NEEDED RIGHT NOW => FOR USING THIS FUNCTIONALITY, THE ACTIVATE COMMAND FOLLOWED BY A DATA BYTE 0x73 NEEDS TO BE SENT AND THE CORRESPONDING BITS NEED TO BE SET IN THE FEATURE REGISTER.
// #define REG_DYNPD 0x1C    // Enable dynamic payload length
// #define REG_FEATURE 0x1D  // Enable dynamic payload, enable PRX ack payload or enable PTX payload without PRX ack
//
#define REG_SET_BIT(x) (1 << (x))      // Create a bitmask for setting a single bit
#define REG_CLEAR_BIT(x) (~(1 << (x))) // Create a bitmask for clearing a single bit
// #define REG_MODIFY_BITS(reg, mask, value) (((reg) & ~(mask)) | ((value) & (mask))) // Create a bitmask for modifying multiple bits
#define REG_GET_BIT(reg, x) (((reg) >> (x)) & 1)

#endif // NRF24L01P_REGISTER_MAP_H_