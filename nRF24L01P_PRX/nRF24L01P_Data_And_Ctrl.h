#ifndef NRF24L01P_DATA_AND_CONTROL_INTERFACE_H_
#define NRF24L01P_DATA_AND_CONTROL_INTERFACE_H_

// nRF24L01+ DATA AND CONTROL INTERFACE (Datasheet page 45 - ...): ///////////////////////////////////////////////////////////////////////////////
//
// Important:
//            - The max SPI speed is 8Mbps
//            - The nRF24L01+ must be in one of the standby modes or in power down mode before writing to the configuration registers. (Datasheet SPI timing page 52)
//            - You can use the SPI to activate the nRF24L01+ data FIFOs or the register map by 1 byte SPI commands during ALL modes of operation. (Datasheet data and control interface page 50)
//            - Each NEW command needs to be preceeded by a falling edge on the CSN pin to alert the nRF24L01+ that an SPI command will be sent. (Datasheet SPI commands page 50)
//            - Reading and writing from registers happens in the order: MSbit of LSByte first.
//            - In parallel to the SPI command word applied on the MOSI pin, the STATUS register is shifted out on the MISO pin. (Datasheet SPI commands page 50)

// R_REGISTER: READ from register
// Command: 0b000A AAAA => Fill in the 5bit register address from which will be read instead of the A's.
// Data: 1 to 5 data bytes (LSB first) will be read from the nRF24L01+ depending on the register that is read.
#define R_REGISTER 0x00 // Default address to mask with the actual 5bit register address.

// W_REGISTER: WRITE to register
// Command: 0b001A AAAA => Fill in the 5bit register address to which will be written instead of the A's.
// Data: Follow the command byte with 1 to 5 data bytes (LSB first) depending on the register that is written to.
#define W_REGISTER 0x20 // Default address to mask with the actual 5bit register address.

// R_RX_PAYLOAD: READ the received payload when the device acts as the primary receiver (PRX)
// Command: 0b0110 0001
// Data: 1 to 32bytes will be read from the RX FIFO (LSB first) depending on the received payload length. The Payload is deleted from the FIFO after it is read.
#define R_RX_PAYLOAD 0x61

// W_TX_PAYLOAD: WRITE the to be sent payload when the device acts as the primary transmitter (PTX)
// Command: 0b1010 0000
// Data: 1 to 32bytes can be written to the TX FIFO (LSB first).
#define W_TX_PAYLOAD 0xA0

// IMPLEMENTING THESE LATER ON AFTER THE FIRST TESTS.
// FLUSH_TX: 0b1110 0001
// FLUSH_RX: 0b1110 0010

// REUSE_TX_PL: Reuse the previously sent payload for the following transmits. This allows repeatedly sending of the same payload when the device acts as PTX as long as the CE pin stays high.
//              REUSE_TX_PL stays active as long as the current payload isn't changed due to executing the W_TX_PAYLOAD or FLUSH TX commands.
// Command: 0b1110 0011
// Data: No data bytes to send/receive
#define REUSE_TX_PL 0xE3

// FOLLOWING COMMANDS REQUIRE TO SET CORRESPONDING BITS IN THE FEATURE REGISTER (Datasheet page 51)
// NOTE ACTIVATE IS A COMMAND THAT IS ONLY FOUND ON THE NRF24L01 AND NOT ON THE NRF24L01+!
// #define ACTIVATE: 0101 0000 followed by the data byte: 0x73 => This is used to disable auto acknowledgment on packet receive by the PRX or sending a payload in the ack packet when acting as PRX.
// ACTIVATE must be executed before the following registers can be used:
#define R_RX_PL_WID 0x60 // 0b0110 0000
// #define W_ACK_PAYLOAD:      0b1010 1PPP
// #define W_TX_PAYLOAD_NOACK: 0b1011 0000

// NOP: This command can be used to perform a No Operation on the nRF24L01+ and shift out the status register on the MISO line.
// If the status register is read by using a register read command with the STATUS register address, the STATUS register would be
// shifted out in parallel to the command, and it will be sent a second time because of the read command.
// This would require an unnecesarry read to get the status register (and this second read is a MUST, because the SPI RX FIFO needs to be cleared).
// Command: 0b1111 1111
// Data: The status register will be shifted out on the MISO line, just like it happens with all other commands.
#define NOP 0xFF

#endif // NRF24L01P_DATA_AND_CONTROL_INTERFACE_H_