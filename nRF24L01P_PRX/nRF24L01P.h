#ifndef NRF24L01P_H_
#define NRF24L01P_H_

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/spi.h"

#include "nRF24L01P_Data_And_Ctrl.h"
#include "nRF24L01P_Register_Map.h"

#define REG_SET_MASK_UNUSED 0x00
#define REG_CLEAR_MASK_UNUSED 0xFF

#define PTX_MAX_PAYLOAD_LENGTH 32

// isr_flag: Will be set true ('1') if the TX_DS, MAX_RT OR RX_DR bit in the STATUS register is set high ('1') which in its turn sets the IRQ pin low ('0').
// Make sure to keep a valid address for the isr_flag during the whole application. Setting it to NULL will result in a malfunctioning device.
void nRF24L01P_GPIO_Init(uint8_t pin_ce, uint8_t pin_irq, volatile bool *isr_flag);

static inline void nRF24L01_Start_Cmd(void)
{
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0); // Create a falling edge on the CSN pin to notify the nRF24L01 that a new command is to be sent.
}

static inline void nRF24L01_End_Cmd(void)
{
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1); // Pull up the CSN pîn after a new command is executed so that other new commands can be send.
}

void nRF24L01P_TX_Setup(void);

void nRF24L01P_RX_Setup(void);

// Returns the status register.
static inline uint8_t nRF24L01P_Read_Status_Reg(void)
{
    uint8_t status_reg = 0x00;

    // 1.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 2.) Send a no operation to the nRF24L01+ so it shifts out the status register.
    uint8_t nop = NOP;
    spi_write_read_blocking(spi_default, &nop, &status_reg, 1);

    // 3.) End the SPI transaction.
    nRF24L01_End_Cmd();

    return status_reg;
}

// IMPORTANT: The status register requires writing a '1' to the bit that needs to be cleared to '0'!
static inline void nRF24L01P_Clear_Status_Reg_IRQ_Flags(uint8_t status_reg_mask)
{
    uint8_t status_reg = 0x00;

    // 1. Mask the write command with the status register address.
    uint8_t reg_w_cmd = W_REGISTER | REG_STATUS;

    printf("Clearing status register IRQ flags: 0X%02X\r\n", status_reg_mask);

    // 2.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 3.) Send the write command to the nRF24L01P.
    // When a command is sent to the nRF24L01P, it will shift out the status register on the MISO line in parallel to the command that is sent. (Datasheet page 47)
    spi_write_read_blocking(spi_default, &reg_w_cmd, &status_reg, 1);

    // 4.) Write the required register content.
    spi_write_blocking(spi_default, &status_reg_mask, 1);

    // 5.) End the SPI transaction.
    nRF24L01_End_Cmd();

    printf("Clear complete.\r\n\n");
}

// Returns the 1byte register content. Note: The actual register address is only 5 bit wide.
uint8_t nRF24L01P_Read_Reg(uint8_t reg_addr);

// Note: The actual register address is only 5 bit wide.
//       set_mask: All bits that are set to '1' in the mask will be set to '1' in the register. Pass 0x00 when set_mask is not used.
//       clear_mask: All bits that are set to '0' in the mask will be set to '0' in the register. Pass 0xFF when clear_mask is not used.
void nRF24L01P_Write_Reg(uint8_t reg_addr, uint8_t set_mask, uint8_t clear_mask);

// Set the TX or RX address.
void nRF24L01P_Set_Addr(uint8_t reg_addr, uint8_t *address, uint8_t addr_len);

// Get the TX or RX address.
void nRF24L01P_Get_Addr(uint8_t reg_addr, uint8_t *address, uint8_t addr_len);

// Print a TX or RX address.
void nRF24L01P_Print_Addr(uint8_t *address, uint8_t addr_len);

// Max payload size is 32bytes.
void nRF24L01P_Write_TX_Payload(uint8_t *payload, uint8_t payload_len);

// Pass a 32byte buffer for now, this to make sure that the received payload will fit into the buffer. The actual payload length will be returned.
uint8_t nRF24L01P_Read_RX_Payload(uint8_t status_register, uint8_t payload[32]);

// Reuse the current payload to be sent over and over again. This also means that the payload will not be cleared from the TX FIFO when the sending to the PRX succeeded.
// Important: TX payload reuse is active until the W_TX_PAYLOAD or FLUSH TX command is executed. This also means that the payload that is reused must be written before executing the REUSE_TX_PL command.
void nRF24L01P_Reuse_TX_Payload(void);

// The nRF24L01 TX FIFO needs to contain data before this send function can be called!
void nRF24L01P_PTX_Send(uint8_t ce_pin);

void nRF24L01P_PRX_Recv(uint8_t ce_pin);

// Nordic Semiconductor nRF24L01+ Enhanced ShockBurst™ Transmitting Payload (PTX) example: (Datasheet page 75) /////////////////////////////////////////////////////////////
//
// 1. The configuration bit PRIM_RX has to be low.
//
// 2. When the application MCU has data to transmit, the address for the receiving node (TX_ADDR)
// and payload data (TX_PLD) has to be clocked into nRF24L01+ through the SPI. The width of TX payload
// is counted from number of bytes written into the TX FIFO from the MCU. TX_PLD must
// be written continuously while holding CSN low. TX_ADDR does not have to be rewritten if it is
// unchanged from last transmit. If the PTX device shall receive an acknowledgement, data pipe 0 has to be
// configured to receive the ACK packet from the PRX. The RX address for data pipe 0 (RX_ADDR_P0) has to be
// equal to the TX address (TX_ADDR) in the PTX device. For the datasheet example in Figure 12. on page 37
// the following address settings have to be performed for the TX5 device and the RX device:
// TX5 device: TX_ADDR = 0xB3B4B5B605
// TX5 device: RX_ADDR_P0 = 0xB3B4B5B605
// RX device: RX_ADDR_P5 = 0xB3B4B5B605
//
// 3. A high pulse on CE starts the transmission. The minimum pulse width on CE is 10μs.
//
// 4. nRF24L01+ ShockBurst™:
// 􀁘 Radio is powered up.
// 􀁘 16MHz internal clock is started.
// 􀁘 RF packet is completed (see the packet description).
// 􀁘 Data is transmitted at high speed (1Mbps or 2Mbps configured by MCU).
//
// 5. If auto acknowledgement is activated (ENAA_P0=1) the radio goes into RX mode immediately,
// unless the NO_ACK bit is set in the received packet. If a valid packet has been received in the
// valid acknowledgement time window, the transmission is considered a success. The TX_DS bit in
// the STATUS register is set high and the payload is removed from TX FIFO. If no valid ACK packet
// is received in the specified time window, the payload is retransmitted (if auto retransmit is
// enabled). If the auto retransmit counter (ARC_CNT) exceeds the programmed maximum limit
// (ARC), the MAX_RT bit in the STATUS register is set high. The payload in TX FIFO is NOT
// removed. The IRQ pin is active when MAX_RT or TX_DS is high. To turn off the IRQ pin, the interrupt
// source must be reset by writing to the STATUS register (see Interrupt chapter). If no ACK
// packet is received for a packet after the maximum number of retransmits, no further packets can
// be transmitted before the MAX_RT interrupt is cleared. The packet loss counter (PLOS_CNT) is
// incremented at each MAX_RT interrupt. That is, ARC_CNT counts the number of retransmits that
// was required to get a single packet through. PLOS_CNT counts the number of packets that did not
// get through after maximum number of retransmits.
//
// 6. nRF24L01+ goes into standby-I mode if CE is low. Otherwise the next payload in TX FIFO is transmitted.
// If TX FIFO is empty and CE is still high, nRF24L01+ enters standby-II mode.
//
// 7. If nRF24L01+ is in standby-II mode, it goes to standby-I mode immediately if CE is set low.
//
// Nordic Semiconductor nRF24L01+ Enhanced ShockBurst™ Transmitting Payload example: ///////////////////////////////////////////////////////////////////////////////////////

// Nordic Semiconductor nRF24L01+ Enhanced ShockBurst™ Receiving Payload (PRX) example: (Datasheet page 76) ////////////////////////////////////////////////////////////////
//
// 1. Select RX by setting the PRIM_RX bit in the CONFIG register to high. All data pipes that receive
// data must be enabled (EN_RXADDR register), enable auto acknowledgement for all pipes running
// Enhanced ShockBurst™ (EN_AA register), and set the correct payload widths (RX_PW_Px registers).
// Set up addresses as described in item 2 in the Enhanced ShockBurst™ transmitting payload
// example above.
//
// 2. Start Active RX mode by setting CE high.
//
// 3. After 130μs nRF24L01+ monitors the air for incoming communication.
//
// 4. When a valid packet is received (matching address and correct CRC), the payload is stored in the
// RX-FIFO, and the RX_DR bit in STATUS register is set high. The IRQ pin is active when RX_DR is
// high. RX_P_NO in STATUS register indicates what data pipe the payload has been received in.
//
// 5. If auto acknowledgement is enabled, an ACK packet is transmitted back, unless the NO_ACK bit
// is set in the received packet. If there is a payload in the TX_PLD FIFO, this payload is added to
// the ACK packet.
//
// 6. MCU sets the CE pin low to enter standby-I mode (low current mode).
//
// 7. MCU can clock out the payload data at a suitable rate through the SPI.
//
// 8. nRF24L01+ is now ready for entering TX or RX mode or power down mode.
//
// Nordic Semiconductor nRF24L01+ Enhanced ShockBurst™ Receiving Payload (PRX) example: //////////////////////////////////////////////////////////////////////////////////

#endif // NRF24L01P_H_