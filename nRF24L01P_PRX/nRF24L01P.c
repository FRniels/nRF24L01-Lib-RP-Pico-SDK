#include "nRF24L01P.h"

volatile bool *isr_flag_ref = NULL;

void ISR_nRF24L01P(uint gpio, uint32_t events)
{
    // printf("irq\r\n"); // test

    if (gpio == 22) // TO DO: THE PIN MACRO NEEDS TO BE ACCESSIBLE HERE
    {
        // printf("Falling edge detected on GPIO 22!\n");
        *isr_flag_ref = true;
    }
}

void nRF24L01P_GPIO_Init(uint8_t pin_ce, uint8_t pin_irq, volatile bool *isr_flag)
{
    // Initialize the CE pin and set it's state to low as default.
    gpio_init(pin_ce);              // Initialize the CE pin
    gpio_set_dir(pin_ce, GPIO_OUT); // Configure as output
    gpio_put(pin_ce, 0);            // Set LOW (idle state)

    // Initialize the IRQ pin and the ISR
    if (isr_flag)
    {
        isr_flag_ref = isr_flag;

        gpio_init(pin_irq);                                                                    // Initialize the IRQ pin
        gpio_set_dir(pin_irq, GPIO_IN);                                                        // Configure as output
        gpio_pull_up(pin_irq);                                                                 // The IRQ pin is Active low: Enable pull-up resistor
        gpio_set_irq_enabled_with_callback(pin_irq, GPIO_IRQ_EDGE_FALL, true, &ISR_nRF24L01P); // Set up interrupt on the falling edge
    }
    else
    {
        printf("Failed to init the nRF24L01P IRQ. isr_flag can not be NULL!\r\n");
    }
}

void nRF24L01P_TX_Setup(void)
{
    // IMPORTANT: EACH NEW COMMAND SHOULD BE ANNOUNCED WITH A FALLING EDGE ON THE CSN LINE!
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1); // Setting the CSN pin high before sending the first command. This is to be sure that the CSN pin is high to create a falling edge.

    // 1.) CONFIG REGISTER:
    // - POWER UP THE DEVICE BY WRITING '1' TO THE PWR_UP BIT (BIT 1).
    // - THE CONFIGURATION BIT PRIM_RX HAS TO BE LOW ('0') TO CONFIGURE THE DEVICE AS A PRIMARY TRANSMITTER (PTX).
    // nRF24L01P_Write_Reg() // TO DO: EXECUTE COMMAND

    // 2.) SET THE TX ADDRESS OF THE DEVICE/PIPE WHERE THE DATA WILL BE SENT TO.

    // 3.) ENABLE THE AUTO ACKNOWLEDGE FUNCTIONALITY. THIS AUTOMATICALLY PUTS THE RADIO IN RECEIVE MODE AFTER SENDING A PACKET AND THE ACKOWLEDGEMENT IS NOT DISABLED.

    // 4.) IF THE PTX DEVICE SHALL RECEIVE AN ACKNOWLEDGEMENT, DATA PIPE 0 HAS TO BE CONFIGURED TO RECEIVE THE ACK PACKET FROM THE PRX.
    // THE RX ADDRESS FOR DATA PIPE 0 (RX_ADDR_P0) HAS TO BE EQUAL TO THE TX ADDRESS (TX_ADDR) IN THE PTX DEVICE.

    // 5.) ENABLE AUTO RETRANSMIT. THIS RETRANSMITS THE CURRENT PACKET IF NO ACKNOWLEDGEMENT IS RECEIVED BY THE PTX, AS LONG AS THE AUTO RETRANSMIT COUNTER VALUE (ARC_CNT)
    //     DOES NOT EQUAL THE CONFIGURED VALUE IN THE AUTO RETRANSMIT COUNT (ARC) REGISTER WHEN FAILING TO RECEIVE AN ACKNOWLEDGEMENT.

    // 6.) CONFIGURE THE MAX AUTO RETANSMIT COUNT.

    // 7.) Check the FIFO_STATUS register if the TX FIFO is full or empty. => TO DO: USE THIS AS A TEST TO SEE IF IT RETURNS EMPTY BEFORE WRITING THE FIRST TX PAYLOAD LIKE IT SHOULD.

    // 8.) WRITE THE PAYLOAD TO SEND TO THE PRX: THE TX_PLD REGISTER MUST BE WRITTEN CONTINUOUSLY WHILE HOLDING CSN LOW. THE AMOUNT OF BYTES IS AUTOMATICALLY COUNTED WHEN THEY ARE SENT OVER SPI TO THE nRF24L01 TX FIFO.
    // W_TX_PAYLOAD (1-32bytes)

    // 9.) Check the FIFO_STATUS register if the TX FIFO is full or empty. => TO DO: USE THIS AS A TEST TO SEEF IT THIS NOT EMPTY AFTER WRITING THE FIRST TX PAYLOAD.

    // 10.) REUSE THE CURRENT TX PAYLOAD AS A TEST
    // REUSE_TX_PL
}

void nRF24L01P_RX_Setup(void)
{
    // NOT YET IMPLEMENTED
}

// Note: The actual register address is only 5 bit wide.
uint8_t nRF24L01P_Read_Reg(uint8_t reg_addr)
{
    // 1.) Mask the read command with the required register address to read from.
    uint8_t reg_r_cmd = R_REGISTER | (reg_addr & 0x1F); // & the passed register address with 0b0001 1111 to make sure that only the first 5bits can be set because the registers only have a 5bit address.
    uint8_t status_reg = 0x00;                          // NOTE: Not using the status register at the moment, but just reading it so it is cleared out of the Pico SPI RX buffer.
    uint8_t reg_data = 0x00;

    printf("Reading reg_addr 0X%02X with read command 0X%02X => content: ", (reg_addr & 0x1F), reg_r_cmd);

    // 2.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 3.) Send the read command to the nRF24L01.
    // When a command is sent to the nRF24L01, it will shift out the status register on the MISO line in parallel to the command that is sent. (Datasheet page 47)
    spi_write_read_blocking(spi_default, &reg_r_cmd, &status_reg, 1);

    // 4.) Read the received register content.
    spi_read_blocking(spi_default, NOP, &reg_data, 1);

    // 5.) End the SPI transaction.
    nRF24L01_End_Cmd();

    printf("0X%02X\r\n", reg_data);
    printf("Status register content: 0X%02X\r\n\n", status_reg);

    return reg_data;
}

// Note: The actual register address is only 5 bit wide.
void nRF24L01P_Write_Reg(uint8_t reg_addr, uint8_t set_mask, uint8_t clear_mask)
{
    printf("Write value to register:\r\n");

    uint8_t status_reg = 0x00; // NOTE: Not using the status register at the moment, but just reading it so it is cleared out of the Pico SPI RX buffer.

    // 1.) Read the current register contents.
    uint8_t reg_data = nRF24L01P_Read_Reg(reg_addr);

    // 2.) Mask the current register bits to set or clear the required bits.
    printf("set_mask: 0X%02X, clear_mask: 0x%02X\r\n", set_mask, clear_mask);
    reg_data |= set_mask;   // Set all bits that are set to '1' in the set_mask.
    reg_data &= clear_mask; // Clear all bits that are set to '0' in the set_mask.
    printf("Original register content masked: 0X%02X\r\n", reg_data);

    // 3. Mask the write command with the required register address to write too.
    uint8_t reg_w_cmd = W_REGISTER | (reg_addr & 0x1F); // Mask the default 0x20 address with the required register address.
                                                        // & the passed register address with 0b0001 1111 to make sure that only the first 5bits can be set because the registers only have a 5bit address.

    printf("Writing new register content to: reg_addr 0X%02X with write command 0X%02X\r\n", (reg_addr & 0x1F), reg_w_cmd);

    // 4.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 5.) Send the write command to the nRF24L01P.
    // When a command is sent to the nRF24L01P, it will shift out the status register on the MISO line in parallel to the command that is sent. (Datasheet page 47)
    spi_write_read_blocking(spi_default, &reg_w_cmd, &status_reg, 1); // Send a write command to the nRF24L01P and read the status register.

    // 6.) Write the required register content.
    spi_write_blocking(spi_default, &reg_data, 1); // Send the modified register value back to the nRF24L01P.

    // 7.) End the SPI transaction.
    nRF24L01_End_Cmd();

    printf("Status register content: 0X%02X\r\n", status_reg);
    printf("Write complete.\r\n\n");
}

void nRF24L01P_Set_Addr(uint8_t reg_addr, uint8_t *address, uint8_t addr_len)
{
    printf("Write address of length %d to register 0X%02X:\r\n", addr_len, (reg_addr & 0x1F));

    uint8_t status_reg = 0x00; // NOTE: Not using the status register at the moment, but just reading it so it is cleared out of the Pico SPI RX buffer.

    // 1. Mask the write command with the required register address to write too.
    uint8_t reg_w_cmd = W_REGISTER | (reg_addr & 0x1F);

    // 2.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 3.) Send the write command to the nRF24L01P and read the status register.
    spi_write_read_blocking(spi_default, &reg_w_cmd, &status_reg, 1);

    // 4.) Write the address to the nRF24L01P.
    spi_write_blocking(spi_default, address, addr_len);

    // 5.) End the SPI transaction.
    nRF24L01_End_Cmd();
}

void nRF24L01P_Get_Addr(uint8_t reg_addr, uint8_t *address, uint8_t addr_len)
{
    printf("Read %dbyte address at 0X%02X:\r\n", addr_len, (reg_addr & 0x1F));

    // 1.) Mask the read command with the required register address to read from.
    uint8_t reg_r_cmd = R_REGISTER | (reg_addr & 0x1F);
    uint8_t status_reg = 0x00; // NOTE: Not using the status register at the moment, but just reading it so it is cleared out of the Pico SPI RX buffer.

    // 2.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 3.) Send the read command to the nRF24L01P.
    spi_write_read_blocking(spi_default, &reg_r_cmd, &status_reg, 1);

    // 4.) Read the received register content (address).
    spi_read_blocking(spi_default, NOP, address, addr_len);

    // 5.) End the SPI transaction.
    nRF24L01_End_Cmd();
}

void nRF24L01P_Print_Addr(uint8_t *address, uint8_t addr_len)
{
    printf("0X");
    for (uint8_t i = 0; i < addr_len; ++i)
    {
        printf("%02X", address[i]);
    }
    printf("\r\n\n");
}

void nRF24L01P_Write_TX_Payload(uint8_t *payload, uint8_t payload_len)
{
    uint8_t tx_payload_w_cmd = W_TX_PAYLOAD;
    uint8_t status_reg = 0x00; // NOTE: Not using the status register at the moment, but just reading it so it is cleared out of the Pico SPI RX buffer.

    // 1.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 2.) Send the write tx payload command to the nRF24L01P and read the status register.
    spi_write_read_blocking(spi_default, &tx_payload_w_cmd, &status_reg, 1);

    // 3.) Write the tx payload to the nRF24L01P.
    spi_write_blocking(spi_default, payload, payload_len);

    // 4.) End the SPI transaction.
    nRF24L01_End_Cmd();
}

uint8_t nRF24L01P_Read_RX_Payload(uint8_t status_register, uint8_t payload[32])
{
    // 1.) Read the status register RX_P_NO bits[1-3] to know which data pipe has received data.
    // Get the data pipe num by masking the status register with & 0b0000 1110.
    // Right Shift the masked status register 1bit to make it easier to get the data pipe address's: the range will be 0-5 after shifting, just like the actual data pipe numbers.
    uint8_t data_pipe_num = ((status_register & 0x0E) >> 1); // After the right shift: 0b0110: not used, 0b0111: RX FIFO is empty
    uint8_t payload_width = 0;

    if (data_pipe_num < 6)
    {
        // 2.) Start the SPI transaction.
        nRF24L01_Start_Cmd();

        // 3.) Get the width of the payload that is at the top of the RX FIFO
        uint8_t rx_pl_wid_cmd = R_RX_PL_WID;
        spi_write_read_blocking(spi_default, &rx_pl_wid_cmd, &status_register, 1);
        spi_read_blocking(spi_default, NOP, &payload_width, 1);
        printf("Received payload width: %d\r\n", payload_width);

        // 4.) End the SPI transaction.
        nRF24L01_End_Cmd();

        // 5.) Start the SPI transaction.
        nRF24L01_Start_Cmd();

        // 6.) Get the received payload
        uint8_t rx_payload_r_cmd = R_RX_PAYLOAD;
        spi_write_read_blocking(spi_default, &rx_payload_r_cmd, &status_register, 1);
        spi_read_blocking(spi_default, NOP, payload, payload_width);

        // 7.) End the SPI transaction.
        nRF24L01_End_Cmd();
    }
    else
    {
        // 8.) 110 is an unused value for the RX_P_NO bits in the status register, so the expected value in this else case is 111, which means the RF FIFO is empty.
        printf("RX FIFO is empty.\r\n");
    }

    return payload_width;
}

void nRF24L01P_Reuse_TX_Payload(void)
{
    uint8_t reuse_tx_payload_cmd = REUSE_TX_PL;
    uint8_t status_reg = 0x00; // NOTE: Not using the status register at the moment, but just reading it so it is cleared out of the Pico SPI RX buffer.

    // 1.) Start the SPI transaction.
    nRF24L01_Start_Cmd();

    // 2.) Send the reuse tx payload command to the nRF24L01P and read the status register.
    spi_write_read_blocking(spi_default, &reuse_tx_payload_cmd, &status_reg, 1);

    // 3.) End the SPI transaction.
    nRF24L01_End_Cmd();
}

void nRF24L01P_PTX_Send(uint8_t ce_pin)
{
    // 1.) A minimum high pulse of 10μs on the CE pin is needed to start the transmission.
    gpio_put(ce_pin, 1);
    sleep_us(10);
    gpio_put(ce_pin, 0);

    // 2.) After the received acknowledgement, the nRF24L01+ goes into standby-I mode if CE is low. Otherwise the next payload in TX FIFO is transmitted.
    // If TX FIFO is empty and CE is still high, nRF24L01+ enters standby-II mode. In case of using the REUSE_TX_PL functionality, the TX FIFO will never
    // be empty and the sending will repeat endlessly if CE stays high.

    // 3.) If the nRF24L01+ is in standby-II mode, it goes to standby-I mode immediately if CE is set low.
}

void nRF24L01P_PRX_Recv(uint8_t ce_pin)
{
    // 1. If the PRX is in Stanby-I mode and the CE pin is pulled high, the PRX device will transfer to RX mode and stay there as long as CE is pulled high.
    gpio_put(ce_pin, 1);
}
