#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#include "nRF24L01P.h"

#define SPI_FREQ (1000 * 1000) // 1MHZ
#define NRF24L01P_CE_PIN 21    // The CE pin is tied to ground at the moment
#define NRF24L01P_IRQ_PIN 22

void SPI_Master_Init(void);
void Print_Payload(uint8_t *payload, uint8_t payload_len);

volatile bool nRF24L01P_isr_flag = false; // Declare the flag volatile so the compiler can not optimize it away!
uint8_t payload[32] = {0};
uint8_t payload_len = 0;

int main()
{
    stdio_init_all();

    sleep_ms(5000); // Give some time to open a terminal for observing the initialization logs.

    printf("Start nRF24L01 program:\r\n");

    SPI_Master_Init();

    nRF24L01P_GPIO_Init(NRF24L01P_CE_PIN, NRF24L01P_IRQ_PIN, &nRF24L01P_isr_flag);

    // 1. CONFIG REGISTER
    // The PRIM_RX should be '1' to configure the nRF24L01+ as a PRX.
    // Using a 2 byte CRC for more reliable but slightly slower communication speed. Note: This needs to be the same as the PTX
    // If the PWZR_UP flag is set to '1' a delay in the range 150us - 4.5ms, dependend on the clock source used, is needed to transition from power down mode to standby-I mode. (Tpd2stby)
    uint8_t reg_config_set_mask = REG_SET_BIT(CONFIG_PWR_UP) | REG_SET_BIT(CONFIG_PRIM_RX) | REG_SET_BIT(CONFIG_CRCO);

    printf("Write the CONFIG register. Set PWR_UP = '1', PRIM_RX = '1', set CRCO = '1':\r\n");
    nRF24L01P_Write_Reg(REG_CONFIG, reg_config_set_mask, REG_CLEAR_MASK_UNUSED);
    nRF24L01P_Read_Reg(REG_CONFIG);
    sleep_ms(5); // Setting a 5ms delay to be sure the device is in stanby mode before the main configuration is done. => TO DO: SEE IF THIS RESOLVE THE PRX FROM SOMETIMES NOT INITIALIZING CORRECTLY ON STARTUP. PTX SEEMS TO NEVER FAIL WITHOUT THIS ???

    // 2. ENABLE AUTO ACKNOWLEDGMENT REGISTER
    // The auto acknowledgement of all data pipes should be enabled by default.
    printf("Read the EN_AA register:\r\n");
    nRF24L01P_Read_Reg(REG_EN_AA);

    // 3. ENABLE RX ADDRESS REGISTER
    // Data pipes 0 and 1 should be enabled by default
    printf("Read the EN_RXADDR register:\r\n");
    nRF24L01P_Read_Reg(REG_EN_RXADDR);

    // 4. SETUP ADDRESS WIDTH REGISTER
    // The default address width on startup is set to 5 bytes.
    // uint8_t reg_setup_aw_set_mask = AW_MASK(AW_5_BYTES);
    // uint8_t reg_setup_aw_clear_mask = 0b11110011;
    // nRF24L01P_Write_Reg(REG_SETUP_AW, reg_setup_aw_set_mask, reg_setup_aw_clear_mask);
    printf("Read the SETUP_AW register:\r\n");
    nRF24L01P_Read_Reg(REG_SETUP_AW);

    // 5. FIXED PAYLOAD WIDTH: RX PAYLOAD WIDTH REGISTER
    // All data pipes payload width are 0bytes at startup, this means that the data pipe is not used!
    // Data pipe 0 is used for the test PRX
    printf("Set the RX_PW_P0 register to use a payload of 6bytes:\r\n"); // Receiving payload from test PTX: 'H', 'e', 'l', 'l', 'o', '\0' => 6bytes payload
    // TO DO: CREATE A MACRO FOR THE MASK TO ENSURE THE PASSED PAYLOAD WIDTH VALUE CAN NOT EXCEED 6 BITS!
    nRF24L01P_Write_Reg(REG_RX_PW_P0, 0x06, REG_CLEAR_MASK_UNUSED);
    nRF24L01P_Read_Reg(REG_RX_PW_P0);

    // 6. RF CHANNEL REGISTER
    // Should be the same channel that the PTX uses.
    printf("Read the RF_CH register:\r\n");
    nRF24L01P_Read_Reg(REG_RF_CH);
    // TO DO: TRY CHANNEL 25 (=2425MHz) ON BOTH PRX AND PTX, THIS WOULD FALL NICELY BETWEEN WIFI CHANNEL 1 AND 6

    // 7. RF SETUP REGISTER
    // The Data rate should be the same as the PTX device.
    printf("Read the RF_SETUP register:\r\n");
    nRF24L01P_Read_Reg(REG_RF_SETUP);

    // 8. FIFO STATUS REGISTER
    printf("Read the REG_FIFO_STATUS register:\r\n");
    nRF24L01P_Read_Reg(REG_FIFO_STATUS);

    // 9. RX_ADDR_P0 REGISTER
    // Set the RX address for data pipe 0 to be able to receive a payload at this data pipe. This address must be equal to the PTX TX address!
    uint8_t address_rx[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0x05}; // 0xB3B4B5B605: address from the data sheet example
    printf("Set the RX ADDR for data pipe 0 in the RX_ADDR_P0 register:\r\n");
    nRF24L01P_Set_Addr(REG_RX_ADDR_P0, address_rx, 5); // The address width is 5byte by default.
    uint8_t address_rx_read[5] = {0};                  // used for a test read to see if the write succeeded.
    nRF24L01P_Get_Addr(REG_RX_ADDR_P0, address_rx_read, 5);
    nRF24L01P_Print_Addr(address_rx_read, 5);

    // 10. TRANSFER TO RX MODE
    printf("Entering RX mode:\r\n");
    nRF24L01P_PRX_Recv(NRF24L01P_CE_PIN);

    while (true)
    {
        // 11. READ THE PAYLOADS THAT ARE RECEIVED FROM THE PTX
        if (nRF24L01P_isr_flag)
        {
            printf("ISR flag set in status register:\r\n");

            // 1. Read the status register
            uint8_t status_reg = nRF24L01P_Read_Status_Reg();
            // IMPORTANT: The status register requires writing a '1' to the bit that needs to be cleared to '0'!
            uint8_t status_clear_flag_mask = 0x00;

            // Datasheet page 63
            // 2. For a PRX only the RX_DR flag can (should) be set upon receiving data
            // 3. Read the payload
            // 4. Execute required logic
            // 5. Clear the interrupt flag
            // 6. Check the FIFO STATUS register if there are more payloads in the RX FIFO
            // 7. If more payloads are available in the RX FIFO, repeat from step 3
            if (REG_GET_BIT(status_reg, STATUS_RX_DR))
            {
                printf("RX_DR flag triggered. Data received in the RX FIFO!\r\n");

                uint8_t fifo_status_reg = 0x00;
                do
                {
                    // Check the status register for the data pipe number that has received data end read the RX FIFO which also clears the packet from the FIFO.
                    printf("Read payload: ");
                    payload_len = nRF24L01P_Read_RX_Payload(status_reg, payload);
                    Print_Payload(payload, payload_len);

                    // Create a clear mask. Clear the interrupt flag by writing '1' to it's bit position in the status register, set all untouched bits in the mask to '0'.
                    status_clear_flag_mask = REG_SET_BIT(STATUS_RX_DR);

                    // Write the clear mask to the status register.
                    nRF24L01P_Clear_Status_Reg_IRQ_Flags(status_clear_flag_mask);

                    // Check if the RX FIFO has more payloads available.
                    printf("Check if the RX FIFO contains more payloads:\r\n");
                    fifo_status_reg = nRF24L01P_Read_Reg(REG_FIFO_STATUS);

                } while (REG_GET_BIT(fifo_status_reg, FIFO_STATUS_RX_EMPTY) == 0);

                printf("Processed all pending payloads\r\n\n");
                nRF24L01P_isr_flag = false;
            }
        }

        tight_loop_contents();
    }
}

void SPI_Master_Init(void)
{
    printf("Initialising the SPI master @ 1MHz\r\n");

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, SPI_FREQ);
    spi_set_format(spi_default, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    // Setting the CSN pin high as default. This is to be sure that the CSN pin is high to create a falling edge for the very first SPI transaction.
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);              // Initialize the CSN pin
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT); // Configure as output
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);            // Set HIGH (idle state)
}

void Print_Payload(uint8_t *payload, uint8_t payload_len)
{
    /*
    for (uint8_t i = 0; i < payload_len; ++i)
    {
        printf("%c", payload[i]);
    }
    printf("\r\n\n");
    */

    // In the test PTX an actual string is sent:
    printf("%s\r\n\n", payload);
}
