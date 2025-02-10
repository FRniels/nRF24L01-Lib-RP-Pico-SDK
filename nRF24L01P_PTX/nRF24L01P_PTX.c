#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#include "nRF24L01P.h"

#define SPI_FREQ (1000 * 1000) // 1MHZ
#define NRF24L01P_CE_PIN 21
#define NRF24L01P_IRQ_PIN 22
#define SEND_INTERVAL_MS (-1000)

void SPI_Master_Init(void);
// Used to send data in a fixed interval.
void PICO_Start_Sending_Timer(void);
bool PICO_Repeating_Timer_Callback(__unused struct repeating_timer *t);

volatile bool nRF24L01P_isr_flag = false; // Declare the flag volatile so the compiler can not optimize it away!

struct repeating_timer timer_send;         // Used to send data in a fixed interval.
volatile bool nRF24L01P_send_flag = false; // Declare the flag volatile so the compiler can not optimize it away!

int main()
{
    stdio_init_all();

    sleep_ms(5000); // Give some time to open a terminal for observing the initialization logs.

    printf("Start nRF24L01P program:\r\n");

    SPI_Master_Init();

    nRF24L01P_GPIO_Init(NRF24L01P_CE_PIN, NRF24L01P_IRQ_PIN, &nRF24L01P_isr_flag);

    // 1. CONFIG REGISTER
    // The PRIM_RX should be '0' to configure the nRF24L01+ as a PTX which is the default value at start up.
    // Using a 2 byte CRC for more reliable but slightly slower communication speed. Note: This needs to be the same as the PRX
    uint8_t reg_config_set_mask = REG_SET_BIT(CONFIG_PWR_UP) | REG_SET_BIT(CONFIG_CRCO);
    // uint8_t reg_config_clear_mask = REG_CLEAR_BIT(CONFIG_PRIM_RX);

    printf("Write the CONFIG register. Set PWR_UP = '1', set CRCO = '1':\r\n");
    nRF24L01P_Write_Reg(REG_CONFIG, reg_config_set_mask, REG_CLEAR_MASK_UNUSED);
    // nRF24L01P_Write_Reg(REG_CONFIG, reg_config_set_mask, reg_config_clear_mask);
    nRF24L01P_Read_Reg(REG_CONFIG);

    // 2. ENABLE AUTO ACKNOWLEDGMENT REGISTER
    // The auto acknowledgement of all data pipes should be enabled by default.
    printf("Read the EN_AA register:\r\n");
    nRF24L01P_Read_Reg(REG_EN_AA);

    // 3. ENABLE RX ADDRESS REGISTER
    // Data pipes 0 and 1 are enabled by default
    printf("Read the EN_RXADDR register:\r\n");
    nRF24L01P_Read_Reg(REG_EN_RXADDR);

    // 4. SETUP ADDRESS WIDTH REGISTER
    // The default address width on startup is set to 5 bytes.
    // uint8_t reg_setup_aw_set_mask = AW_MASK(AW_5_BYTES);
    // uint8_t reg_setup_aw_clear_mask = 0b11110011;
    // nRF24L01P_Write_Reg(REG_SETUP_AW, reg_setup_aw_set_mask, reg_setup_aw_clear_mask);
    printf("Read the SETUP_AW register:\r\n");
    nRF24L01P_Read_Reg(REG_SETUP_AW);

    // 5. SETUP RETRANSMISSION REGISTER
    printf("Read the SETUP_RETR register:\r\n");
    nRF24L01P_Read_Reg(REG_SETUP_RETR);

    // 6. RF CHANNEL REGISTER
    // Should be the same channel that the PRX uses.
    printf("Read the RF_CH register:\r\n");
    nRF24L01P_Read_Reg(REG_RF_CH);
    // TO DO: TRY CHANNEL 25 (=2425MHz) ON BOTH PRX AND PTX, THIS WOULD FALL NICELY BETWEEN WIFI CHANNEL 1 AND 6

    // 7. RF SETUP REGISTER
    // The Data rate should be the same as the PRX device.
    printf("Read the RF_SETUP register:\r\n");
    nRF24L01P_Read_Reg(REG_RF_SETUP);

    // 8. FIFO STATUS REGISTER
    printf("Read the REG_FIFO_STATUS register:\r\n");
    nRF24L01P_Read_Reg(REG_FIFO_STATUS);

    // 9. TX_ADDR REGISTER
    printf("Set the TX ADDR in the TX_ADDR register:\r\n");
    uint8_t address_tx[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0x05}; // 0xB3B4B5B605: address from the data sheet example
    nRF24L01P_Set_Addr(REG_TX_ADDR, address_tx, 5);         // The address width is 5byte by default.
    uint8_t address_tx_read[5] = {0};                       // used for a test read to see if the write succeeded.
    nRF24L01P_Get_Addr(REG_TX_ADDR, address_tx_read, 5);
    nRF24L01P_Print_Addr(address_tx_read, 5);

    // 10. RX_ADDR_P0 REGISTER
    // Set the RX address for data pipe 0 to be able to receive the auto acknowledgment from the PRX. This address must be equal to the TX address!
    printf("Set the RX ADDR for data pipe 0 in the RX_ADDR_P0 register:\r\n");
    nRF24L01P_Set_Addr(REG_RX_ADDR_P0, address_tx, 5); // The address width is 5byte by default.
    uint8_t address_rx_read[5] = {0};                  // used for a test read to see if the write succeeded.
    nRF24L01P_Get_Addr(REG_RX_ADDR_P0, address_rx_read, 5);
    nRF24L01P_Print_Addr(address_rx_read, 5);

    // 11. W_TX_PAYLOAD COMMAND
    uint8_t tx_payload[6] = {'H', 'e', 'l', 'l', 'o', '\0'};
    printf("Writing the payload: '%s' to the TX_PAYLOAD register:\r\n\n", tx_payload);
    nRF24L01P_Write_TX_Payload(tx_payload, 6);

    // 12. REUSE_TX_PL COMMAND
    printf("Sending REUSE_TX_PL:\r\n\n");
    nRF24L01P_Reuse_TX_Payload();

    // 13. START SENDING THE PAYLOAD TO THE PRX IN A FIXED INTERVAL
    printf("Start the PTX sending loop at an interval of %dms:\r\n", (-SEND_INTERVAL_MS));
    PICO_Start_Sending_Timer();

    while (true)
    {
        // 14. SEND THE PAYLOAD TO THE PRX THAT HAS THE CONFIGURED PTX TX ADDRESS AS PRX RX ADDRESS
        if (nRF24L01P_send_flag)
        {
            printf("Send the payload to the PRX:\r\n");
            nRF24L01P_PTX_Send(NRF24L01P_CE_PIN);

            nRF24L01P_send_flag = false;
        }

        // 15. CHECK IF THE TX_DS OR MAX_RT FLAG IN THE STATUS REGISTER HAS BEEN SET AND CLEAR THE FLAG IF SET. AUTO ACKNOWLEDGEMENT IS ENABLED THUS THE TX_DS FLAG IS SET WHEN THE ACKNOWLEDGEMENT IS RECEIVED.
        if (nRF24L01P_isr_flag)
        {
            printf("ISR flag set in status register:\r\n");

            // 1. Read the status register
            uint8_t status_reg = nRF24L01P_Read_Status_Reg();
            // IMPORTANT: The status register requires writing a '1' to the bit that needs to be cleared to '0'!
            uint8_t status_clear_flag_mask = 0x00;

            // 2. Check which irq flag is set: TX_DS OR MAX_RT
            // 3. Execute required logic
            // 4. Clear the interrupt flag
            if (REG_GET_BIT(status_reg, STATUS_TX_DS))
            {
                printf("TX_DS flag triggered. Acknowledgment received from PRX, sending data succeeded!\r\n");

                // Execute required logic

                // Create a clear mask. Clear the interrupt flag by writing '1' to it's bit position in the status register, set all untouched bits in the mask to '0'.
                status_clear_flag_mask = REG_SET_BIT(STATUS_TX_DS);
            }
            else if (REG_GET_BIT(status_reg, STATUS_MAX_RT))
            {
                printf("MAX_RT flag triggered. Max transmissions exceeded!\r\n");

                // Execute required logic

                // Create a clear mask. Clear the interrupt flag by writing '1' to it's bit position in the status register, set all untouched bits in the mask to '0'.
                status_clear_flag_mask = REG_SET_BIT(STATUS_MAX_RT);
            }

            // Write the clear mask to the status register.
            nRF24L01P_Clear_Status_Reg_IRQ_Flags(status_clear_flag_mask);
            nRF24L01P_isr_flag = false;
        }
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

void PICO_Start_Sending_Timer(void)
{
    // Negative delay: The delay is from callback start to callback start, regardless of how long the callback took to execute.
    add_repeating_timer_ms(SEND_INTERVAL_MS, PICO_Repeating_Timer_Callback, NULL, &timer_send);
}

bool PICO_Repeating_Timer_Callback(__unused struct repeating_timer *t)
{
    // printf("Repeat at %lld\n", time_us_64());
    // printf("timer\r\n");
    nRF24L01P_send_flag = true;
    return true;
}
