/******************************************************************************
* File Name: 6EDL_perhiperal_xmc14.c
*
* Description: SPI interface for middleware and application code
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "01_TopLevel/core_control.h"
#include <xmc_gpio.h>
#include "02_Lib_6EDL7141/6EDL_peripheral_xmc14.h"

/*********************************************************************************************************************
 * DATA STRUCTURE
 ********************************************************************************************************************/
uint8_t error_SPI;

/* SPI configuration structure */
XMC_SPI_CH_CONFIG_t spi_config =
{
    .baudrate = SPI_BAUD_RATE,
    .bus_mode = XMC_SPI_CH_BUS_MODE_MASTER,
    .selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS,  //set Slave select active low
    .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE
};

/* GPIO SPI TX pin configuration */
XMC_GPIO_CONFIG_t tx_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9
};

/* GPIO SPI DX pin configuration */
XMC_GPIO_CONFIG_t dx_pin_config =
{
    .mode = XMC_GPIO_MODE_INPUT_TRISTATE
};

/* GPIO SPI SELO pin configuration */
XMC_GPIO_CONFIG_t selo_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8,
};

/* GPIO SPI SCLKOUT pin configuration */
XMC_GPIO_CONFIG_t clk_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8,
};

/* 6EDL EN_DRV pin */
const XMC_GPIO_CONFIG_t GPIO_EN_DRV_Config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
/* 6EDL watchdog clock on EN_DRV pin */
const XMC_GPIO_CONFIG_t GPIO_WD_CLK_Config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t GPIO_nBRAKE_Config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,
};
const XMC_GPIO_CONFIG_t GPIO_AUTO_ZERO_Config =
{
    .mode = XMC_GPIO_MODE_INPUT_PULL_UP,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

void gate_driver_gpio_init(void)
{
    /* Initialize SPI pins */
    XMC_GPIO_Init(CYBSP_GD_SPI_MOSI_PORT, CYBSP_GD_SPI_MOSI_PIN, &tx_pin_config);
    XMC_GPIO_Init(CYBSP_GD_SPI_MISO_PORT, CYBSP_GD_SPI_MISO_PIN, &dx_pin_config);
    XMC_GPIO_Init(CYBSP_GD_SPI_nSCS_PORT, CYBSP_GD_SPI_nSCS_PIN, &selo_pin_config);
    XMC_GPIO_Init(CYBSP_GD_SPI_SCLK_PORT, CYBSP_GD_SPI_SCLK_PIN, &clk_pin_config);
    /* Initialize EN_DRV/WD_CLK/nBRAKE/ZUTO_ZERO pins */
    XMC_GPIO_Init(CYBSP_GD_EN_DRV_PORT, CYBSP_GD_EN_DRV_PIN, &GPIO_EN_DRV_Config);
    XMC_GPIO_Init(CYBSP_GD_WD_CLK_PORT, CYBSP_GD_WD_CLK_PIN, &GPIO_WD_CLK_Config);
    XMC_GPIO_Init(CYBSP_GD_nBRAKE_PORT, CYBSP_GD_nBRAKE_PIN, &GPIO_nBRAKE_Config);
    XMC_GPIO_Init(CYBSP_GD_AZ_PORT, CYBSP_GD_AZ_PIN, &GPIO_AUTO_ZERO_Config);
}

/* API to initialize USIC SPI peripherals */
void gate_driver_spi_master_init(void)
{
    /* Initialize SPI master*/
    XMC_USIC_CH_t * const channel = SPI_MAS_CH;
    XMC_SPI_CH_Init(channel, &spi_config);
    XMC_SPI_CH_SetWordLength(channel, SPI_WORD_LENGTH);
    XMC_SPI_CH_SetFrameLength(channel, SPI_FRAME_LENGTH);
    XMC_SPI_CH_SetBitOrderMsbFirst(channel);
    XMC_SPI_CH_EnableSlaveSelect(channel, XMC_SPI_CH_SLAVE_SELECT_0);
    XMC_SPI_CH_ConfigureShiftClockOutput(channel,
                                         XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_DISABLED,
                                         XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK); //Falling sclk sampling and no polarity inversion
    XMC_SPI_CH_SetSlaveSelectPolarity(channel,
                                      XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS); //Slave select is active low to communicate with 6EDL7141

    /* Initialize FIFO */
    XMC_USIC_CH_RXFIFO_Configure(channel, 40, XMC_USIC_CH_FIFO_SIZE_8WORDS, 2); //receive from SPI slave: limit is 1, when FIFO is full with 2 word and a 3nd is received, the event happens.
    //this is used in main to while until 3 words are received
    XMC_USIC_CH_TXFIFO_Configure(channel, 32, XMC_USIC_CH_FIFO_SIZE_8WORDS, 0);

    /* Configure input multiplexer */
    XMC_SPI_CH_SetInputSource(channel, XMC_SPI_CH_INPUT_DIN0, USIC1_C1_DX0_P0_1); //P0.1 DX0B
    /* Start operation. */
    XMC_SPI_CH_Start(channel);

    error_SPI = 0;
}

/* To send the read command to the SPI channel.
   The RegAddr is the address of register to read.
   Data parameter is a pointer to store the two bytes of data read from the register */
uint16_t Read_Word_16b(uint8_t RegAddr)
{
    XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
    SPI_MAS_CH->IN[0] = RegAddr & REG_READ;
    SPI_MAS_CH->IN[0] = 0x00;
    SPI_MAS_CH->IN[0] = 0x00;
    uint32_t timeout_counter = 0;
    /* wait for received data /2 words */
    while ((XMC_USIC_CH_RXFIFO_GetEvent(SPI_MAS_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 0)
    {
        if (++timeout_counter > SPI_TIMEOUT)
        {
            error_SPI = 1;
            break;
        }
    };

    XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
    SPI_MAS_CH->OUTR;                       //data read of invalid data
    uint8_t data_high = SPI_MAS_CH->OUTR;   //data read of MSB
    uint8_t data_low = SPI_MAS_CH->OUTR;    //data read of LSB
    return ((data_high << 8) + data_low);
}

/* To write data to register via SPI channel.
   The RegAddr is the address of register to write to.
   Data parameter is a pointer which stored the two bytes of data written to the register */
void Write_Word_16b(uint8_t RegAddr, uint16_t data)
{
    XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
    SPI_MAS_CH->IN[0] = RegAddr | REG_WRITE;
    SPI_MAS_CH->IN[0] = (uint8_t)(data >> 8);
    SPI_MAS_CH->IN[0] = (uint8_t)data;
    uint32_t timeout_counter = 0;
    /* wait for received data /2 words */
    while ((XMC_USIC_CH_RXFIFO_GetEvent(SPI_MAS_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 0)
    {
        if (++timeout_counter > SPI_TIMEOUT)
        {
            error_SPI = 1;
            break;
        }
    };

    XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
    SPI_MAS_CH->OUTR;   //data read to clear buffer
    SPI_MAS_CH->OUTR;   //data read to clear buffer
    SPI_MAS_CH->OUTR;   //data read to clear buffer
}

