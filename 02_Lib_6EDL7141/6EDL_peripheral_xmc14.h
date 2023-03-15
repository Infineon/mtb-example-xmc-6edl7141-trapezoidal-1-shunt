/******************************************************************************
* File Name: 6EDL_perhiperal_xmc14.h
*
* Description: XMC1400 SPI communication to gate driver IC
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
#pragma once

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "01_TopLevel/core_control.h"
#include <xmc_common.h>
#include <xmc_spi.h>
#include <xmc_usic.h>

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
/* SPI configuration */
#define SPI_MAS_CH       XMC_SPI1_CH1
#define SPI_BAUD_RATE    8000000
#define SPI_WORD_LENGTH  (8U)
#define SPI_FRAME_LENGTH (24U)
#define SPI_TIMEOUT      (25)           /* software delay counter for SPI communication fault detection, need to adjust by MCU platform */

/* Register write and read definition */
#define REG_WRITE        0x80
#define REG_READ         0x7F

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
void gate_driver_gpio_init(void);
void gate_driver_spi_master_init(void);
/* SPI Functions */
uint16_t Read_Word_16b(uint8_t RegAddr);
void Write_Word_16b(uint8_t RegAddr, uint16_t data);

