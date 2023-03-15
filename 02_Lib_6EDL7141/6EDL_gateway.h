/******************************************************************************
* File Name: 6EDL_gateway.h
*
* Description: UART communication to GUI
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
#include "02_Lib_6EDL7141/6EDL7141.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
extern uint8_t GUIwrReg_6EDL7141_addr;
extern uint8_t WriteReg_6EDL7141_addr;
extern uint16_t GUIwrReg_6EDL7141_data;
extern uint16_t WriteReg_6EDL7141_data;
extern uint8_t GuiMonitor_6EDL7141_addr;
extern uint16_t GuiMonitor_6EDL7141_value;

typedef struct
{
	uint8_t en_drv_level:1;
	uint8_t nbrake_level:1;
} edl_io_control_t;
extern edl_io_control_t EdlIo;

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
void load_edl7141_flash_parameter(void);
void spi_read_6EDL7141_registers(void);
void spi_write_6EDL7141_registers(void);
void edl7141_update(void);
void edl7141_gateway_init(void);

