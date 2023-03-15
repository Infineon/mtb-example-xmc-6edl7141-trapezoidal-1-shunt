/******************************************************************************
* File Name: uCProbe.h
*
* Description: Header file for uCProbe.c
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

#include "02_GUI_Interface/probe_scope.h"

/*
 * @brief Structure for UCPROBE interface for BLDC Scalar  motor control
 */
typedef struct BLDC_SCALAR_UCPROBE
{
    volatile uint32_t control_word;                             /*UCPROBE control word to send comments from GUI*/
    volatile int32_t dclinkgvoltage_Volt;                       /*Actual DC link voltage in Volt*/
    volatile int32_t motor_current_mA;                          /*Actual motor current in mA*/
    volatile int32_t motor_speed_RPM;                           /*Actual motor speed in RPM*/
    volatile int32_t speed_pi_error;                            /*Speed control PI error*/
    volatile int32_t speed_set_RPM;                             /*Actual motor speed set value in RPM*/
    volatile int32_t current_pi_error;                          /*Current control PI error*/
    volatile int32_t current_set_mA;                            /*Actual motor current set value in mA*/
    volatile int32_t voltage_set_Volt;                          /*Actual motor voltage set value in Volt*/

} BLDC_SCALAR_UCPROBE_t;
extern BLDC_SCALAR_UCPROBE_t Motor0_BLDC_SCALAR_ucprobe;

/**
 * @return None
 *
 * Description:
 * UCPROBE scheduler function to handle ucprobe comments from UI
 */
void Motor0_BLDC_SCALAR_uCProbe_Scheduler(void);
/**
 * @return None
 *
 * Description:
 * Handling flash variable, if flash contain any valid data, write to actual variable from flash
 */
void Motor0_BLDC_SCALAR_Flash_Var_Init(void);
