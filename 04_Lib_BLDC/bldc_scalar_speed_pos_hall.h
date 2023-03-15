/******************************************************************************
* File Name: bldc_scalar_speed_pos_hall.h
*
* Description:Speed and position interface using 3 hall sensor feedback. 
*             This uses floating prescaler feature of CCU4 for speed capture
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

#include <stdint.h>

/**
 * @brief structure to get the position and speed
 */
typedef struct BLDC_SCALAR_SPEED_POS_HALL
{
    uint32_t captureval[6];  /*!< captured time value between two correct hall events */
    uint32_t capval;         /*!< Captured time Value */
    uint32_t speedcheck;     /*!< whether motor speed can be calculated */
    uint32_t speedaccum;     /*!< accumulated speed of the motor for 6 samples */
    uint32_t speed_constant; /*!< constant value used for speed calculation */
    uint8_t speedindex;      /*!< index of an array of the speed capture variables */
} BLDC_SCALAR_SPEED_POS_HALL_t;
extern BLDC_SCALAR_SPEED_POS_HALL_t Motor0_BLDC_SCALAR_SPEED_POS_HALL;

uint32_t Motor0_BLDC_SCALAR_SPEED_POS_HALL_ReadCaptureValue(void);
void Motor0_BLDC_SCALAR_SPEED_POS_HALL_SpeedCalculation(uint32_t capval, uint32_t* speed);
