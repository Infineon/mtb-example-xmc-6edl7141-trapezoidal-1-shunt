/******************************************************************************
* File Name: lib_bldc.h
*
* Description: Define functions prototype for trapezoidal control
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
#include "03_Application_BLDC/application_bldc.h"

typedef struct
{
	union
	{
		struct
		{
			uint16_t	   :16;
			int16_t  output:16;
		};
		int32_t sum;
	};
} ramp_type_t;
extern ramp_type_t Ramp;

void control_loop_set_ptr(void);
void control_loop_reset_ptr(void);
void hall_learning_init(void);
void Motor0_BLDC_SCALAR_Hall_Learning(void);
void Motor0_BLDC_SCALAR_Ramp_Linear(void);
void offset_calibration(void);
void read_inverter_temperature(void);
void SPEED_POS_HALL_LowSpeedCalculation(int32_t* motor_speed);
void Motor0_BLDC_SCALAR_SPEED_POS_HALL_ClearSpeedFilter(void);
void ramp_reset(void);
void ramp_init(void);
void Motor0_BLDC_SCALAR_PatternInitiator(void);
void control_loop_set_ptr(void);
void control_loop_reset_ptr(void);

void Motor0_BLDC_SCALAR_StallDetection(void);
void motor_common_protection(void);
void Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(int16_t amplitude);
bool Motor0_BLDC_SCALAR_Bootstrap(void);				/* return true if in bootstrap state */

void set_cmpval_all_phases(uint16_t cmpval);			/* Set all phases to same output duty cycle */
void set_pwm_phase_pattern(uint16_t mcm_val);
void Motor0_BLDC_SCALAR_GetCurrentValue(void);
