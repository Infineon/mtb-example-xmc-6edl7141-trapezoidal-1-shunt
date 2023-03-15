/******************************************************************************
* File Name: application_bldc.c
*
* Description: Control algorithm and MCU peripheral configurations 
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
#include "01_TopLevel/user_input_config.h"
#include "04_Lib_BLDC/lib_bldc.h"

motor_var_t MotorVar;
motor_par_t MotorParam;
BLDC_SCALAR_t Motor0_BLDC_SCALAR;

/*
 *  This function handles command input either from
 *      a. analog (potentiometer)
 *      b. direct static assignment by writing to MotorVar.TargetValue
 */
void Motor0_BLDC_SCALAR_GetCommand(void)
{
    if (MotorParam.AnalogControl == ENABLED)
    {
        /* Potentiometer measurement */
        /*Read the pot result */
        uint16_t pot_value_adc = get_adc_pot_result_xmc14();
        MotorVar.PotentiometerFilter.sum += ((pot_value_adc - MotorVar.PotentiometerFilter.output) * 2048) << 2;    /* simple low pass filter, time constant = 8ms */
        int32_t potvalue = MotorVar.PotentiometerFilter.output * 4;      /* convert to Q14 */

        if (XMC_GPIO_GetInput(CYBSP_DIR_INPUT_PORT, CYBSP_DIR_INPUT_PIN) == 0)
        {
            potvalue = -potvalue;
        }
        Motor0_BLDC_SCALAR.analogip_val = potvalue;

        if ((potvalue < MotorParam.ANALOG_LOW_LIMIT) && (potvalue > -MotorParam.ANALOG_LOW_LIMIT))
        {
            potvalue = 0;
        }
        MotorVar.TargetValue = potvalue;

        if ((MotorVar.TargetValue == 0) && (Ramp.output == 0))
        {
            MotorVar.Command = MOTOR_STOP_CMD;
        }
        else // (MotorVar.TargetValue != 0)
        {
            MotorVar.Command = MOTOR_START_CMD;
        }
    }
    else
    {
        /* PUT YOUR CODE HERE TO ASSIGN A STATIC SPEED FOR THE MOTOR
         * when MotorParam.AnalogControl is disabled through BPA Motor Control GUI,
         * the MotorVar.TargetValue can be setup directly in this 'else' section
        */

    }
}

void ClearCommand(void)
{
    Ramp.sum = 0;
    MotorVar.PotentiometerFilter.sum = 0;
    MotorVar.TargetValue = 0;
    MotorVar.Command = MOTOR_STOP_CMD;
}
