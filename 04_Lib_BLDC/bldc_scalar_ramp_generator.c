/******************************************************************************
* File Name: bldc_scalar_ramp_generator.c
*
* Description:Linear ramp generator
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
#include "04_Lib_BLDC/lib_bldc.h"

/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
ramp_type_t Ramp;

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
void ramp_reset(void)
{
    Ramp.sum = 0;
}

void ramp_init(void)
{
    if (MotorParam.EnableCatchSpin == ENABLED)
    {
        Ramp.output = Motor0_BLDC_SCALAR.motor_speed;
    }
    /* Initialize ramp by target direction */
    else if (MotorVar.TargetValue >= 0)
    {
        Ramp.output = MotorParam.RAMP_INITIAL_VALUE;
    }
    else
    {
        Ramp.output = -MotorParam.RAMP_INITIAL_VALUE;
    }
}


/* This generates the linear ramp as per configured slew rate */
void Motor0_BLDC_SCALAR_Ramp_Linear(void)
{
    if ((MotorParam.RAMP_UP_RATE != 0) && (MotorParam.RAMP_DOWN_RATE != 0)) /* only do ramp if up rate and down rate are not 0 */
    {
        int32_t diff = MotorVar.TargetValue - Ramp.output;
        if (Ramp.sum >= 0)
        {
            /* Positive direction */
            if (diff >= 0)
            {
                /* positive direction ramp up */
                if (MotorParam.RAMP_UP_RATE == 0)
                {
                    Ramp.sum = MotorVar.TargetValue << 16;
                }
                else
                {
                    Ramp.sum = Ramp.sum + MotorParam.RAMP_UP_RATE;
                    if (Ramp.sum > (MotorVar.TargetValue << 16))
                    {
                        Ramp.sum = MotorVar.TargetValue << 16;
                    }
                }
            }
            else
            {
                /* positive direction ramp down */
                  /* Ramp down - If DC Link voltage within threshold limit then only ramp down */
                  if ((uint32_t)MotorVar.DCLinkVoltageFilter.output < MotorParam.RAMP_DOWN_VOLTAGE_LIMIT)
                  {
                    Ramp.sum = Ramp.sum - MotorParam.RAMP_DOWN_RATE;
                    if ((Ramp.sum < 0) && (MotorVar.TargetValue >= 0))
                        Ramp.sum = 0;
                  }
            }
        }
        else
        {
            /* Negative direction */
            if (diff < 0)
            {
                /* negative direction ramp up */
                Ramp.sum = Ramp.sum - MotorParam.RAMP_UP_RATE;
                if (Ramp.sum < (MotorVar.TargetValue << 16))
                {
                    Ramp.sum = MotorVar.TargetValue << 16;
                }
            }
            else
            {
                /* negative direction ramp down */
                  /* Ramp down - If DC Link voltage within threshold limit then only ramp down */
                  if ((uint32_t)MotorVar.DCLinkVoltageFilter.output < MotorParam.RAMP_DOWN_VOLTAGE_LIMIT)
                  {
                      Ramp.sum = Ramp.sum + MotorParam.RAMP_DOWN_RATE;
                      if ((Ramp.sum > 0) && (MotorVar.TargetValue < 0))
                          Ramp.sum = 0;
                  }
            }
        }
    }
    else /* no ramp if ramp rate is set 0 */
    {
        Ramp.output = MotorVar.TargetValue;
    }
}
