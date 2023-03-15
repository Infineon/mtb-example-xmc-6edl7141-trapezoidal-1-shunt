/******************************************************************************
* File Name: bldc_scalar_control_scheme.c
*
* Description: Control algorithm routines like control scheme, 
*              voltage compensation, stall detection
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
#include "03_Application_BLDC/application_bldc.h"
#include "04_Lib_BLDC/lib_bldc.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
extern void bldc_pwm_output(int16_t voltage_command);

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

/**
* @param output Output of the speed control
* @return None <br>
*
* \par<b>Description:</b><br>
* Executes speed PI loop and returns the output voltage
* This is called from control loop event.
*/
void speed_pi_regulator(void)
{
    /*Speed Control based on PI technique*/
    MotorVar.speed_integral += MotorVar.speed_error * MotorParam.SPEED_KI * MotorVar.speed_sat; /*Integral output I[k] = I[k-1] + Ki * error[k] */
    int32_t output = (MotorVar.speed_error * MotorParam.SPEED_KP + MotorVar.speed_integral) >> SPEED_PI_SCALE; /*PI output U[k] = Kp * error[k] + I[k]. */
    if (output > MotorParam.SPEED_PI_LIMIT)
    {
        output = MotorParam.SPEED_PI_LIMIT;
        MotorVar.speed_sat = 0;
    }
    else if (output < -MotorParam.SPEED_PI_LIMIT)
    {
        output = -MotorParam.SPEED_PI_LIMIT;
        MotorVar.speed_sat = 0;
    }
    else
    {
        MotorVar.speed_sat = 1;
    }
    MotorVar.speed_pi_output = output;
}

/**
* @param output Output of the current control
* @return None <br>
*
* \par<b>Description:</b><br>
* Executes current PI loop and returns the output voltage
* This is called from control loop event.
*/
void current_pi_regulator(void)
{
    /*Current Control based on PI technique*/
    MotorVar.current_integral += MotorVar.current_error * MotorParam.CURRENT_KI * MotorVar.current_sat; /*Integral output I[k] = I[k-1] + Ki * error[k] */
    int32_t output = (MotorVar.current_error * MotorParam.CURRENT_KP + MotorVar.current_integral) >> CURRENT_PI_SCALE; /*PI output U[k] = Kp * error[k] + I[k]. */
    if (output > MotorParam.CURRENT_PI_LIMIT)
    {
        output = MotorParam.CURRENT_PI_LIMIT;
        MotorVar.current_sat = 0;
    }
    else if (output < -MotorParam.CURRENT_PI_LIMIT)
    {
        output = -MotorParam.CURRENT_PI_LIMIT;
        MotorVar.current_sat = 0;
    }
    else
    {
        MotorVar.current_sat = 1;
    }
    MotorVar.current_pi_output = output;
}

void ControlLoop_Voltage_Mode(void)
{
    /* Voltage control */
    bldc_pwm_output(Ramp.output);
}

void ControlLoop_Speed_Mode(void)
{
    /* Speed control */
    Motor0_BLDC_SCALAR.speedcontrol_rate_counter++;
    if (Motor0_BLDC_SCALAR.speedcontrol_rate_counter >= MotorParam.SPEED_CONTROL_RATE)
    {
        Motor0_BLDC_SCALAR.speedcontrol_rate_counter = 0;
        MotorVar.speed_ref = Ramp.output;
        MotorVar.speed_error = MotorVar.speed_ref - Motor0_BLDC_SCALAR.motor_speed;
        speed_pi_regulator();
        bldc_pwm_output(MotorVar.speed_pi_output);
    }
}

void ControlLoop_Current_Mode(void)
{
  /* Current control */
    MotorVar.current_ref = Ramp.output;
    MotorVar.current_error = MotorVar.current_ref - MotorVar.MotorCurrent;
    current_pi_regulator();
    bldc_pwm_output(MotorVar.current_pi_output);
}

void ControlLoop_SpeedCurrent_Mode(void)
{
    /* Speed inner current control */
    Motor0_BLDC_SCALAR.speedcontrol_rate_counter++;
    if (Motor0_BLDC_SCALAR.speedcontrol_rate_counter >= MotorParam.SPEED_CONTROL_RATE)
    {
        Motor0_BLDC_SCALAR.speedcontrol_rate_counter = 0;
        MotorVar.speed_ref = Ramp.output;
        MotorVar.speed_error = MotorVar.speed_ref - Motor0_BLDC_SCALAR.motor_speed;
        speed_pi_regulator();
        MotorVar.current_ref = MotorVar.speed_pi_output;
        MotorVar.current_error = MotorVar.current_ref - MotorVar.MotorCurrent;
        current_pi_regulator();
        bldc_pwm_output(MotorVar.current_pi_output);
    }
}

void control_loop_set_ptr(void)
{
    if (MotorParam.ControlScheme == SPEED_CURRENT_CONTROL)
    {
        SystemVar.ControlLoop = ControlLoop_SpeedCurrent_Mode;
    }
    else if (MotorParam.ControlScheme == SPEED_CONTROL)
    {
        SystemVar.ControlLoop = ControlLoop_Speed_Mode;
    }
    else if (MotorParam.ControlScheme == CURRENT_CONTROL)
    {
        SystemVar.ControlLoop = ControlLoop_Current_Mode;
    }
    else    /* VOLTAGE_CONTROL */
    {
        SystemVar.ControlLoop = ControlLoop_Voltage_Mode;
    }
}


void Null_Function(void)
{

}


void control_loop_reset_ptr(void)
{
    SystemVar.ControlLoop = Null_Function;     /* initialize control loop function pointer */
}

/**
 * @endcond
 */
