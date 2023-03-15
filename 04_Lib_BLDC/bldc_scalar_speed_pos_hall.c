/******************************************************************************
* File Name: bldc_scalar_speed_pos_hall.c
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
#include "01_TopLevel/core_control.h"
#include "04_Lib_BLDC/lib_bldc.h"
#include "04_Lib_BLDC/bldc_scalar_speed_pos_hall.h"

/** This defines speed accumulation limit value*/
#define BLDC_SCALAR_SPEED_POS_HALL_SPEEDACCUMLIMITCHECK  (5U)
/** Maximum period value of CCU4 slice */
#define BLDC_SCALAR_SPEED_POS_HALL_CAP_COMP_VAL          (0xFFFFU)
/** Capture register number */
#define BLDC_SCALAR_HALL_CAPTURE_REGITSER                (3U)

BLDC_SCALAR_SPEED_POS_HALL_t Motor0_BLDC_SCALAR_SPEED_POS_HALL;

/**
 * @brief Hall sensor low speed handling
 * 
 * @param motor_speed 
 * @return 
 */
void SPEED_POS_HALL_LowSpeedCalculation(int32_t* motor_speed)
{
    /* If motor stopped or running very slow, we still need to update speed, otherwise motor speed will stuck if no new hall events captured */
    /* Read timer value pair with actual prescaler value */
    uint32_t prescaler_value_prev = PeriPtr.capture_slice->FPC >> CCU4_CC4_FPC_PVAL_Pos;
    uint32_t timer_value = PeriPtr.capture_slice->TIMER;
    uint32_t prescaler_value = PeriPtr.capture_slice->FPC >> CCU4_CC4_FPC_PVAL_Pos;
    if (prescaler_value != prescaler_value_prev)
    {
        /* if prescaler value changed, need to read timer value again */
        timer_value = PeriPtr.capture_slice->TIMER;
    }
    /* calculate floating prescaler timer value */
    uint32_t capval = (((1UL << (prescaler_value - CCU4_PRESCALER)) - 1) << 16) + timer_value;
    /* find the very first capture value in the array */
    uint32_t index;
    index = Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex;
    index += 6 - Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedcheck;    /* need to adjust index if we are not calculating 6 samples */
    if (index > 5) index -= 6;
    uint32_t first_capval = Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[index];
    /* re-calculate speed because motor speed is reducing */
    if (capval > first_capval)  /* re-calculate speed because motor speed is reducing */
    {
        // This is not very good approach to adjust speed, leave it for now
        int32_t speed_temp = *motor_speed;
        if ((speed_temp >= -64) && (speed_temp < 0)) speed_temp += 4;
        else speed_temp = speed_temp * 63 >> 6;
        *motor_speed = speed_temp;
    }
}


/**
 * @return none <br>
 *
 * \par<b>Description:</b><br>
 * Resets variables related to speed calculation.
 */
void Motor0_BLDC_SCALAR_SPEED_POS_HALL_ClearSpeedFilter(void)
{
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedcheck = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[0] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[1] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[2] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[3] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[4] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[5] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.speed_constant = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.capval = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum = 0U;
}


/**
 * @param capval captured value calculated from captured timer and prescaler
 * @return BLDC_SCALAR_SPEED_POS_HALL_STATUS_t status of API execution \n
 * BLDC_SCALAR_SPEED_POS_HALL_STATUS_FAILURE: if valid capture value is not available \n
 * BLDC_SCALAR_SPEED_POS_HALL_STATUS_SUCCESS: if valid capture value is available \n
 *  <br>
 *
 * \par<b>Description:</b><br>
 *  Calculates the captured time from timer value and current prescaler value in capture register
 *
 */
uint32_t Motor0_BLDC_SCALAR_SPEED_POS_HALL_ReadCaptureValue(void)
{
    uint32_t temp_capval = XMC_CCU4_SLICE_GetCaptureRegisterValue(PeriPtr.capture_slice, BLDC_SCALAR_HALL_CAPTURE_REGITSER);

    /* Read the captured value and save in the dynamic handle */
    uint32_t curr_psc = (temp_capval & CCU4_CC4_CV_FPCV_Msk) >> CCU4_CC4_CV_FPCV_Pos;
    temp_capval = temp_capval & BLDC_SCALAR_SPEED_POS_HALL_CAP_COMP_VAL;
    uint32_t capval = (((1UL << (curr_psc - CCU4_PRESCALER)) - 1) << 16) + temp_capval;

    return (capval);
}

/**
 * @param capval time between two hall events (60 degrees)
 * @param speed Calculated electrical speed in RPM
 * @return None
 *  <br>
 *
 * \par<b>Description:</b><br>
 * Calculates the speed based upon the captured time value between two correct hall events.
 * It uses the floating prescaler for better resolution and low speed value.
 */
/*This function will calculate the speed based upon the captured time values.*/
void Motor0_BLDC_SCALAR_SPEED_POS_HALL_SpeedCalculation(uint32_t capval, uint32_t* speed)
{
    /* Use speedcheck to indicate how many valid samples used for speed calculation */
    if (Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedcheck < 6)
    {
        Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedcheck++;
        Motor0_BLDC_SCALAR_SPEED_POS_HALL.speed_constant = (uint32_t)((1000000.0F * CCU4_FREQUENCY_MHZ) * 10.0F) * Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedcheck;
    }
    /* Moving average to calculate the speed */
    else // (Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedcheck >= 6)
    {
        Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum -= Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex];
    }
    Motor0_BLDC_SCALAR_SPEED_POS_HALL.captureval[Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex] = capval;
    Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum += capval;

    *speed = Motor0_BLDC_SCALAR_SPEED_POS_HALL.speed_constant / (Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum);

    /* Update moving average index */
    if (Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex >= BLDC_SCALAR_SPEED_POS_HALL_SPEEDACCUMLIMITCHECK)
    {
        Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex = 0U;
    }
    else
    {
        Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedindex++;
    }
}
