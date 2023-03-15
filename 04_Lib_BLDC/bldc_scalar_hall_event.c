/******************************************************************************
* File Name: bldc_scalar_hall_event.c
*
* Description: Wrong hall event (always enabled ) and 
*              Correct hall event (only for state identification)
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
#include <stdlib.h>
#include "01_TopLevel/core_control.h"
#include "04_Lib_BLDC/lib_bldc.h"
#include "04_Lib_BLDC/bldc_scalar_speed_pos_hall.h"
#include "04_Lib_BLDC/bldc_scalar_pwm_bc.h"

#define BLDC_SCALAR_SPEED_SCALE_RES                  (10)
void Commutation_ISR(void);

/**
 * @addtogroup BLDC_SCALAR BLDC Motor Control
 * @{
 */

/**
 * @addtogroup Interrupt Interrupts
 * @brief  Interrupt Service Routines  <br>
 * @{
 */
/**
 * @param none
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Multi-channel pattern shadow transfer interrupt ISR. \n
 * Prepare hall and multi-channel pattern for the next commutation.
 * Calculate the Speed based on captured value in CCU4 capture slice.
 */
 /**
 * The diag_supress pragma is applied to supress the specified diagnostic message
 * due to the calling of flash function from RAM function.
 * The diag_supress is documented on page 411.
 * On page 395, __ram_func allows to disable the warning.
 * https://wwwfiles.iar.com/arm/webic/doc/ewarm_developmentguide.enu.pdf
*/
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif 
__RAM_FUNC void Motor0_BLDC_SCALAR_HallEvent_ISR(void)
{
    /* Update new hall pattern and read capture value */
    uint32_t new_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(PeriPtr.posif);
    int32_t new_hall_sector = (MotorParam.HALL_SECTOR_MAP >> (new_hall_pattern << 2)) & 0x0F;
    MotorVar.hall_pattern = new_hall_pattern;  /* store the sampled pattern */
    uint32_t capture_value = Motor0_BLDC_SCALAR_SPEED_POS_HALL_ReadCaptureValue();
    Motor0_BLDC_SCALAR_SPEED_POS_HALL.capval = capture_value;   // make stall detection code happy

    /* validate the new hall sector */
    enum {NO_REQUEST=0, UPDATE_MCM=1, UPDATE_SPEED=2};
    uint8_t request = NO_REQUEST;        /* 0: no request, 1: request multi-channel pattern update, 2: request speed update, 3: request both */
    int32_t expected_fwd_sector = MotorVar.hall_sector + 1;
    if (expected_fwd_sector == 6) expected_fwd_sector = 0;
    /* Moving forward? */
    if (new_hall_sector == expected_fwd_sector)
    {
        if (Motor0_BLDC_SCALAR.actual_motor_direction == 1)
        {
            /* No direction change, update speed */
            request = UPDATE_MCM + UPDATE_SPEED;       /* need to update motor speed and multi-channel pattern */
        }
        else
        {
            if (Motor0_BLDC_SCALAR.motor_speed > -1024)
            {
                /* There is direction change, only update MCM and set speed to 0 */
                Motor0_BLDC_SCALAR.actual_motor_direction = 1;
                Motor0_BLDC_SCALAR.motor_speed = 0;
                Motor0_BLDC_SCALAR.abs_motor_speed = 0;
                request = UPDATE_MCM;   /* need to update multi-channel pattern */
            }
            else
            {
                /* was running reverse high speed, direction change should not happen */
            }
        }
    }
    else
    {
        int32_t expected_rev_sector = MotorVar.hall_sector - 1;
        if (expected_rev_sector == -1) expected_rev_sector = 5;
        /* Moving reverse? */
        if (new_hall_sector == expected_rev_sector)
        {
            if (Motor0_BLDC_SCALAR.actual_motor_direction == -1)
            {
                /* No direction change, update speed */
                request = UPDATE_MCM + UPDATE_SPEED;       /* need to update motor speed and multi-channel pattern */
            }
            else /* (Motor0_BLDC_SCALAR.actual_motor_direction == 1) */
            {
                if (Motor0_BLDC_SCALAR.motor_speed < 1024)
                {
                    /* There is direction change, only update MCM and set speed to 0 */
                    Motor0_BLDC_SCALAR.actual_motor_direction = -1;
                    Motor0_BLDC_SCALAR.motor_speed = 0;
                    Motor0_BLDC_SCALAR.abs_motor_speed = 0;
                    request = UPDATE_MCM;   /* need to update multi-channel pattern */
                }
                else
                {
                    /* was running forward high speed, direction change should not happen */
                }
            }
        }
        /* Mis-trigger may happen at low speed: after wrong hall event detected, rotor may move back to previous position.
         * Only report fault when sampled pattern is different to previous pattern, to avoid false trigger wrong hall fault */
        else if (new_hall_sector == MotorVar.hall_sector)
        {
            /* sector is unchanged */
            request = UPDATE_MCM;   /* need to update multi-channel pattern */
        }
        else
        {
            /* sector is invalid, set wrong hall fault bit */
            MotorVar.error_status |= 1U << BLDC_SCALAR_EID_WRONGHALL;
        }
    }

    /* If requested, update multi-channel pattern and apply the PWM modulation */
    if ((request & UPDATE_MCM) && (Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_NORMAL_OPERATION))
    {
        int32_t speed_delta = ((abs(Motor0_BLDC_SCALAR.motor_speed) - MotorParam.PHADV_START_SPEED) * (int32_t)MotorParam.PHADV_START_SCALE) >> 14;
        uint16_t commutation_delay = MotorParam.COMMUTATION_DELAY;
        if (speed_delta > 0)
        {
            /* Phase advance */
            if (MotorVar.phase_advance_flag == 0)
            {
                /* Phase advance was off, set PWM pattern for smooth transition */
                MotorVar.hall_sector = new_hall_sector;    /* Use new hall sector */
                Commutation_ISR();
                MotorVar.phase_advance_flag = 1;
            }
            if(speed_delta>(1U<<14))
            {
                speed_delta = 1U<<14;
            }

            int32_t delay_angle_q8 = 60 * 256 - ((MotorParam.PHADV_MAX_ANGLE * speed_delta) >> 6);

            /* commutation_delay = (delay_angle_q8 *  Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum )/(360<<8); */
            /* Calculation to avoid division: 1/(360<<8) = 1/(45<<11) = 91/(4096<<11) = 91>>23 */
            /* So we get commutation_delay = (delay_angle_q8 * 91 *  (Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum >> 8)) >> 15; */
            /* Take care not to overflow since Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum may be a large value */
            commutation_delay = ((delay_angle_q8 * 91 *  (Motor0_BLDC_SCALAR_SPEED_POS_HALL.speedaccum >> 8)) >> 15) - commutation_delay;
        }
        else
        {
            /* No phase advance */
            MotorVar.phase_advance_flag = 0;
        }

        /* Stop and clear timer before check whether commutation ISR already serviced */
        PeriPtr.mcmsync_slice->TCCLR = CCU4_CC4_TCCLR_TRBC_Msk |        /* Stop timer */
                                       CCU4_CC4_TCCLR_TCC_Msk;          /* Clear timer */
        if (MotorVar.commutation_flag == 1)
        {
            /* There is pending commutation, do it now! */
            Commutation_ISR();
        }
        PeriPtr.mcmsync_slice->PRS = commutation_delay;
        MotorVar.commutation_flag = 1;      /* Indicate there is pending commutation */
        PeriPtr.ccu4->GCSS = 1 << (PeriPtr.mcmsync_slice_num * 4);  /* Enable Shadow Transfer for Period, Compare and Passive Levels */
        PeriPtr.mcmsync_slice->TCSET = CCU4_CC4_TCSET_TRBS_Msk;     /* Start commutation timer */

        /* To blank the direct DC link current measurement at the commutation point */
        MotorVar.demag_blanking_counter = MotorParam.DEMAG_BLANKING_COUNT;

        /* Reset stall detection counter */
        MotorVar.stall_detection_counter = 0U;
    }

    MotorVar.hall_sector = new_hall_sector;

    /* If requested, update motor speed */
    if (request & UPDATE_SPEED)
    {
        /* Speed calculation */
        uint32_t speed;
        Motor0_BLDC_SCALAR_SPEED_POS_HALL_SpeedCalculation(capture_value, &speed);
        Motor0_BLDC_SCALAR.motor_speed = ((Motor0_BLDC_SCALAR.actual_motor_direction * (int32_t)speed * (int32_t)MotorParam.SPEED_MECH_SCALE) >> BLDC_SCALAR_SPEED_SCALE_RES);
        Motor0_BLDC_SCALAR.abs_motor_speed = abs(Motor0_BLDC_SCALAR.motor_speed);
        XMC_GPIO_ToggleOutput(CYBSP_LED_FG_PORT, CYBSP_LED_FG_PIN);
    }
}
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif

 /**
 * The diag_supress pragma is applied to supress the specified diagnostic message
 * due to the calling of flash function from RAM function.
 * The diag_supress is documented on page 411.
 * On page 395, __ram_func allows to disable the warning.
 * https://wwwfiles.iar.com/arm/webic/doc/ewarm_developmentguide.enu.pdf
*/
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif 
__RAM_FUNC void Commutation_ISR(void)
{
    /*Get the shadow multi-channel pattern value and apply the PWM modulation.*/
    MotorVar.pwm_sector = SetPwmSector(MotorVar.hall_sector, MotorVar.phase_advance_flag);
    uint16_t phase_pattern = SetShadowMcmPattern(MotorVar.pwm_sector);
    set_pwm_phase_pattern(phase_pattern);	
	/* Update the CCU8 compare values */
    Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(MotorVar.amplitude);
	PeriPtr.posif->MCMS = POSIF_MCMS_MNPS_Msk;
    MotorVar.commutation_flag = 0;
}
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif
