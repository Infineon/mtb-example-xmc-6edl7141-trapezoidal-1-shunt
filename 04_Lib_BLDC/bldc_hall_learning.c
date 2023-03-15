/******************************************************************************
* File Name: bldc_hall_learning.c
*
* Description: Hall sensor feedback control algorithm
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
#include "04_Lib_BLDC/bldc_scalar_pwm_bc.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define BLDC_SCALAR_HALL_DIRECTION_INDEX            (8U)
#define BLDC_SCALAR_HALL_LEARNING_RETRY_COUNT       (10U)
#define BLDC_SCALAR_HALL_CLOSE_LOOP_ARRAY_SIZE      (7U)
#define BLDC_SCALAR_HALL_MCM_SYNC_UPDATE_COUNT      (3U)
#define BLDC_SCALAR_HALL_LEARNING_OPEN_LOOP_COUNT   (14U)   /*!< Number of tries to capture hall pattern */

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/
/**
 * @brief control parameters of the hall learning algorithm.
 */
typedef struct BLDC_SCALAR_HALL_LEARNING
{
    uint8_t adapt_algoindex;          /*!< Number of pattern count in which motor runs in open loop. Maximum value of this is equal to openloop_count.*/
    uint8_t pwm_sector;
    uint8_t retry_counter;            /*!< This is the number of iteration count in which motor run in open loop.*/
    uint16_t counter;                 /*!< Number of intermediate value in open loop.*/
    uint8_t capt_hallpatt[14];        /*!< This array capture the 14 sample of hall pattern in open loop.*/
} BLDC_SCALAR_HALL_LEARNING_t;

BLDC_SCALAR_HALL_LEARNING_t Motor0_BLDC_SCALAR_HallLearning =
{
    .capt_hallpatt = { 0 },
};

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
/* Hall learning */
void hall_learning_init(void)
{
    Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex = 0;
    Motor0_BLDC_SCALAR_HallLearning.pwm_sector = 0;
    Motor0_BLDC_SCALAR_HallLearning.counter = 0;
    Motor0_BLDC_SCALAR_HallLearning.retry_counter = 0;
}

static void Hall_Learning_ValidationAndUpdate(void)
{
    /* Validates the open loop captured pattern, start from 1 because first entry is not useful */
    for (uint32_t iter_count = 1; iter_count < BLDC_SCALAR_HALL_DIRECTION_INDEX; iter_count++)
    {
        /* Validation fails if no repetitive pattern found */
        if ((Motor0_BLDC_SCALAR_HallLearning.capt_hallpatt[iter_count] != Motor0_BLDC_SCALAR_HallLearning.capt_hallpatt[iter_count + 6]) |
            (Motor0_BLDC_SCALAR_HallLearning.capt_hallpatt[iter_count] == Motor0_BLDC_SCALAR_HallLearning.capt_hallpatt[iter_count + 1]))
        {
            Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex = 0;   /* pattern is not valid, reset adapt_algoindex counter */
        }
    }

    if (Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex == 0) /* captured pattern is invalid */
    {
        if (++Motor0_BLDC_SCALAR_HallLearning.retry_counter >= BLDC_SCALAR_HALL_LEARNING_RETRY_COUNT)
        {
            /* Failed too many times, set error bit and stop hall learning */
            MotorVar.error_status |= 1 << BLDC_SCALAR_EID_HALL_LEARNING_TIMEOUT;
            MotorVar.hall_learning_flag = 0;    /* hall learning is finished */
        }
    }
    else /* captured pattern is validated */
    {
        /* Update pattern array to be used for user configuration update */
        uint8_t hall_sector_table[8] = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F}; /* it stores results from pwm output 120°/180°/240°/300°/0°/60° */
        for (uint32_t i=1; i<7; i++) /* start from 1 because first entry is not useful */
        {
            uint8_t hall_sector;
            hall_sector = i - 1;
            hall_sector_table[Motor0_BLDC_SCALAR_HallLearning.capt_hallpatt[i]] = hall_sector;
        }
        uint32_t hall_sector_map = 0;
        for (uint32_t i=0; i<8; i++)
        {
            hall_sector_map += hall_sector_table[i] << (i*4);
        }
        MotorParam.HALL_SECTOR_MAP = hall_sector_map;
    }
}

/* Hall learning, adaptive Pattern Capture*/
void Motor0_BLDC_SCALAR_Hall_Learning(void)
{
    if (Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex < BLDC_SCALAR_HALL_LEARNING_OPEN_LOOP_COUNT)
    {
        /*
         * Read the hall state
         * Update the hall and multi-channel pattern in captured array
         * Apply next multi-channel pattern
         */
        if (Motor0_BLDC_SCALAR_HallLearning.counter == 0)
        {
            /* save latest hall pattern and output pwm sector into table */
            Motor0_BLDC_SCALAR_HallLearning.capt_hallpatt[Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex] = MotorVar.hall_pattern;

            Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex++;
            if (Motor0_BLDC_SCALAR_HallLearning.adapt_algoindex >= BLDC_SCALAR_HALL_LEARNING_OPEN_LOOP_COUNT)
            {
                Hall_Learning_ValidationAndUpdate();
            }
            else
            {
                Motor0_BLDC_SCALAR_HallLearning.counter = MotorParam.HALL_LEARNING_OL_COUNT;

                uint8_t pwm_sector = Motor0_BLDC_SCALAR_HallLearning.pwm_sector;
                if (Motor0_BLDC_SCALAR_HallLearning.pwm_sector == 5)
                    Motor0_BLDC_SCALAR_HallLearning.pwm_sector = 0;
                else
                    Motor0_BLDC_SCALAR_HallLearning.pwm_sector++;
                uint8_t pwm_sector_prev;
                if (pwm_sector == 0)
                {
                    pwm_sector_prev = 5;
                }
                else
                {
                    pwm_sector_prev = pwm_sector - 1;
                }

                /* multi-channel pattern used to lock the rotor */
                uint16_t phase_pattern = SetShadowMcmPattern(pwm_sector) | SetShadowMcmPattern(pwm_sector_prev);
                PeriPtr.posif->MCSM = phase_pattern & 0x3333;
                PeriPtr.posif->MCMS = POSIF_MCMS_STMR_Msk;      /* Request immediate multi-channel shadow transfer */

                uint16_t mcmval = PeriPtr.posif->MCM;           /* Read back actual multi-channel pattern */
                set_pwm_phase_pattern(mcmval);

                /*update open loop duty cycle*/
                Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(MotorParam.HALL_LEARNING_OL_DUTY);
            }
        }
        else
        {
            Motor0_BLDC_SCALAR_HallLearning.counter--;
        }
    }
    else
    {
        MotorVar.hall_learning_flag = 0;    /* hall learning is finished */
    }
}
