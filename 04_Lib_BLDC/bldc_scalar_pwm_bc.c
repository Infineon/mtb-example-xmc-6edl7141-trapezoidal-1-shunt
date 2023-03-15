/******************************************************************************
* File Name: bldc_scalar_pwm_bc.c
*
* Description:Block commutation PWM generation. This supports high side, 
*             low side and high side with synchronous rectification modulation
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

/**
 * Compare register update flag
 */
typedef enum PWM_BC_COMPAREFLAG
{
  PWM_BC_COMPARE_ZERO,        /*!<update compare register with value 0. Duty cycle is 100% */
  PWM_BC_COMPARE_PERIOD,      /*!<update compare register with value (period +1). Duty cycle is 0% */
  PWM_BC_COMPARE_DUTY,        /*!<update compare register with value direct duty. Duty cycle is ((period - duty)/period)% */
  PWM_BC_COMPARE_INVERSE_DUTY /*!<update compare register with value (period - duty). Duty cycle is ((duty)/period)% */
} PWM_BC_COMPAREFLAG_t;
static PWM_BC_COMPAREFLAG_t cmpval_index[3];           /*!< Control flag for compare value update */

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/**
 * Mapping table from pwm_sector to mcm value
 */
static const uint32_t MCM_PATTERN_TABLE[3] =
{
    /*************************************
     * PWM sector 0,   0° -  60°, U->W
     * PWM sector 1,  60° - 120°, V->W
     * PWM sector 2, 120° - 180°, V->U
     * PWM sector 3, 180° - 240°, W->U
     * PWM sector 4, 240° - 300°, W->V
     * PWM sector 5, 300° - 360°, U->V
     * MCM_OFF       = 000(0)   both sides off
     * MCM_HS_ON     = 001(1)   high side 100% on
     * MCM_LS_ON     = 010(2)   low side 100% on
     * MCM_HS_SW     = 101(5)   high side switching
     * MCM_LS_SW     = 110(6)   low side switching
     * MCM_HS_COMP   = 011(3)   high side switching complementary
     * MCM_LS_COMP   = 111(7)   low side switching complementary
     ************************************/
    /* PwmScheme = PWM_HIGHSIDE_120_DEG   (0U), high side 120° complementary switching */
    MCM_LS_ON   << 9*3 | MCM_LS_ON   << 8*3 | MCM_OFF     << 7*3 | MCM_HS_COMP << 6*3 |
    MCM_HS_COMP << 5*3 | MCM_OFF     << 4*3 | MCM_LS_ON   << 3*3 | MCM_LS_ON   << 2*3 | MCM_OFF << 1*3 | MCM_HS_COMP << 0*3,  /* U phase pattern */
    /* PwmScheme = PWM_LOWSIDE_120_DEG    (1U), low side 120° complementary switching */
    MCM_LS_COMP << 9*3 | MCM_LS_COMP << 8*3 | MCM_OFF     << 7*3 | MCM_HS_ON   << 6*3 |
    MCM_HS_ON   << 5*3 | MCM_OFF     << 4*3 | MCM_LS_COMP << 3*3 | MCM_LS_COMP << 2*3 | MCM_OFF << 1*3 | MCM_HS_ON   << 0*3,  /* U phase pattern */
    /* PwmScheme = PWM_HIGHSIDE_60_DEG    (2U), high side 60° complementary switching */
    MCM_LS_ON   << 9*3 | MCM_LS_COMP << 8*3 | MCM_OFF     << 7*3 | MCM_HS_ON   << 6*3 |
    MCM_HS_COMP << 5*3 | MCM_OFF     << 4*3 | MCM_LS_ON   << 3*3 | MCM_LS_COMP << 2*3 | MCM_OFF << 1*3 | MCM_HS_ON   << 0*3,  /* U phase pattern */
};

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
uint8_t SetPwmSector(uint8_t hall_sector, uint8_t phase_advance)
{
    /* Set shadow multi-channel pattern */
    uint8_t pwm_sector;
    if (MotorVar.voltage_command >= 0)
    {
        /* hall sector is 0 (330°-30°) and phase_advance is 0: positive Vq is at middle of pwm sector 1 (90°) */
        /* hall sector is 0 (330°-30°) and phase_advance is 1, positive Vq is at middle of pwm sector 2 (150°) */
        pwm_sector = hall_sector + 1 + phase_advance;
    }
    else
    {
        /* hall sector is 0 (330°-30°) and phase_advance is 0: negative Vq is at middle of pwm sector 4 (270°) */
        /* hall sector is 0 (330°-30°) and phase_advance is 1, negative Vq is at middle of pwm sector 3 (210°) */
        pwm_sector = hall_sector + 4 - phase_advance;
    }
    if (pwm_sector >= 6)
        pwm_sector -= 6;
    return pwm_sector;
}


void set_pwm_phase_pattern(uint16_t mcm_val)
{
    for (uint8_t count = 0; count < CCU8_MAXPHASE_COUNT; count++)
    {
        uint16_t phase_mode = (mcm_val >> (PeriPtr.ccu8_slice_num[count] * 4)) & 0x0F;
        switch (phase_mode)
        {
        case MCM_OFF:
        default:
            break;
        case MCM_HS_ON:
            cmpval_index[count] = PWM_BC_COMPARE_ZERO;
            /* Disable dead time */
            PeriPtr.ccu8_slice[count]->DTC = 0;
            break;
        case MCM_LS_ON:
            cmpval_index[count] = PWM_BC_COMPARE_PERIOD;
            /* Disable dead time */
            PeriPtr.ccu8_slice[count]->DTC = 0;
            break;
        case MCM_HS_SW:
            cmpval_index[count] = PWM_BC_COMPARE_INVERSE_DUTY;
            /* Disable dead time */
            PeriPtr.ccu8_slice[count]->DTC = 0;
            break;
        case MCM_LS_SW:
            cmpval_index[count] = PWM_BC_COMPARE_DUTY;
            /* Disable dead time */
            PeriPtr.ccu8_slice[count]->DTC = 0;
            break;
        case MCM_HS_COMP:
            cmpval_index[count] = PWM_BC_COMPARE_INVERSE_DUTY;
            /* Enable dead time */
            PeriPtr.ccu8_slice[count]->DTC |= CCU8_CC8_DTC_DTE1_Msk | CCU8_CC8_DTC_DCEN1_Msk | CCU8_CC8_DTC_DCEN2_Msk;
            break;
        case MCM_LS_COMP:
            cmpval_index[count] = PWM_BC_COMPARE_DUTY;
            /* Enable dead time */
            PeriPtr.ccu8_slice[count]->DTC |= CCU8_CC8_DTC_DTE1_Msk | CCU8_CC8_DTC_DCEN1_Msk | CCU8_CC8_DTC_DCEN2_Msk;
            break;
        }
    }
}


uint16_t SetShadowMcmPattern(uint8_t pwm_sector)
{
    /* Set shadow multi-channel pattern */
    uint8_t mcm_phase[3];
    uint8_t pwm_scheme = MotorVar.PwmScheme;
    mcm_phase[0] = (MCM_PATTERN_TABLE[pwm_scheme] >> ((pwm_sector + 0)*3)) & 0x07;  /* U phase mcm value */
    mcm_phase[1] = (MCM_PATTERN_TABLE[pwm_scheme] >> ((pwm_sector + 4)*3)) & 0x07;  /* V phase mcm value */
    mcm_phase[2] = (MCM_PATTERN_TABLE[pwm_scheme] >> ((pwm_sector + 2)*3)) & 0x07;  /* W phase mcm value */
    uint16_t phase_pattern = 0;
    phase_pattern += mcm_phase[0] << (PeriPtr.ccu8_slice_num[0]<<2);
    phase_pattern += mcm_phase[1] << (PeriPtr.ccu8_slice_num[1]<<2);
    phase_pattern += mcm_phase[2] << (PeriPtr.ccu8_slice_num[2]<<2);
    PeriPtr.posif->MCSM = phase_pattern & 0x3333;
    return phase_pattern;
}


/* Set all phases to same output duty cycle */
void set_cmpval_all_phases(uint16_t cmpval)
{
    /*Load the duty cycle to the channel compare shadow register*/
    PeriPtr.ccu8_slice[0]->CR1S = cmpval;
    PeriPtr.ccu8_slice[1]->CR1S = cmpval;
    PeriPtr.ccu8_slice[2]->CR1S = cmpval;
    /*Enable the shadow transfer*/
    PeriPtr.ccu8->GCSS = PeriPtr.shadow_transfer;
}


void Motor0_BLDC_SCALAR_PatternInitiator(void)
{
    uint8_t new_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(PeriPtr.posif);
    MotorVar.hall_pattern = new_hall_pattern;
    MotorVar.hall_sector = (MotorParam.HALL_SECTOR_MAP >> (new_hall_pattern << 2)) & 0x0F;
    MotorVar.pwm_sector = SetPwmSector(MotorVar.hall_sector, MotorVar.phase_advance_flag);

    /* Set shadow multi-channel pattern */
    uint16_t phase_pattern = SetShadowMcmPattern(MotorVar.pwm_sector);
    /* Prepare for block commutation for phases which are conducting */
    set_pwm_phase_pattern(phase_pattern);
    PeriPtr.posif->MCMS = POSIF_MCMS_STMR_Msk;      /* Request immediate multi-channel shadow transfer */
}


void Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(int16_t amplitude)
{
  MotorVar.compareval[2] = (amplitude * MotorVar.PWM_TIMER_RELOAD) >> 14;       /* compare value of phase with command duty */
  MotorVar.compareval[3] = MotorVar.PWM_TIMER_RELOAD - MotorVar.compareval[2];  /* compare value of phase with inversed command duty */

  /*Load the duty cycle to the channel compare shadow register*/
  PeriPtr.ccu8_slice[0]->CR1S = MotorVar.compareval[cmpval_index[0]];   /* U phase */
  PeriPtr.ccu8_slice[1]->CR1S = MotorVar.compareval[cmpval_index[1]];   /* V phase */
  PeriPtr.ccu8_slice[2]->CR1S = MotorVar.compareval[cmpval_index[2]];   /* W phase */

  /*Enable the shadow transfer*/
  PeriPtr.ccu8->GCSS = PeriPtr.shadow_transfer;
}


/* This API will be called periodically from state machine during bootstrapping time. */
bool Motor0_BLDC_SCALAR_Bootstrap(void)
{
  bool status = true;

  /*Enable Bootstrap for Phase U*/
  if (MotorVar.bootstrap_counter < MotorParam.BOOTSTRAP_COUNT)
  {
    /*
     * Disable multi-channel mode for phase U so that low side switch of phase U is continuously ON
     * Make sure that compare value of phase U is set greater than period value.
     */
    PeriPtr.ccu8_slice[0]->TC &= ~((uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1);
  }
  /*Enable Bootstrap for Phase V*/
  else if (MotorVar.bootstrap_counter < (MotorParam.BOOTSTRAP_COUNT * 2U))
  {
    /*
     * Disable multi-channel mode for phase V so that low side switch of phase V is continuously ON
     * Make sure that compare value of phase V is set greater than period value.
     */
    PeriPtr.ccu8_slice[0]->TC |= (uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1;
    PeriPtr.ccu8_slice[1]->TC &= ~((uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1);
  }

  /*Enable Bootstrap for Phase W*/
  else if (MotorVar.bootstrap_counter < (MotorParam.BOOTSTRAP_COUNT * 3U))
  {
    /*
     * Disable multi-channel mode for phase W so that low side switch of phase W is continuously ON
     * Make sure that compare value of phase W is set greater than period value.
     */
    PeriPtr.ccu8_slice[1]->TC |= (uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1;
    PeriPtr.ccu8_slice[2]->TC &= ~((uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1);
  }

  else
  {
    /* Enable all phases */
    PeriPtr.ccu8_slice[0]->TC |= (uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1;
    PeriPtr.ccu8_slice[1]->TC |= (uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1;
    PeriPtr.ccu8_slice[2]->TC |= (uint32_t)CCU8_CC8_TC_MCME1_Msk << XMC_CCU8_SLICE_COMPARE_CHANNEL_1;
    set_cmpval_all_phases(0);

    status = false;
  }
  MotorVar.bootstrap_counter++;
  return (status);
}


void bldc_pwm_output(int16_t voltage_command)
{
    MotorVar.voltage_command_prev = MotorVar.voltage_command;
    MotorVar.voltage_command = voltage_command;

    int32_t amplitude;
    if (voltage_command >= 0)
    {
        amplitude = voltage_command;
    }
    else
    {
        amplitude = -voltage_command;
    }
    if (MotorParam.DcBusCompensation == ENABLED)
    {
        /* DC bus voltage compensation */
        int32_t dc_link_voltage = MotorVar.DCLinkVoltageFilter.output; /* DC link voltage */
        /* Adjust the amplitude based upon DC link voltage ripple */
        if (dc_link_voltage > 0)
        {
            /*Do the voltage compensation based on dc link value*/
            amplitude = (amplitude * 16384) / dc_link_voltage;
        }
        else
        {
            amplitude = 0;
        }
    }

    /*
     * To avoid discharging of the bootstrap capacitor in high side modulation,
     * apply 100% voltage if voltage amplitude is greater than amplitude threshold limit
     */
    if (amplitude > MotorParam.MAX_DUTY_CYCLE)
    {
        amplitude = MotorParam.MAX_DUTY_CYCLE;
    }

    MotorVar.amplitude = amplitude;
    /* Update the CCU8 compare values */
    Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(amplitude);
}
