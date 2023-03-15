/******************************************************************************
* File Name: parameter_bldc.c
*
* Description: Initialization of user parameter input to code example's 
*              internal structure variable
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
#include <stdint.h>
#include "01_TopLevel/user_input_config.h"
#include "01_TopLevel/core_control.h"

#if (PROGRAM_DEFAULT_PARAM == 1)

#define POWER_10                           (1024U)  
#define POWER_14                           (16384U) 
#define POWER_15                           (32768U)
#define POWER_16                           (65536U)

void parameter_set_default(void)
{
	MotorParam.ParamVersion = MOTOR_PARAM_VERSION;

/*********************************************************************************************************************
 * Hardware group
********************************************************************************************************************/
	/* HwConfig */
#if (XMC_ON_CHIP_GAIN == 1)
	MotorParam.InternalGain = 0;
#elif (XMC_ON_CHIP_GAIN == 3)
	MotorParam.InternalGain = 1;
#elif (XMC_ON_CHIP_GAIN == 6)
	MotorParam.InternalGain = 2;
#elif (XMC_ON_CHIP_GAIN == 12)
	MotorParam.InternalGain = 3;
#else //default, if user assigns an invalid value
	MotorParam.InternalGain = 0;
#endif

/*********************************************************************************************************************
 * System group
 ********************************************************************************************************************/
	MotorParam.Vadc_Ref_Voltage = VADC_REF_VOLTAGE * 10.0F;
	/* SysConfig */
	MotorParam.AnalogControl = ENABLE_ANALOG_CONTROL;
	MotorParam.ControlScheme = CONTROL_SCHEME;
	MotorParam.EnableCatchSpin = ENABLE_CATCH_SPIN;
	MotorParam.PwmModulationType = PWM_MODULATION_TYPE;
	MotorParam.DcBusCompensation = DC_BUS_COMPENSATION;

/*********************************************************************************************************************
 * Protection group
 ********************************************************************************************************************/
	/* EnableFault */
	MotorParam.EnableFault.Trap = TRAP_PROTECTION;
	MotorParam.EnableFault.WrongHall = WRONG_HALL_PROTECTION;
	MotorParam.EnableFault.HallLearning = HALL_LEARNING_PROTECTION;
	MotorParam.EnableFault.Stall = STALL_PROTECTION;
	MotorParam.EnableFault.OverVoltage = OVER_VOLTAGE_PROTECTION;
	MotorParam.EnableFault.UnderVoltage = UNDER_VOLTAGE_PROTECTION;
	MotorParam.EnableFault.SpiError = SPI_ERROR_PROTECTION;
	MotorParam.EnableFault.GdReset = GD_RESET_PROTECTION;

/*********************************************************************************************************************
 * Motor group
 ********************************************************************************************************************/
	MotorParam.MAX_CURRENT = (uint16_t)((1000.0F * (VADC_REF_VOLTAGE - (CURRENT_AMPLIFIER_OFFSET * (float)XMC_ON_CHIP_GAIN))) / (CURRENT_RSHUNT * CURRENT_AMPLIFIER_GAIN * (float)XMC_ON_CHIP_GAIN));
	MotorParam.BASE_SPEED_MECH_RPM = MOTOR_NO_LOAD_SPEED;	//
	MotorParam.BASE_VOLTAGE = NOMINAL_DC_LINK_VOLT;
	MotorParam.BASE_CURRENT = MotorParam.MAX_CURRENT;
	MotorParam.PWM_FREQUENCY_10HZ = PWM_FREQUENCY_KHZ * 100.0F;
	MotorParam.RISING_DEAD_TIME = PWM_RISING_DEAD_TIME * PWM_TIMER_FREQ_MHZ;
	MotorParam.FALLING_DEAD_TIME = PWM_FALLING_DEAD_TIME * PWM_TIMER_FREQ_MHZ;
	MotorParam.CURRENT_SCALE = (uint32_t)((float)POWER_15 * MotorParam.MAX_CURRENT/(float)POWER_14);
	MotorParam.VOLTAGE_SCALE = (POWER_15 * NOMINAL_DC_LINK_VOLT) / POWER_14;
	MotorParam.SPEED_SCALE = ((float)POWER_15 * (float)MOTOR_NO_LOAD_SPEED)/(float)POWER_14;
	MotorParam.SPEED_MECH_SCALE = (POWER_15 * POWER_10)/((uint32_t)MotorParam.SPEED_SCALE * MOTOR_POLE_PAIRS);
	MotorParam.VOLTAGE_ADC_SCALE = (uint32_t)((4.0F * VADC_REF_VOLTAGE) / (NOMINAL_DC_LINK_VOLT / (VOLTAGE_DIVIDER_R_HIGH / VOLTAGE_DIVIDER_R_LOW + 1)) * POWER_14);
	MotorParam.CURRENT_ADC_SCALE = (uint32_t)(4.0F * (VADC_REF_VOLTAGE / (VADC_REF_VOLTAGE - CURRENT_AMPLIFIER_OFFSET)) * (float)POWER_14);
	MotorParam.RAMP_INITIAL_VALUE = (uint32_t)((CONTROL_RAMP_INITIAL_VALUE / 100.0F) * (float)POWER_14);
	int32_t ramp_up_time_ms = RAMP_UP_TIME * 1000U;
	if (ramp_up_time_ms == 0)
        MotorParam.RAMP_UP_RATE = 0;
	else
        MotorParam.RAMP_UP_RATE = POWER_14 * POWER_16 / ramp_up_time_ms;
	int32_t ramp_down_time_ms = RAMP_DOWN_TIME * 1000U;
	if (ramp_down_time_ms == 0)
        MotorParam.RAMP_DOWN_RATE = 0;
	else
        MotorParam.RAMP_DOWN_RATE = POWER_14 * POWER_16 / ramp_down_time_ms;
	MotorParam.RAMP_DOWN_VOLTAGE_LIMIT = (uint32_t)((RAMP_DOWN_MAX_VOLTAGE / NOMINAL_DC_LINK_VOLT) * POWER_14);
	MotorParam.MAX_SPEED = (MOTOR_NO_LOAD_SPEED * POWER_15) / MotorParam.SPEED_SCALE;
	MotorParam.CURRENT_TRIGGER = CURRENT_TRIGGER_DELAY * PWM_TIMER_FREQ_MHZ;
	MotorParam.OVER_VOLTAGE_THRESHOLD = (uint16_t)((DC_LINK_OVER_VOLTAGE_LEVEL / NOMINAL_DC_LINK_VOLT) * POWER_14);
	MotorParam.UNDER_VOLTAGE_THRESHOLD = (uint16_t)((DC_LINK_UNDER_VOLTAGE_LEVEL / NOMINAL_DC_LINK_VOLT) * POWER_14);
	MotorParam.STALL_DETECTION_TIME = (uint16_t)(STALL_DETECTION_TIME_DEFAULT * 1000.0F);
	MotorParam.STALL_MIN_AMPLITUDE = (uint16_t)((POWER_14 * STALL_MIN_AMPLITUDE_DEFAULT) / 100.0F);
	MotorParam.ANALOG_LOW_LIMIT = (uint16_t)((ANALOG_INPUT_LOW_LIMIT / 100.0F) * POWER_14);
	MotorParam.BOOTSTRAP_COUNT = BOOTSTRAP_TIME;
	MotorParam.MAX_DUTY_CYCLE = (CONTROL_MAX_DUTY_CYCLE * (float)POWER_14) / 100.0F;
	MotorParam.DEMAG_BLANKING_COUNT = DEMAG_BLANKING_TIME / (1000.0F / PWM_FREQUENCY_KHZ);
	MotorParam.COMMUTATION_DELAY = COMMUTATION_DELAY_TIME * CCU4_FREQUENCY_MHZ;
	MotorParam.PHADV_START_SPEED = (PHASE_ADVANCE_START_SPEED << 14) / MOTOR_NO_LOAD_SPEED;    /* to convert rpm to motorspeed */
	MotorParam.PHADV_START_SCALE = (MOTOR_NO_LOAD_SPEED << 14) / (MOTOR_NO_LOAD_SPEED - PHASE_ADVANCE_START_SPEED);    /* to scale phase advance from start speed */
	MotorParam.PHADV_MAX_ANGLE = PHASE_ADVANCE_MAX_ANGLE;
	MotorParam.SPEED_CONTROL_RATE = SPEED_EXECUTION_RATE;
    MotorParam.CURRENT_CONTROL_RATE = CURRENT_EXECUTION_RATE;
	MotorParam.SPEED_KP = SPEED_KP_VALUE;
	MotorParam.SPEED_KI = SPEED_KI_VALUE * MotorParam.SPEED_CONTROL_RATE;
	MotorParam.SPEED_PI_LIMIT = SPEED_PI_LIMIT_VALUE * (float)POWER_14 / 100.0F;
	MotorParam.CURRENT_KP = CURRENT_KP_VALUE;
	MotorParam.CURRENT_KI = CURRENT_KI_VALUE * MotorParam.CURRENT_CONTROL_RATE;
	MotorParam.CURRENT_PI_LIMIT = CURRENT_PI_LIMIT_VALUE * (float)POWER_14 / 100.0F;
	MotorParam.VDC_FILTER_GAIN = (uint16_t)((float)POWER_14 / (VDC_FILTER_TIMECONST * PWM_FREQUENCY_KHZ));
	MotorParam.IDC_FILTER_GAIN = (uint16_t)((float)POWER_14 / (IDC_FILTER_TIMECONST * PWM_FREQUENCY_KHZ));
	MotorParam.HALL_LEARNING_OL_DUTY = (uint16_t)(((HALL_LEARNING_VOLTAGE * NOMINAL_DC_LINK_VOLT / 100) * POWER_15) / MotorParam.VOLTAGE_SCALE);
	MotorParam.HALL_LEARNING_OL_COUNT = HALL_LEARNING_TPULSE;
	MotorParam.HALL_SECTOR_MAP = HALL_LEARNING_PATTERN;  /* hall pattern --> hall sector */
}
#endif


void bldc_derived_parameter_init(void)
{
    /* Initialize potentiometer low pass filter filter */
	MotorVar.PotentiometerFilter.sum = 0;

	/* Initialization of DC link voltage */
    MotorVar.DCLinkVoltageFilter.output = (MotorParam.OVER_VOLTAGE_THRESHOLD + MotorParam.UNDER_VOLTAGE_THRESHOLD) / 2;	/* init to the middle of under voltage and over voltage threshold */

    /* Initialization of user input into structure of ccu8*/
    uint32_t pwm_period = (SystemCoreClock * 2) / (MotorParam.PWM_FREQUENCY_10HZ * 10);
    MotorVar.PWM_PERIOD = pwm_period;
    uint16_t timer_period = pwm_period / 2 - 1;
    MotorVar.PWM_TIMER_RELOAD = timer_period;
    MotorVar.compareval[0] = 0;							/* compare value of phase with 100% duty */
    MotorVar.compareval[1] = timer_period + 1;			/* compare value of phase with 0% duty */

    MotorVar.error_status = 0;
    SystemVar.OffsetCalibrated = 0;		/* clear bit, will trigger offset calibration in STOP state */
    MotorVar.hall_learning_flag = 0;
}

void update_parameter(void)
{
	MotorVar.PwmScheme = MotorParam.PwmModulationType;

    /* update dead time */
    PeriPtr.ccu8_slice[0]->DC1R = MotorParam.RISING_DEAD_TIME << CCU8_CC8_DC1R_DT1R_Pos |
                        MotorParam.FALLING_DEAD_TIME << CCU8_CC8_DC1R_DT1F_Pos;
    PeriPtr.ccu8_slice[1]->DC1R = MotorParam.RISING_DEAD_TIME << CCU8_CC8_DC1R_DT1R_Pos |
                        MotorParam.FALLING_DEAD_TIME << CCU8_CC8_DC1R_DT1F_Pos;
    PeriPtr.ccu8_slice[2]->DC1R = MotorParam.RISING_DEAD_TIME << CCU8_CC8_DC1R_DT1R_Pos |
                        MotorParam.FALLING_DEAD_TIME << CCU8_CC8_DC1R_DT1F_Pos;
}
