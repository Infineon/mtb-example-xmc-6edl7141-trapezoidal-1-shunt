/******************************************************************************
* File Name: user_input_config.h
*
* Description: Control algorithm parameters which user need to configure.
*              By default, these are configured for EVAL_6EDL7141_TRAP_1SH
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

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "01_TopLevel/core_control.h"
#include "03_Application_BLDC/application_bldc.h"

/*********************************************************************************************************************
 *                                    Default value of Trapezoidal motor control
 * Motor type tested: QMotor QBL4208_61_04_013
 *     https://www.trinamic.com/products/drives/bldc-motors-details/qbl4208/
 ********************************************************************************************************************/
/****************************
 * Hardware group
 ***************************/
#define	NOMINAL_DC_LINK_VOLT                        (24.0F)             /* Nominal DC link voltage */
#define	XMC_ON_CHIP_GAIN                            (1)                 /* XMC1400 VADC internal gain for current sensing */
#define VDC_FILTER_TIMECONST                        (0.5F)              /* DC link voltage low pass filter time constant in mSec */
#define IDC_FILTER_TIMECONST                        (100.0F)            /* DC link current low pass filter time constant in mSec */
#define VADC_REF_VOLTAGE                            (5.0F)              /* Reference voltage of VADC */
#define CURRENT_AMPLIFIER_OFFSET                    (1.25F)             /* Current sense amplifier output voltage at zero current */
#define CURRENT_RSHUNT                              (0.5F)              /* Current amplifier shunt resistor value in mOhms */
#define CURRENT_AMPLIFIER_GAIN                      (12.0F)             /* Current amplifier gain */
#define VOLTAGE_DIVIDER_R_HIGH                      (75.0F)             /* DC link voltage sense resistor divider high side value, in kOhm. 56K for 1S eval board, 75K for 1S demo board */
#define VOLTAGE_DIVIDER_R_LOW                       (7.87F)             /* DC link voltage sense resistor divider low side value, in kOhm. 7.5K for 1S eval board, 7.87K for 1S demo board */

/****************************
 * System group
 ***************************/
#define ENABLE_ANALOG_CONTROL                       (ENABLED)               /* Use potentiometer to control motor on/off and speed */
#define CONTROL_SCHEME                              (SPEED_CONTROL)         /* Select motor control scheme, VOLTAGE_CONTROL, SPEED_CONTROL, CURRENT_CONTROL, SPEED_CURRENT_CONTROL */
#define ENABLE_CATCH_SPIN                           (DISABLED)              /* Catch start a spinning motor */
#define PWM_MODULATION_TYPE                         (PWM_HIGHSIDE_120_DEG)  /* PWM modulation scheme selection, PWM_HIGHSIDE_120_DEG, PWM_LOWSIDE_120_DEG or PWM_HIGHSIDE_60_DEG */
#define BOOTSTRAP_TIME                              (0.0F)                  /* Bootstrap capacitor charging time for each leg in mSec, set 0 to disable bootstrap */
#define ANALOG_INPUT_LOW_LIMIT                      (5.0F)                  /* Minimum potentiometer input in order for motor to start, in % of full input */

/****************************
 * Protection group
 ***************************/
#define	STALL_DETECTION_TIME_DEFAULT                (1.2F)              /* Detection time for stall protection in Sec, set to 0 disable stall protection */
#define	STALL_MIN_AMPLITUDE_DEFAULT                 (10U)               /* When amplitude of set point is below this value, stall detection is disabled */
#define DC_LINK_OVER_VOLTAGE_LEVEL                  (30.0F)             /* DC link over voltage level, in volts */
#define DC_LINK_UNDER_VOLTAGE_LEVEL                 (16.0F)             /* DC link under voltage level, in volts */
#define TRAP_PROTECTION                             (DISABLED)          /* TRAP over current protection, motor stops immediately by hardware */
#define WRONG_HALL_PROTECTION                       (ENABLED)           /* Wrong hall protection */
#define HALL_LEARNING_PROTECTION                    (ENABLED)           /* Hall learning timeout protection */
#define STALL_PROTECTION                            (ENABLED)           /* Motor stall protection */
#define OVER_VOLTAGE_PROTECTION                     (ENABLED)           /* DC link over voltage protection */
#define UNDER_VOLTAGE_PROTECTION                    (ENABLED)           /* DC link under voltage protection */
#define SPI_ERROR_PROTECTION                        (ENABLED)           /* 6EDL7141 SPI communication timeout protection */
#define GD_RESET_PROTECTION                         (DISABLED)          /* 6EDL7141 reset protection */

/****************************
 * Motor group
 ***************************/
#define MOTOR_NO_LOAD_SPEED                         (4000U)             /* Motor maximum speed */
#define MOTOR_POLE_PAIRS                            (4U)                /* Motor pole pairs (number of poles/2) */

/****************************
 * PWM group
 ***************************/
#define PWM_FREQUENCY_KHZ                           (20.0F)             /* PWM switching frequency. Range: 1 to 100 kHz */
#define PWM_RISING_DEAD_TIME                        (0.75F)             /* Dead time for rising edge in uSec*/
#define PWM_FALLING_DEAD_TIME                       (0.75F)             /* Dead time for falling edge in uSec*/
#define DC_BUS_COMPENSATION                         (DISABLED)          /* DC bus compensation for PWM output */
#define CURRENT_TRIGGER_DELAY                       (0.75F)             /* Current sense ADC trigger delay from center of PWM on time */
#define CONTROL_MAX_DUTY_CYCLE                      (100.0F)            /* Maximum PWM output duty cycle in % */
#define DEMAG_BLANKING_TIME                         (100.0F)            /* Demagnitization blanking time for skipping DC link current measure after PWM commutation */
#define COMMUTATION_DELAY_TIME                      (10.0F)             /* Minimum time delay from hall event to multi-channel pattern update */
#define PHASE_ADVANCE_START_SPEED                   (1000U)             /* Phase advance starting speed, advance angle increase linearly above this speed */
#define PHASE_ADVANCE_MAX_ANGLE                     (30U)               /* Maximum phase advance angle (at Maximum speed), set 0 to disable phase advance */

/****************************
 * Control Loop group
 ***************************/
#define CONTROL_RAMP_INITIAL_VALUE                  (5.0F)              /* Initial control reference from ramp output in % of full range */
#define RAMP_UP_TIME                                (0.5F)              /* Ramp up time from 0 to maximum, in seconds */
#define RAMP_DOWN_TIME                              (0.5F)              /* Ramp down time from maximum to 0, in seconds */
#define RAMP_DOWN_MAX_VOLTAGE                       (28.0F)             /* Maximum DC link voltage that allows ramp down, ramp down will be hold if DC link voltage above the threshold */
#define SPEED_EXECUTION_RATE                        (1U)                /* Speed control execution rate in number of PWM cycles */
#define CURRENT_EXECUTION_RATE                      (1U)                /* Current control execution rate in number of PWM cycles */
#define SPEED_KP_VALUE                              (200U)              /* Proportional gain of speed PI in Q15 */
#define SPEED_KI_VALUE                              (10U)               /* Integral gain of speed PI in Q15. Range: 0 to 32767 */
#define SPEED_PI_LIMIT_VALUE                        (100.0F)            /* Speed PI regulator voltage output limit */
#define CURRENT_KP_VALUE                            (1000U)             /* Proportional gain of current PI in Q15 */
#define CURRENT_KI_VALUE                            (200U)              /* Integral gain of current PI in Q15 */
#define CURRENT_PI_LIMIT_VALUE                      (100.0F)            /* Current PI regulator output limit in % of base voltage */

/****************************
 * Hall sensor group
 ***************************/
#define HALL_LEARNING_VOLTAGE                       (10.0F)             /* Open loop voltage to be applied during hall learning, in % of nominal voltage */
#define HALL_LEARNING_TPULSE                        (50U)               /* Pulse time duration of each sector during hall learning, in mSec */
#define HALL_LEARNING_PATTERN                       (0xF510342F)        /* Mapping from hall sensor input pattern (range 1-6) to hall sector number (range 0-5) */
