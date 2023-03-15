/******************************************************************************
* File Name: core_control.c
*
* Description: This file contains the sysTick API for update of gate driver registers value and control loop ISR for system monitoring
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

#include "xmc1_flash.h"
#include "01_TopLevel/core_control.h"
#include "02_Lib_6EDL7141/6EDL_gateway.h"
#include "02_GUI_Interface/probe_scope.h"
#include "03_Application_BLDC/application_bldc.h"
#include "04_Lib_BLDC/lib_bldc.h"

mc_info_t MC_INFO;          /* MC_INFO data structure */
system_var_t SystemVar;

static uint16_t StatusLedCounter = 0;       /* 1ms counter for status LED toggling */

/**
 * @addtogroup Interrupt Interrupts
 * @brief  Interrupt Service Routines  <br>
 * @{
 */
/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Systick event handler for motor state machine \n
 * This is the lowest priority interrupt which handles the state transitions and also
 * performs less time critical tasks like ramp, potentiometer reading.\n
 */
void SysTick_Handler(void)
{
    SystemVar.GlobalTimer++;  /* this timer reaches 0xFFFFFFFF after 49.7 days */

    read_inverter_temperature();

    /* Call motor control state machine */
    Motor0_BLDC_SCALAR_MSM();

    /* Status LED toggles at 0.5Hz (no fault) or 4Hz (fault) */
    if (StatusLedCounter > 0)
    {
        StatusLedCounter--;
    }
    else
    {
        uint16_t toggle_period;
        if (MotorVar.MaskedFault.Value == 0)
        {
            toggle_period = 1000;
        }
        else
        {
            toggle_period = 125;
        }
        StatusLedCounter = toggle_period;
        XMC_GPIO_ToggleOutput(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
    }

    /* GUI communication handling */
    edl7141_update();
}


/**
 * @param None
 * @return none <br>
 *
 * \par<b>Description:</b><br>
 * CCU8 one match interrupt.\n
 * Control loop interrupt is based on the PWM frequency.\n
 * This ISR executes the control scheme (PI), direction control, current and voltage reading,
 * voltage compensation and updates the duty cycle of the PWM.
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
__RAM_FUNC void Motor0_BLDC_SCALAR_ControlLoop_ISR(void)
{
    /***************** Current Reading ***************************************/
    /* Get the IDC Link instantaneous current value */
    Motor0_BLDC_SCALAR_GetCurrentValue();
    Motor0_BLDC_SCALAR.motor_current = MotorVar.MotorCurrent;

    /***************** DC link voltage reading ***************************************/
    /* Get the DC link voltage value */
    uint32_t dcvoltage;
    dcvoltage = (get_adc_vbus_result_xmc14() * MotorParam.VOLTAGE_ADC_SCALE) >> 14;

    /* Low pass filter */
    MotorVar.DCLinkVoltageFilter.sum += ((dcvoltage - MotorVar.DCLinkVoltageFilter.output) * MotorParam.VDC_FILTER_GAIN) << 2;

    /* Check if Under/Over voltage protection is enabled */
    /* Check whether the measured dc link voltage is not in set voltage limit */
    if (MotorVar.DCLinkVoltageFilter.output > MotorParam.OVER_VOLTAGE_THRESHOLD)
    {
        MotorVar.error_status |= (1U << BLDC_SCALAR_EID_OVER_VOLTAGE);
    }
    if (MotorVar.DCLinkVoltageFilter.output < MotorParam.UNDER_VOLTAGE_THRESHOLD)
    {
        MotorVar.error_status |= (1U << BLDC_SCALAR_EID_UNDER_VOLTAGE);
    }

    /*****************Duty cycle and direction control**************************/
    /* Call control loop */
    SystemVar.ControlLoop();

#if (UCPROBE_ENABLE == 1)
    ProbeScope_Sampling();
#endif
}
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif


void load_motor_flash_parameter(void)
{
    /* MC_INFO structure contains expected parameter version for the firmware. Head of parameter block should contain
     * same value, otherwise parameter block won't be loaded
     */
#if (PROGRAM_DEFAULT_PARAM == 1)
    /* For test only, program the parameter block with default value */
    /* Configure with default parameter */
    parameter_set_default();
    /* Write default parameter into parameter block */
    XMC_FLASH_ProgramVerifyPage(MotorParameterBlock_Addr, (uint32_t *)&MotorParam);
    SystemVar.MotorConfigured = 1;
#else
    uint16_t parameter_version = (uint16_t)*MotorParameterBlock_Addr;
    if (parameter_version != MC_INFO.parameter_version)
    {
        /* If there is no valid parameter block */
        /* Do nothing, state machine will stay at IDLE state, until parameter is configured by master control */
        SystemVar.MotorConfigured = 0;
    }
    else
    {
        /* Found parameter block with matching version, copy parameter block data into MotorParam structure */
        uint8_t *parameter_ptr = (uint8_t *)&MotorParam;
        uint8_t *flash_ptr = (uint8_t *)MotorParameterBlock_Addr;
        for (uint32_t i=0; i<MOTOR_PARAM_SIZE; i++)
        {
            *parameter_ptr = *flash_ptr;
            parameter_ptr++;
            flash_ptr++;
        }
        SystemVar.MotorConfigured = 1;
    }
#endif
}
