/******************************************************************************
* File Name: uCProbe.c
*
* Description: Scheduler functions to handle command from GUI
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
#include <xmc1_flash.h>
#include "01_TopLevel/core_control.h"
#include "02_Lib_6EDL7141/6EDL_gateway.h"
#include "02_GUI_Interface/uCProbe.h"
#include "04_Lib_BLDC/lib_bldc.h"

BLDC_SCALAR_UCPROBE_t Motor0_BLDC_SCALAR_ucprobe;

void Motor0_BLDC_SCALAR_uCPorbe_Update_UI_Var(void)
{
    Motor0_BLDC_SCALAR_ucprobe.dclinkgvoltage_Volt = ((int32_t) (MotorVar.DCLinkVoltageFilter.output * MotorParam.BASE_VOLTAGE * 10) >> 14);
    Motor0_BLDC_SCALAR_ucprobe.motor_current_mA = ((int32_t) (Motor0_BLDC_SCALAR.motor_current * MotorParam.BASE_CURRENT * 1000) >> 14);
    Motor0_BLDC_SCALAR_ucprobe.motor_speed_RPM = ((int32_t) (Motor0_BLDC_SCALAR.motor_speed * MotorParam.BASE_SPEED_MECH_RPM) >> 14);

    if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_STOP)
    {
        Motor0_BLDC_SCALAR_ucprobe.speed_pi_error = (int32_t) (Ramp.output) - (int32_t) (Motor0_BLDC_SCALAR.motor_speed);
        Motor0_BLDC_SCALAR_ucprobe.speed_set_RPM = (int32_t) (Ramp.output * MotorParam.BASE_SPEED_MECH_RPM) >> 14;
        Motor0_BLDC_SCALAR_ucprobe.current_pi_error = (int32_t)Ramp.output - (int32_t)Motor0_BLDC_SCALAR.motor_average_current;
        Motor0_BLDC_SCALAR_ucprobe.current_set_mA = (int32_t)(Ramp.output * MotorParam.BASE_CURRENT *1000) >> 14;
        Motor0_BLDC_SCALAR_ucprobe.voltage_set_Volt = ((int32_t)(Ramp.output * MotorParam.BASE_VOLTAGE * 10) >>14);
    }
    else
    {
        Motor0_BLDC_SCALAR_ucprobe.speed_pi_error = 0;
        Motor0_BLDC_SCALAR_ucprobe.speed_set_RPM = 0;
        Motor0_BLDC_SCALAR_ucprobe.current_pi_error = 0;
        Motor0_BLDC_SCALAR_ucprobe.current_set_mA =0;
        Motor0_BLDC_SCALAR_ucprobe.voltage_set_Volt =0;
    }
}

/*UCproBE scheduler function to handle ucprobe comments from UI */
void Motor0_BLDC_SCALAR_uCProbe_Scheduler(void)
{
    switch (Motor0_BLDC_SCALAR_ucprobe.control_word)
    {
    case 1: /*Received start command*/
        /* ucprobe scheduler doesn't control motor state directly, it only send command to motor control state machine */
        if (Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_STOP)
        {
            MotorVar.Command = MOTOR_START_CMD;
        }
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    case 2: /*Received stop command*/
        MotorVar.Command = MOTOR_STOP_CMD;
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    case 3: /*Clear Error state*/
        MotorVar.fault_clear = 1;
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    case 4: /* Program parameter RAM value into flash */
        /*Erase and write 256 byte of data*/
        XMC_FLASH_ProgramVerifyPage(MotorParameterBlock_Addr, (uint32_t *)&MotorParam); /*Address, data*/
        XMC_FLASH_ProgramVerifyPage(Edl7141ParameterBlock_Addr, (uint32_t *)&Edl7141Reg); /*Address, data*/
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    case 5: /* Erase parameter block in the flash */
        /*Erase 256 byte of data*/
        XMC_FLASH_ErasePage(MotorParameterBlock_Addr);
        XMC_FLASH_ErasePage(Edl7141ParameterBlock_Addr);
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    case 6: /*Learn Hall pattern*/
        if (Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_STOP)
        {
            MotorVar.hall_learning_flag = 1;
        }
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    case 10: /* Reload parameter RAM value from flash */
        if ((Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_STOP) || (Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_ERROR))
        {
            load_motor_flash_parameter();
            SystemVar.ReInitParam = 1;      /* request to re-init peripheral because key parameters are changed */
        }
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;

    default:
        Motor0_BLDC_SCALAR_ucprobe.control_word = 0;
        break;
    } /*End of switch(Motor0_BLDC_SCALAR_ucprobe.control_word)*/
    Motor0_BLDC_SCALAR_uCPorbe_Update_UI_Var();
}
