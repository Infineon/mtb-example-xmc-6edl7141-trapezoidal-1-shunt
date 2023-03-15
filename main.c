/******************************************************************************
* File Name: main.c
*
* Description: main file of the trapezoidal 6EDL7141 shunt application
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

/***********************************************************************************************************************
 * Hardware setup for Trapezoidal control with Hall test
 **********************************************************************************************************************/
/* Microcontroller:          XMC1404-Q064x0128
 * Evaluation board:         EVAL_6EDL7141_TRAP_1SH
 * Hall connection:          HALL0 (Green)     -> P4.1
 *                           HALL1 (White)     -> P4.2
 *                           HALL2 (Black)     -> P4.3
 * Motor QBL4208-61-04-013 connection: Yellow  -> U of inverter
 *                                     Red     -> V
 *                                     Black   -> W
 */
#include "cybsp.h"
#include "cy_utils.h"
#include "01_TopLevel/core_control.h"

extern void Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func(void);
extern void load_edl7141_flash_parameter(void);
extern void edl7141_gateway_init(void);

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
/**
 @brief
 This example project controls 3-phase BLDC motor with 3 hall sensor feedback using trapezoidal control algorithm.
 This is configured for EVAL_6EDL7141_TRAP_1SH board, tested with Trinamic motor part number QBL4208-61-04-013

 A list of APIs is provided to control the motor and change the configurations
 - Motor0_BLDC_SCALAR_MotorStart
 - Motor0_BLDC_SCALAR_MotorStop
 Below APIs are for speed control. Similar APIs are available for voltage control and current control.
 - BLDC_SCALAR_SetSpeedVal
 - BLDC_SCALAR_SetSpeedProportionalGain
 - BLDC_SCALAR_SetSpeedIntegralGain
 */

/* Default configuration:
 * EVAL_6EDL7141_TRAP_1SH board, tested with Trinamic motor part number QBL4208-61-04-013
 * Control scheme: SPEED_CONTROL
 * PWM modulation: PWM_HIGHSIDE
 * PWM frequency: 20000Hz
 * Speed set: 2000 RPM
 * Ramp up and ramp down rate: 500 RPM/s
 * Over-current protection with direct DC link current measurement
 * Seamless bi-directional, Hall pattern learning and stall detection are enabled
 */

/* Change of configurations:
 * Control algorithm configurations are in the 01_TopLevel/user_input_config.h file.
 * MCU HW resources configurations are in the 01_TopLevel/peripheral_xmc14.h file.
 */


int main(void)
{
    /*************************************************
     * Initialize the device and bsp configurations
     * **********************************************/
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /************************************************************
     * Initialize MC_INFO data, check chip_id and read kit_id
     * *********************************************************/
    MC_INFO.parameter_version = MOTOR_PARAM_VERSION;
    MC_INFO.firmware_version = FIRMWARE_VERSION;
    MC_INFO.chip_id = check_chip_id_xmc14();     /* return 0 if chip_id is wrong */
    if (MC_INFO.chip_id == 0)
    {
        /* chip_id is wrong */
        MC_INFO.kit_id = 0;     /* set kit_id to 0 (Unidentified) */
        while(1);               /* stay here .... */
    }
    else
    {
        /* chip_id is correct */
        MC_INFO.kit_id = read_kit_id_xmc14();    /* Read hardware kit ID from analog pin */
    }

    /**************************************
     * load motor parameter from flash
     * ***********************************/
    load_motor_flash_parameter();   /* set MotorConfigured to 1 if load was succeed */
    /* load 6EDL7141 parameter from flash, set Edl7141Configured to 1 if load was succeed */
    load_edl7141_flash_parameter();
	/*******************************************************************
     * Initialize SPI interface with 6EDL7141, and 6EDL7141 related IO
     * ****************************************************************/
	edl7141_gateway_init();

    /***************************************************
     * Initialize motor state machine and start systick
     * ************************************************/
    Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func();
    SystemVar.GlobalTimer = 0;		            /* reset timer */
    SysTick_Config(SystemCoreClock / 1000);     /* Systick = 1ms */

    /****************************************************************
     * Don't run application code in main loop. All application
     * code should be handled in ISR
     * *************************************************************/
    while (1U)
    {
        /* do nothing */
    }
}
