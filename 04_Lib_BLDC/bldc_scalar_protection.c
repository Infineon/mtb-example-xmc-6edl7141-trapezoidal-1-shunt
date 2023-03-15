/******************************************************************************
* File Name: bldc_scalar_protection.c
*
* Description:VADC channel event used for current protection and 
*             voltage protection
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

extern void load_edl7141_flash_parameter(void);
extern void spi_write_6EDL7141_registers(void);
extern void spi_read_6EDL7141_registers(void);

/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Finds if motor is stalled based on stall detection timeout (@ref MOTOR0_BLDC_SCALAR_STALL_DETECTION_TIME_mS) and
 * time difference between last captured value and time since last hall event (@ref STALL_TIME_DIFF_RATIO_DEFAULT)
 */
void Motor0_BLDC_SCALAR_StallDetection(void)
{
    if (MotorParam.STALL_DETECTION_TIME != 0)  /* if stall detection is enabled */
    {
        if (MotorVar.amplitude > MotorParam.STALL_MIN_AMPLITUDE)  /* do stall detection only if output duty cycle is high enough */
        {
            MotorVar.stall_detection_counter++;

            /*
             * No Hall event is detected within the configured stall detection time
             */
            if((MotorVar.stall_detection_counter > MotorParam.STALL_DETECTION_TIME))
            {
                MotorVar.error_status |= ((uint16_t) 1 << (uint16_t) BLDC_SCALAR_EID_STALL);
            }
        }
        else
        {
            MotorVar.stall_detection_counter = 0;
        }
    }


}


/* Common protection is executed in systick */
void motor_common_protection(void)
{
    if (MotorVar.fault_clear == 1)
    {
        MotorVar.fault_clear = 0;
        if (MotorVar.error_status & (1 << BLDC_SCALAR_EID_CTRAP_ERROR))
        {
            /* to set CTRAP trigger level EV2LM to default active high bit 10 */
        	PeriPtr.ccu8_slice[0]->INS2 &= 0xFFFFFBFF; /* to set CTRAP trigger level EV2LM to active high '0' at bit 10 */
            PeriPtr.ccu8_slice[1]->INS2 &= 0xFFFFFBFF; /* to set CTRAP trigger level EV2LM to active high '0' at bit 10 */
            PeriPtr.ccu8_slice[2]->INS2 &= 0xFFFFFBFF; /* to set CTRAP trigger level EV2LM to active high '0' at bit 10 */
        	/* Clear TRAP fault */
			PeriPtr.ccu8_slice[0]->SWR = 0x00000C00;  //RTRPF=1, RE2A=1, clear TRAP and Event 2 flags
			PeriPtr.ccu8_slice[1]->SWR = 0x00000C00;  //RTRPF=1, RE2A=1, clear TRAP and Event 2 flags
			PeriPtr.ccu8_slice[2]->SWR = 0x00000C00;  //RTRPF=1, RE2A=1, clear TRAP and Event 2 flags

		    PeriPtr.posif->MCMC = (1UL << POSIF_MCMC_MNPC_Pos) |      /* Multi-Channel Pattern Update Enable Clear */
		                          (1UL << POSIF_MCMC_MPC_Pos);        /* Multi-Channel Pattern Clear */
        }
        if (MotorVar.error_status & (1 << BLDC_SCALAR_EID_GD_RESET))
        {
            /* If there is GD_RESET fault, reconfigure gate driver parameter */
        	load_edl7141_flash_parameter();
        	spi_write_6EDL7141_registers();
            spi_read_6EDL7141_registers();
        }
        MotorVar.error_status = 0;
    }
    /* Check Trap over current */
    if (PeriPtr.ccu8_slice[0]->INTS & 0x00000C00)
    {
        /* Update the error status and stop the motor in case of trap event */
        MotorVar.error_status |= 1 << BLDC_SCALAR_EID_CTRAP_ERROR;
    }

    /* DC bus over-voltage and under-voltage check are done in fast tic */

    /* Fault signaling */
    MotorVar.MaskedFault.Value = MotorVar.error_status & MotorParam.EnableFault.Value;
}
