/******************************************************************************
* File Name: state_machine_bldc.c
*
* Description: system timer event used for state machine
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
#include "02_Lib_6EDL7141/6EDL_gateway.h"
#include "02_GUI_Interface/uCProbe.h"
#include "04_Lib_BLDC/lib_bldc.h"

extern void bldc_derived_parameter_init(void);
extern void update_parameter(void);

/*********************************************************************************************************************
 * MACROS
 * *******************************************************************************************************************/

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
void Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_IDLE_Func(void);

void Motor0_BLDC_SCALAR_MSM_STOP_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_STOP_Func(void);

void Motor0_BLDC_SCALAR_MSM_OFFSET_CALIBRATION_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_OFFSET_CALIBRATION_Func();

void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Func(void);
void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func(void);

void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Func(void);

void Motor0_BLDC_SCALAR_MSM_ERROR_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_ERROR_Func(void);

void Motor0_BLDC_SCALAR_MSM_HALL_LEARNING_Entry_Func(void);
void Motor0_BLDC_SCALAR_MSM_HALL_LEARNING_Func(void);
/*******************************************************************************
 * BLDC state machine: IDLE state
 *
 ******************************************************************************/
static uint8_t idle_state_delay_counter;    /* have to add extra 3ms delay due to 6EDL7141 watchdog clock, otherwise offset calibration will be off or current sensing doesn't working properly */
/*
 * State entry function - IDLE state
 */
void Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func(void)
{
    /* Point to null control mode code */
    control_loop_reset_ptr();
    idle_state_delay_counter = 0;
    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_IDLE;
}

/*
 * State main function - IDLE state
 */
void Motor0_BLDC_SCALAR_MSM_IDLE_Func(void)
{
    if ((SystemVar.MotorConfigured == 1) && (SystemVar.Edl7141Configured == 1))
    {
        if (idle_state_delay_counter++ == 0)
        {
            /* Calculate derived parameters and variables, base on loaded parameters */
            bldc_derived_parameter_init();

            /* To write the 6EDL7141 configuration registers to default values */
            spi_write_6EDL7141_registers();

            /* Initialization mcu peripheral */
            peripheral_init_xmc14();

            /* Update hall pattern and hall sector, to avoid wrong hall fault after power up */
            Motor0_BLDC_SCALAR_PatternInitiator();

            /* Clear TRAP fault */
            MotorVar.error_status |= 1 << BLDC_SCALAR_EID_CTRAP_ERROR;
            MotorVar.fault_clear = 1;

        #if (UCPROBE_ENABLE == 1)
            Motor0_BLDC_SCALAR_ucprobe.control_word = 0;

            /* The ProbeScope_Init() function is initialized with the 20kHz sampling frequency.
             * This sampling frequency has to match with the sampling frequency in
             * BPA Motor Control GUI. GUI reads the sampling frequency from MotorParam.PWM_FREQUENCY_10HZ.
             * */
            ProbeScope_Init(20000);
        #endif
        }
        else if (idle_state_delay_counter > 3)
        {
            idle_state_delay_counter = 0;
            Motor0_BLDC_SCALAR_MSM_STOP_Entry_Func();
        }
    }
    else
    {
        /* stay in state */
        MotorVar.MaskedFault.ParameterLoad = 1;     /* set fault to let LED blinking fast */
    }
}

/*******************************************************************************
 * BLDC state machine: STOP state
 *
 ******************************************************************************/
/*
 * State entry function - STOP state
 *   Multi-channel pattern as 0x00
 *   Stop the POSIF
 *   Disable hall event, multi-channel shadow transfer event
 *   Disable CCU8 one match event
 */
void Motor0_BLDC_SCALAR_MSM_STOP_Entry_Func(void)
{
    MotorVar.amplitude = 0;
    ramp_reset();

    /* Immediately turn off all PWM, all PWMs in inactive state */
    PeriPtr.posif->MCMC = (1UL << POSIF_MCMC_MNPC_Pos) |    /* Multi-Channel Pattern Update Enable Clear */
                        (1UL << POSIF_MCMC_MPC_Pos);        /* Multi-Channel Pattern Clear */

    /* Point to null control mode code */
    control_loop_reset_ptr();

    Motor0_BLDC_SCALAR_SPEED_POS_HALL_ClearSpeedFilter();

    /* ISR Init for pattern update handler */
    NVIC->ISER[0] = 1<<21 |             /* Enable Commutation interrupt */
                    1<<27;              /* Enable POSIF hall event interrupt */

    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_STOP;
}

/*
 * State main function - STOP state
 *   This is hub state, it goes to different states
 */
void Motor0_BLDC_SCALAR_MSM_STOP_Func(void)
{
    PeriPtr.posif->MCMC = (1UL << POSIF_MCMC_MNPC_Pos) |    /* Multi-Channel Pattern Update Enable Clear */
                        (1UL << POSIF_MCMC_MPC_Pos);        /* Multi-Channel Pattern Clear */
	if (SystemVar.ReInitParam == 1)			/* There is parameter re-init request */
    {
        /* execute re-init routine.... */
        SystemVar.ReInitParam = 0;
    }
    else
    {
        update_parameter();
    }

    /* Handle state transition */
    if (SystemVar.OffsetCalibrated == 0)        /* Need to do current sense offset calibration */
    {
        Motor0_BLDC_SCALAR_MSM_OFFSET_CALIBRATION_Entry_Func();
    }
    else if (MotorVar.hall_learning_flag == 1)  /* Need to do hall learning */
    {
        MotorVar.Command = MOTOR_START_CMD;
        Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func();
    }
    else  /* Start motor if there is command */
    {
        Motor0_BLDC_SCALAR_GetCommand();

        /************************** Low speed handling ***************************/
        SPEED_POS_HALL_LowSpeedCalculation(&Motor0_BLDC_SCALAR.motor_speed);

        if (MotorVar.Command != MOTOR_STOP_CMD)
        {
            Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func();
        }
    }
}

/*******************************************************************************
 * BLDC state machine: OFFSET_CALIBRATION state
 *
 ******************************************************************************/
/*
 * State entry function - OFFSET_CALIBRATION state
 */
void Motor0_BLDC_SCALAR_MSM_OFFSET_CALIBRATION_Entry_Func(void)
{
    EdlIo.en_drv_level = 1;
    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_OFFSET_CALIBRATION;
}

/*
 * State main function - OFFSET_CALIBRATION state
 */
/**
* The diag_supress pragma is applied to supress the specified diagnostic message
* due to the calling of flash function from RAM function.
* The diag_supress is documented on page 411.
* On page 395, __ram_func allows to disable the warning.
*
* https://wwwfiles.iar.com/arm/webic/doc/ewarm_developmentguide.enu.pdf
*/
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
void Motor0_BLDC_SCALAR_MSM_OFFSET_CALIBRATION_Func()
{
    offset_calibration();
    SystemVar.OffsetCalibrated = 1;
    Motor0_BLDC_SCALAR_MSM_STOP_Entry_Func();
}
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif

/*******************************************************************************
 * BLDC state machine: BOOTSTRAP state
 *
 ******************************************************************************/
/*
 * State entry function - BOOTSTRAP state
 *   Compare value = 0xFFFF
 *   Stop POSIF
 *   Disable hall event
 *   Start CCU8
 */
void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func(void)
{
    if (MotorParam.BOOTSTRAP_COUNT == 0)	/* skip bootstrap if bootstrap_count is 0 */
    {
        Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func();
    }
    else
    {
        /* Initialize all run time variables */
        MotorVar.bootstrap_counter = 0U;

        uint16_t BLDC_SCALAR_BOOTSTRAP_CMP_VAL = 0xFFFFU;
        set_cmpval_all_phases(BLDC_SCALAR_BOOTSTRAP_CMP_VAL);

        /*Change Motor Control State Machine to Boot Strap*/
        Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_BOOTSTRAP;
    }
}

/*
 * State main function - BOOTSTRAP state
 *   Wait for the bootstrap time
 */
void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Func(void)
{
    if (Motor0_BLDC_SCALAR_Bootstrap() == false)
    {
      Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func();
    }
}

/*
 * State exit function - BOOTSTRAP state
 *   Next state:
 *   HALL_LEARNING: if hall learning command is received
 *   NORMAL_OPERATION: if motor start command is received
 */
void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func(void)
{
    if (MotorVar.hall_learning_flag == 1)
    {
        /* goto hall learning */
        Motor0_BLDC_SCALAR_MSM_HALL_LEARNING_Entry_Func();
    }
    else
    {
        /* Start motor */
        Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Entry_Func();
    }
}


/*******************************************************************************
 * BLDC state machine: NORMAL_OPERATION state
 *
 ******************************************************************************/
/*
 * State entry function - NORMAL_OPERATION state
 *   Prepare hall and multi-channel pattern and start the POSIF and CCU4
 *   Start the CCU8 for PWM
 *   Disable correct hall event
 *   Enable wrong hall event, multi-channel pattern shadow transfer event and CCU8 one match event
 */
void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Entry_Func(void)
{
    /* Initialize all run time variables */
    Motor0_BLDC_SCALAR.motor_current = 0;
    Motor0_BLDC_SCALAR.motor_average_current = 0;
    MotorVar.demag_blanking_counter = 0;

    /* Initialize ramp by target direction */
    ramp_init();

    /* Reset the PI integral buffer */
    MotorVar.speed_integral = 0;
    MotorVar.speed_sat = 1;
    MotorVar.current_integral = 0;
    MotorVar.current_sat = 1;
    Motor0_BLDC_SCALAR.speedcontrol_rate_counter = 0;

    /* initialize stall check */
    MotorVar.stall_detection_counter = 0;

    /* Set initial pwm pattern */
    Motor0_BLDC_SCALAR_PatternInitiator();

    /* Point to control mode code */
    control_loop_set_ptr();

#if(CURRENT_EXECUTION_RATE > 1U)
    /*
     * Initialize Phase-V period match event and Bind Phase-U period match event SR(service request)
     * to interrupt node for trap.
     */
    BLDC_PWM_Config.phase_ptr[1U]->SRS |= XMC_CCU8_SLICE_SR_ID_1 << CCU8_CC8_SRS_POSR_Pos;
    XMC_CCU8_SLICE_EnableEvent(BLDC_PWM_Config.phase_ptr[1U], XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
#endif/*end of #if(CURRENT_EXECUTION_RATE > 1U)*/

    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_NORMAL_OPERATION;
}

/*
 * State main function - NORMAL_OPERATION state
 *   Potentiometer measurement
 *   Ramp
 *   Stall detection
 *   zero duty condition handling
 */
void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Func(void)
{
    update_parameter();	/* allows changing some parameters on the fly */

    Motor0_BLDC_SCALAR_GetCommand();

    /************************** Low speed handling ***************************/
    SPEED_POS_HALL_LowSpeedCalculation(&Motor0_BLDC_SCALAR.motor_speed);

    /****************************** Ramp ******************************/
    Motor0_BLDC_SCALAR_Ramp_Linear();

    /************************** Stall detection start**************************/
    Motor0_BLDC_SCALAR_StallDetection();

    /* Handle state transition */
    if (MotorVar.Command == MOTOR_STOP_CMD)
    {
        /* Switch off all the PWM outputs when reference is zero and duty is zero */
        Motor0_BLDC_SCALAR_MSM_STOP_Entry_Func();
    }
}

/*******************************************************************************
 * BLDC state machine: ERROR state
 *
 ******************************************************************************/
/*
 * State entry function - ERROR state
 */

/**
 * The diag_supress pragma is applied to supress the specified diagnostic message
 * due to the calling of flash function from RAM function.
 * The diag_supress is documented on page 411
 * https://wwwfiles.iar.com/arm/webic/doc/ewarm_developmentguide.enu.pdf
*/
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif 
void Motor0_BLDC_SCALAR_MSM_ERROR_Entry_Func(void)
{
	/* Immediately turn off all PWM, all PWMs in inactive state */
    PeriPtr.ccu8_slice[0]->INS2 |= 0x00000400; /* simulate TRAP error by set CTRAP trigger level EV2LM to active low bit 10 */
    PeriPtr.ccu8_slice[1]->INS2 |= 0x00000400; /* simulate TRAP error by set CTRAP trigger level EV2LM to active low bit 10 */
    PeriPtr.ccu8_slice[2]->INS2 |= 0x00000400; /* simulate TRAP error by set CTRAP trigger level EV2LM to active low bit 10 */
    PeriPtr.posif->MCMC = (1UL << POSIF_MCMC_MNPC_Pos) |      /* Multi-Channel Pattern Update Enable Clear */
                          (1UL << POSIF_MCMC_MPC_Pos);        /* Multi-Channel Pattern Clear */

    ClearCommand();

    /* Point to null control mode code */
    control_loop_reset_ptr();

    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_ERROR;
}
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif

/*
 * State main function - ERROR state
 *   Change the state to STOP when all the errors are cleared
 */
void Motor0_BLDC_SCALAR_MSM_ERROR_Func(void)
{
    /* Change the state to STOP when all the errors are cleared */
    if (MotorVar.MaskedFault.Value == 0U)
    {
        Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_STOP;
    }
}

/*******************************************************************************
 * BLDC state machine: HALL_LEARNING state
 *
 ******************************************************************************/
/*
 * State entry function - HALL_LEARNING state
 *   Start CCU8 for PWM
 *   Stop POSIF if already running
 */
void Motor0_BLDC_SCALAR_MSM_HALL_LEARNING_Entry_Func(void)
{
    /* Initialize all run time variables */
    hall_learning_init();

    control_loop_reset_ptr();

    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_HALL_LEARNING;
}

/*
 * State entry function - HALL_LEARNING state
 *   Lock the rotor at the middle of the hall change @0, 60, 120 .... degrees
 *   Change to  NORMAL_OPERATION state if hall learning is successful
 */
void Motor0_BLDC_SCALAR_MSM_HALL_LEARNING_Func(void)
{
    Motor0_BLDC_SCALAR_Hall_Learning();

    /* Handle state transition */
    if (MotorVar.hall_learning_flag == 0)    /* hall learning is finished */
    {
        MotorVar.Command = MOTOR_STOP_CMD;
    }
    if (MotorVar.Command == MOTOR_STOP_CMD)
    {
        Motor0_BLDC_SCALAR_MSM_STOP_Entry_Func();
    }
}

/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This is the state machine function called from state machine event.
 * This handles the state transitions.
 */

 /**
 * The diag_supress pragma is applied to supress the specified diagnostic message
 * due to the calling of flash function from RAM function.
 * The diag_supress is documented on page 411
 * https://wwwfiles.iar.com/arm/webic/doc/ewarm_developmentguide.enu.pdf
*/
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif 
__RAM_FUNC void Motor0_BLDC_SCALAR_MSM(void)
{
    motor_common_protection();
    if ((Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_ERROR) && (MotorVar.MaskedFault.Value != 0U))
    {
        Motor0_BLDC_SCALAR_MSM_ERROR_Entry_Func();
    }

    switch (Motor0_BLDC_SCALAR.msm_state)
    {
    case BLDC_SCALAR_MSM_NORMAL_OPERATION:
        Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Func();
        break;

    case BLDC_SCALAR_MSM_IDLE:
        Motor0_BLDC_SCALAR_MSM_IDLE_Func();
        break;

    case BLDC_SCALAR_MSM_BOOTSTRAP:
        Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Func();
        break;

    case BLDC_SCALAR_MSM_ERROR:
        Motor0_BLDC_SCALAR_MSM_ERROR_Func();
        break;

    case BLDC_SCALAR_MSM_HALL_LEARNING:
        Motor0_BLDC_SCALAR_MSM_HALL_LEARNING_Func();
        break;

    case BLDC_SCALAR_MSM_STOP:
        Motor0_BLDC_SCALAR_MSM_STOP_Func();
        break;

    case BLDC_SCALAR_MSM_OFFSET_CALIBRATION:
        Motor0_BLDC_SCALAR_MSM_OFFSET_CALIBRATION_Func();
        break;

    default:
        break;
  }

#if (UCPROBE_ENABLE == 1)
    Motor0_BLDC_SCALAR_uCProbe_Scheduler();
#endif
}
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif
