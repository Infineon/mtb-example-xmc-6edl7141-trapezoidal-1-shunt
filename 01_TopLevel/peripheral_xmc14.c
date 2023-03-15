/******************************************************************************
* File Name: peripheral_xmc14.c
*
* Description: Three CCU8 slices are used to drive three motor phases (U, V and W) 
*              with PWM signals.
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
#include "peripheral_xmc14.h"
#include "04_Lib_BLDC/lib_bldc.h"

/** Structure initialization of CCU8 slices connected to phase U, V & W */
bldc_peripheral_ptr_t PeriPtr;

/** GPIO Init handle for Phase U High Pin */
const XMC_GPIO_CONFIG_t GPIO_PhU_High_Config =
{
    .mode = GPIO_PH_U_HS_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

/** GPIO Init handle for Phase U Low Pin */
const XMC_GPIO_CONFIG_t GPIO_PhU_Low_Config =
{
    .mode = GPIO_PH_U_LS_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

/** GPIO Init handle for Phase V High Pin */
const XMC_GPIO_CONFIG_t GPIO_PhV_High_Config =
{
    .mode = GPIO_PH_V_HS_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

/** GPIO Init handle for Phase V Low Pin */
const XMC_GPIO_CONFIG_t GPIO_PhV_Low_Config =
{
    .mode = GPIO_PH_V_LS_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

/** GPIO Init handle for Phase W High Pin */
const XMC_GPIO_CONFIG_t GPIO_PhW_High_Config =
{
    .mode = GPIO_PH_W_HS_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

/** GPIO Init handle for Phase W Low Pin */
const XMC_GPIO_CONFIG_t GPIO_PhW_Low_Config =
{
    .mode = GPIO_PH_W_LS_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

static void peri_ptr_init(void)
{
    /* Init CCU8 slice number and slice pointer for each motor phase */
    PeriPtr.ccu8 = (CCU8_GLOBAL_TypeDef*)(CCU80_BASE + 0x4000 * CCU8_MODULE_NUM);
    PeriPtr.ccu8_slice_num[0] = CCU8_PH_U_SLICE_NUM;
    PeriPtr.ccu8_slice_num[1] = CCU8_PH_V_SLICE_NUM;
    PeriPtr.ccu8_slice_num[2] = CCU8_PH_W_SLICE_NUM;
    uint32_t ccu8_cc80_base = (uint32_t)PeriPtr.ccu8 + 0x100;
    PeriPtr.ccu8_slice[0] = (CCU8_CC8_TypeDef*)(ccu8_cc80_base + 0x100 * CCU8_PH_U_SLICE_NUM);
    PeriPtr.ccu8_slice[1] = (CCU8_CC8_TypeDef*)(ccu8_cc80_base + 0x100 * CCU8_PH_V_SLICE_NUM);
    PeriPtr.ccu8_slice[2] = (CCU8_CC8_TypeDef*)(ccu8_cc80_base + 0x100 * CCU8_PH_W_SLICE_NUM);
    uint8_t u_slice_pos = CCU8_PH_U_SLICE_NUM * 4;
    uint8_t v_slice_pos = CCU8_PH_V_SLICE_NUM * 4;
    uint8_t w_slice_pos = CCU8_PH_W_SLICE_NUM * 4;
    PeriPtr.shadow_transfer = (1 << u_slice_pos) | (1 << v_slice_pos) | (1 << w_slice_pos);

    /* Init POSIF HAL */
    PeriPtr.posif = (POSIF_GLOBAL_TypeDef*)(POSIF0_BASE + 0x4000 * POSIF_MODULE_NUM);

    /* Init CCU4 HAL */
    PeriPtr.ccu4 = (CCU4_GLOBAL_TypeDef*)(CCU40_BASE + 0x4000 * CCU4_MODULE_NUM);
    PeriPtr.capture_slice_num = CCU4_CAPTURE_SLICE_NUM;
    PeriPtr.mcmsync_slice_num = CCU4_MCMSYNC_SLICE_NUM;
    uint32_t ccu4_cc40_base = (uint32_t)PeriPtr.ccu4 + 0x100;
    PeriPtr.capture_slice = (CCU4_CC4_TypeDef*)(ccu4_cc40_base + 0x100 * CCU4_CAPTURE_SLICE_NUM);
    PeriPtr.mcmsync_slice = (CCU4_CC4_TypeDef*)(ccu4_cc40_base + 0x100 * CCU4_MCMSYNC_SLICE_NUM);
}

/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Initialize CCU8 slice for all 3 phases PWM generation.
 */
static void ccu8_init(void)
{
    /* Enable CCU8 module */
    XMC_CCU8_EnableModule(PeriPtr.ccu8);
    PeriPtr.ccu8->GCTRL = (2UL << CCU8_GCTRL_SUSCFG_Pos); /* PWM set to passive level when suspended */
    /* Enable clock for slice, start the prescaler */
    PeriPtr.ccu8->GIDLC = 1 << PeriPtr.ccu8_slice_num[0] |
                          1 << PeriPtr.ccu8_slice_num[1] |
                          1 << PeriPtr.ccu8_slice_num[2] |
                          1 << CCU8_GIDLC_SPRB_Pos;

    /*Initialize CCU8 Phase-U, V and W Slice*/
    XMC_CCU8_SLICE_t *slice_u_ptr = PeriPtr.ccu8_slice[0];
    XMC_CCU8_SLICE_t *slice_v_ptr = PeriPtr.ccu8_slice[1];
    XMC_CCU8_SLICE_t *slice_w_ptr = PeriPtr.ccu8_slice[2];
    slice_u_ptr->TCCLR = CCU8_CC8_TCCLR_TRBC_Msk;   /* Stop slice timer */
    slice_v_ptr->TCCLR = CCU8_CC8_TCCLR_TRBC_Msk;   /* Stop slice timer */
    slice_w_ptr->TCCLR = CCU8_CC8_TCCLR_TRBC_Msk;   /* Stop slice timer */
    slice_u_ptr->TC = 1 << CCU8_CC8_TC_TCM_Pos |    /* Center Aligned */
                      2 << CCU8_CC8_TC_ENDM_Pos |   /* External Stop signal clears the timer and run bit */
                      1 << CCU8_CC8_TC_STRM_Pos |   /* External Start signal clears the timer and sets run bit */
                      0 << CCU8_CC8_TC_FPE_Pos |    /* Normal prescaler mode */
                      1 << CCU8_CC8_TC_TRAPE0_Pos | /* Enable TRAP for OUT0 */
                      1 << CCU8_CC8_TC_TRAPE1_Pos | /* Enable TRAP for OUT1 */
                      1 << CCU8_CC8_TC_MCME1_Pos |  /* Enable Multi-Channel mode for Channel 1 */
                      1 << CCU8_CC8_TC_STOS_Pos;    /* CC8yST2 forward to CCU8x.STy */
    slice_v_ptr->TC = 1 << CCU8_CC8_TC_TCM_Pos |    /* Center Aligned */
                      2 << CCU8_CC8_TC_ENDM_Pos |   /* External Stop signal clears the timer and run bit */
                      1 << CCU8_CC8_TC_STRM_Pos |   /* External Start signal clears the timer and sets run bit */
                      0 << CCU8_CC8_TC_FPE_Pos |    /* Normal prescaler mode */
                      1 << CCU8_CC8_TC_TRAPE0_Pos | /* Enable TRAP for OUT0 */
                      1 << CCU8_CC8_TC_TRAPE1_Pos | /* Enable TRAP for OUT1 */
                      1 << CCU8_CC8_TC_MCME1_Pos |  /* Enable Multi-Channel mode for Channel 1 */
                      0 << CCU8_CC8_TC_STOS_Pos;    /* CC8yST1 forward to CCU8x.STy */
    slice_w_ptr->TC = 1 << CCU8_CC8_TC_TCM_Pos |    /* Center Aligned */
                      2 << CCU8_CC8_TC_ENDM_Pos |   /* External Stop signal clears the timer and run bit */
                      1 << CCU8_CC8_TC_STRM_Pos |   /* External Start signal clears the timer and sets run bit */
                      0 << CCU8_CC8_TC_FPE_Pos |    /* Normal prescaler mode */
                      1 << CCU8_CC8_TC_TRAPE0_Pos | /* Enable TRAP for OUT0 */
                      1 << CCU8_CC8_TC_TRAPE1_Pos | /* Enable TRAP for OUT1 */
                      1 << CCU8_CC8_TC_MCME1_Pos |  /* Enable Multi-Channel mode for Channel 1 */
                      0 << CCU8_CC8_TC_STOS_Pos;    /* CC8yST1 forward to CCU8x.STy */
    slice_u_ptr->INS1 = CCU8_PH_U_EXTSTART_EVT0_IN << CCU8_CC8_INS1_EV0IS_Pos |
                        CCU8_PH_U_EXTSTART_EVT1_IN << CCU8_CC8_INS1_EV1IS_Pos |
                        CCU8_PH_U_CTRAP_EVT2_IN << CCU8_CC8_INS1_EV2IS_Pos;
    slice_v_ptr->INS1 = CCU8_PH_V_EXTSTART_EVT0_IN << CCU8_CC8_INS1_EV0IS_Pos |
                        CCU8_PH_V_EXTSTART_EVT1_IN << CCU8_CC8_INS1_EV1IS_Pos |
                        CCU8_PH_V_CTRAP_EVT2_IN << CCU8_CC8_INS1_EV2IS_Pos;
    slice_w_ptr->INS1 = CCU8_PH_W_EXTSTART_EVT0_IN << CCU8_CC8_INS1_EV0IS_Pos |
                        CCU8_PH_W_EXTSTART_EVT1_IN << CCU8_CC8_INS1_EV1IS_Pos |
                        CCU8_PH_W_CTRAP_EVT2_IN << CCU8_CC8_INS1_EV2IS_Pos;
    slice_u_ptr->INS2 = 1 << CCU8_CC8_INS2_EV0EM_Pos |                        /* Event 0 rising edge */
                        2 << CCU8_CC8_INS2_EV1EM_Pos |                        /* Event 1 falling edge */
                        0 << CCU8_CC8_INS2_EV2LM_Pos |                        /* Event 2 active high */
                        3 << CCU8_CC8_INS2_LPF2M_Pos;                         /* Event 2 low pass filter 7 clock cycles */
    slice_v_ptr->INS2 = 1 << CCU8_CC8_INS2_EV0EM_Pos |                        /* Event 0 rising edge */
                        2 << CCU8_CC8_INS2_EV1EM_Pos |                        /* Event 1 falling edge */
                        0 << CCU8_CC8_INS2_EV2LM_Pos |                        /* Event 2 active high */
                        3 << CCU8_CC8_INS2_LPF2M_Pos;                         /* Event 2 low pass filter 7 clock cycles */
    slice_w_ptr->INS2 = 1 << CCU8_CC8_INS2_EV0EM_Pos |                        /* Event 0 rising edge */
                        2 << CCU8_CC8_INS2_EV1EM_Pos |                        /* Event 1 falling edge */
                        0 << CCU8_CC8_INS2_EV2LM_Pos |                        /* Event 2 active high */
                        3 << CCU8_CC8_INS2_LPF2M_Pos;                         /* Event 2 low pass filter 7 clock cycles */
    slice_u_ptr->CMC = XMC_CCU8_SLICE_EVENT_0 << CCU8_CC8_CMC_STRTS_Pos |     /* External Start triggered by Event 0 */
                       XMC_CCU8_SLICE_EVENT_1 << CCU8_CC8_CMC_ENDS_Pos |      /* External Stop triggered by Event 1 */
                       ENABLE_CTRAP << CCU8_CC8_CMC_TS_Pos;                   /* Trap function selector */
    slice_v_ptr->CMC = XMC_CCU8_SLICE_EVENT_0 << CCU8_CC8_CMC_STRTS_Pos |     /* External Start triggered by Event 0 */
                       XMC_CCU8_SLICE_EVENT_1 << CCU8_CC8_CMC_ENDS_Pos |      /* External Stop triggered by Event 1 */
                       ENABLE_CTRAP << CCU8_CC8_CMC_TS_Pos;                   /* Trap function selector */
    slice_w_ptr->CMC = XMC_CCU8_SLICE_EVENT_0 << CCU8_CC8_CMC_STRTS_Pos |     /* External Start triggered by Event 0 */
                       XMC_CCU8_SLICE_EVENT_1 << CCU8_CC8_CMC_ENDS_Pos |      /* External Stop triggered by Event 1 */
                       ENABLE_CTRAP << CCU8_CC8_CMC_TS_Pos;                   /* Trap function selector */
    slice_u_ptr->PSL = PWM_ACTIVE_HIGH << CCU8_CC8_PSL_PSL11_Pos |            /* Output passive level for OUT0 */
                       PWM_ACTIVE_HIGH << CCU8_CC8_PSL_PSL12_Pos;             /* Output passive level for OUT1 */
    slice_v_ptr->PSL = PWM_ACTIVE_HIGH << CCU8_CC8_PSL_PSL11_Pos |            /* Output passive level for OUT0 */
                       PWM_ACTIVE_HIGH << CCU8_CC8_PSL_PSL12_Pos;             /* Output passive level for OUT1 */
    slice_w_ptr->PSL = PWM_ACTIVE_HIGH << CCU8_CC8_PSL_PSL11_Pos |            /* Output passive level for OUT0 */
                       PWM_ACTIVE_HIGH << CCU8_CC8_PSL_PSL12_Pos;             /* Output passive level for OUT1 */
    slice_u_ptr->CHC = 0 << CCU8_CC8_CHC_ASE_Pos |                            /* Asymmetric PWM is disabled */
                       XMC_CCU8_SOURCE_OUT0_ST1 << CCU8_CC8_CHC_OCS1_Pos |    /* OUT0 use ST1 signal, PWMxH */
                       XMC_CCU8_SOURCE_OUT1_INV_ST1 << CCU8_CC8_CHC_OCS2_Pos; /* OUT1 use inverted ST1 signal, PWMxL */
    slice_v_ptr->CHC = 0 << CCU8_CC8_CHC_ASE_Pos |                            /* Asymmetric PWM is disabled */
                       XMC_CCU8_SOURCE_OUT0_ST1 << CCU8_CC8_CHC_OCS1_Pos |    /* OUT0 use ST1 signal, PWMxH */
                       XMC_CCU8_SOURCE_OUT1_INV_ST1 << CCU8_CC8_CHC_OCS2_Pos; /* OUT1 use inverted ST1 signal, PWMxL */
    slice_w_ptr->CHC = 0 << CCU8_CC8_CHC_ASE_Pos |                            /* Asymmetric PWM is disabled */
                       XMC_CCU8_SOURCE_OUT0_ST1 << CCU8_CC8_CHC_OCS1_Pos |    /* OUT0 use ST1 signal, PWMxH */
                       XMC_CCU8_SOURCE_OUT1_INV_ST1 << CCU8_CC8_CHC_OCS2_Pos; /* OUT1 use inverted ST1 signal, PWMxL */
    slice_u_ptr->PRS = MotorVar.PWM_TIMER_RELOAD;                             /* Update CCU8 slice period registers */
    slice_v_ptr->PRS = MotorVar.PWM_TIMER_RELOAD;                             /* Update CCU8 slice period registers */
    slice_w_ptr->PRS = MotorVar.PWM_TIMER_RELOAD;                             /* Update CCU8 slice period registers */

    /* Use U phase one match event as interrupt SR for control loop execution */
    slice_u_ptr->SRS = (slice_u_ptr->SRS & ~(3 << CCU8_CC8_SRS_POSR_Pos)) |
                       XMC_CCU8_SLICE_SR_ID_0 << CCU8_CC8_SRS_POSR_Pos;
    XMC_CCU8_SLICE_EnableEvent(slice_u_ptr, XMC_CCU8_SLICE_IRQ_ID_ONE_MATCH);

    /* Use V phase compare match 2 down event as SR for ADC trigger */
    slice_v_ptr->SRS = (slice_v_ptr->SRS & ~(3 << CCU8_CC8_SRS_CM2SR_Pos)) |
                       XMC_CCU8_SLICE_SR_ID_2 << CCU8_CC8_SRS_CM2SR_Pos;
    XMC_CCU8_SLICE_EnableEvent(slice_v_ptr, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_DOWN_CH_2);
    slice_v_ptr->CR2S = MotorVar.PWM_TIMER_RELOAD - MotorParam.CURRENT_TRIGGER; /* Set trigger point */

    /*Enable CCU8 shadow transfer*/
    PeriPtr.ccu8->GCSS = PeriPtr.shadow_transfer;
}

/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Initializes capture CCU4 slice for motor speed calculations from HALL events.
 */
static void CCU4_SpeedCapture_Init(void)
{
    /*capture slice initiation*/
    PeriPtr.capture_slice->TC = 1UL << CCU4_CC4_TC_CAPC_Pos |   /* Clear on capture control, 1: Clear on capture into capture register 2 and 3 */
                                1UL << CCU4_CC4_TC_FPE_Pos;     /* Enable floating prescaler */
    PeriPtr.capture_slice->CMC = 0UL;
    PeriPtr.capture_slice->PSC = 0UL << CCU4_CC4_PSC_PSIV_Pos;            /* Prescaler initial value */
    PeriPtr.capture_slice->PSL = 0UL << CCU4_CC4_PSL_PSL_Pos;             /* Program timer output passive level */
    PeriPtr.capture_slice->FPCS = 0UL << CCU4_CC4_FPCS_PCMP_Pos;          /* Program floating prescaler compare value */
    PeriPtr.capture_slice->FPC = CCU4_PRESCALER << CCU4_CC4_FPC_PVAL_Pos; /*Set prescaler value*/

    /* In any case, update the initial value of the divider which is to be loaded once the prescaler increments to the compare value */
    PeriPtr.capture_slice->PSC = CCU4_PRESCALER << CCU4_CC4_PSC_PSIV_Pos;

    /* configure Event 0 signal as capture event*/
    PeriPtr.capture_slice->INS1 = CCU41_IN2_POSIF1_OUT0 << CCU4_CC4_INS1_EV0IS_Pos;                                 /* CCU4x_CC42 slice input 28(CCU4x.IN2BC) is from POSIFx.OUT0 */
    PeriPtr.capture_slice->INS2 = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE << CCU4_CC4_INS2_EV0EM_Pos |    /* Configure the edge sensitivity */
                                  XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH << CCU4_CC4_INS2_EV0LM_Pos |   /* Configure Level */
                                  XMC_CCU4_SLICE_EVENT_FILTER_DISABLED << CCU4_CC4_INS2_LPF0M_Pos;                  /* Configure debounce filter */

    /* configure channel for speed capture*/
    PeriPtr.capture_slice->CMC |= XMC_CCU4_SLICE_EVENT_0 << CCU4_CC4_CMC_CAP1S_Pos; /* External capture 1 triggered by Event 0 */

    /* Clear IDLE mode.*/
    PeriPtr.ccu4->GIDLC = 1UL << PeriPtr.capture_slice_num;
}

/* Multi-channel update delay slice initialization */
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * After a hall event is detected, apply a delay time before trigger MCM update.
 * Runs in single shut mode, timer starts by POSIF1.OUT0. PS1 output is connected to POSIF to request
 * multi-channel pattern update.
 */
static void CCU4_MCMSync_Init(void)
{
    /*Timer initiation*/
    PeriPtr.mcmsync_slice->TC = 0 << CCU4_CC4_TC_TCM_Pos |                /* 0: Edge aligned mode */
                                1 << CCU4_CC4_TC_TSSM_Pos |               /* 1: Enable single shot mode */
                                0 << CCU4_CC4_TC_CLST_Pos |               /* 0: Disable shadow transfer on clear */
                                1 << CCU4_CC4_TC_STRM_Pos |               /* 1: Clear the timer and set run bit (flush/start) */
                                0 << CCU4_CC4_TC_FPE_Pos |                /* 0: Disable floating prescaler */
                                0 << CCU4_CC4_TC_MCME_Pos;                /* 0: Disable Multi-channel mode */
    PeriPtr.mcmsync_slice->PSC = CCU4_PRESCALER << CCU4_CC4_PSC_PSIV_Pos; /* Prescaler initial value */
    PeriPtr.mcmsync_slice->PSL = 0 << CCU4_CC4_PSL_PSL_Pos;               /* Output pin passive level is low */
    PeriPtr.mcmsync_slice->FPCS = 0 << CCU4_CC4_FPCS_PCMP_Pos;            /* Program floating prescaler compare value */

    PeriPtr.mcmsync_slice->SRS = 0 << CCU4_CC4_SRS_POSR_Pos;  /* Period/One match forward to SR0 */
    PeriPtr.mcmsync_slice->INTE = 1 << CCU4_CC4_INTE_PME_Pos; /* Enable interrupt: Period match */

    PeriPtr.ccu4->GIDLC = 1 << PeriPtr.mcmsync_slice_num; /* Enable clock - Idle mode clear */
}

/*
 * Initialize POSIF and CCU4 peripherals
 * Hall sensor input pins are initialized by BSP
 */
static void ccu4_posif_init()
{
    /************************************************************************************
     * Configure CCU4
     ***********************************************************************************/
    /* Enable CCU4 module */
    XMC_CCU4_Init(PeriPtr.ccu4, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    /* Speed capture slice initialization */
    CCU4_SpeedCapture_Init();
    /* Fast sync slice initialization */
    CCU4_MCMSync_Init();

    /************************************************************************************
     * Enable the POSIF module
     ***********************************************************************************/
    XMC_POSIF_Enable(PeriPtr.posif);
    /* Stop POSIF */
    PeriPtr.posif->PRUNC = POSIF_PRUNC_CRB_Msk | POSIF_PRUNC_CSM_Msk;
    /* POSIF configured with hall sensor mode and input multiplexer configurations */
    PeriPtr.posif->PCONF = XMC_POSIF_MODE_HALL_SENSOR << POSIF_PCONF_FSEL_Pos |     /* POSIF Operational mode */
                           POSIF_HALL_0_INSEL << POSIF_PCONF_INSEL0_Pos |           /* Choice of input for Input-1 */
                           POSIF_HALL_1_INSEL << POSIF_PCONF_INSEL1_Pos |           /* Choice of input for Input-2 */
                           POSIF_HALL_2_INSEL << POSIF_PCONF_INSEL2_Pos |           /* Choice of input for Input-3 */
                           XMC_POSIF_FILTER_16_CLOCK_CYCLE << POSIF_PCONF_LPC_Pos | /* Input filter configuration */
                                                                                    /* POSIF multi-channel configurations */
                           1U << POSIF_PCONF_MCUE_Pos |                             /* Multi-Channel pattern is updated via 0: HW, 1: SW */
                           XMC_POSIF_INPUT_PORT_A << POSIF_PCONF_MSYNS_Pos |        /* PWM synchronization signal selector, POSIFx.MSYNCA = CCU8x.PS1, POSIFx.MSYNCC = CCU4x.PS1 TYS: IF fast SYN enable use Port_C */
                                                                                    /* Hall sensor mode configurations */
                           1U << POSIF_PCONF_HIDG_Pos |                             /* Disable the IDLE signal */
                           1U << POSIF_PCONF_DSEL_Pos |                             /* Delay pin selector, 0: HSDA(CCU4x.ST0), 1: HSDB (POUT0) */
                           0U << POSIF_PCONF_SPES_Pos;                              /* Edge selector for sampling trigger, 0: rising, 1: falling */
    /* Bind interrupt node */
    PeriPtr.posif->PFLGE = 1 << POSIF_PFLGE_EHIE_Pos |  /* Enable hall input interrupt */
                           0 << POSIF_PFLGE_HIESEL_Pos; /* Hall input interrupt forward to 0: SR0; 1: SR1 */

    /************************************************************************************
     * Start POSIF and Hall sensor related CCU4 timer
     ***********************************************************************************/
    PeriPtr.posif->PRUNS = POSIF_PRUNS_SRB_Msk;
    PeriPtr.capture_slice->TCCLR = CCU4_CC4_TCCLR_TCC_Msk;  /* Clear timer */
    PeriPtr.capture_slice->TCSET = CCU4_CC4_TCSET_TRBS_Msk; /* Start timer */
}

static void vadc_init(void)
{
    /***************************************************
     * Enable the VADC module
     **************************************************/
    XMC_VADC_GLOBAL_EnableModule();
    VADC->CLC = 0 << VADC_CLC_DISR_Pos;             /* 0: Enable the module clock */
    VADC->GLOBCFG = 0 << VADC_GLOBCFG_DIVA_Pos |    /* 0: F_adci = F_adc/2 */
                    0 << VADC_GLOBCFG_DIVD_Pos |    /* 0: F_adcd = F_adc */
                    1 << VADC_GLOBCFG_DIVWC_Pos;

    /***************************************************
     * VADC group 0
     **************************************************/
    VADC_G0->ARBCFG = 3 << VADC_G_ARBCFG_ANONC_Pos |    /* 3: Analog part always active */
                      1 << VADC_G_ARBCFG_ARBM_Pos;      /* 1: Arbiter only runs if at least one conversion request is pending */
    /* Program the input classes for Group[1] */
    VADC_G0->ICLASS[0] = 1 << VADC_G_ICLASS_STCS_Pos;   /* 1 additional clock cycle for standard conversion */
    /* Initialize the Queue1 */
    VADC_G0->ARBPR = XMC_VADC_GROUP_RS_PRIORITY_1 << VADC_G_ARBPR_PRIO0_Pos |   /* Request Source priority */
                     XMC_VADC_STARTMODE_WFS << VADC_G_ARBPR_CSM0_Pos |          /* Conversion start mode = wait-for-start mode */
                     1 << VADC_G_ARBPR_ASEN0_Pos;                               /* Enable arbitration slot 0 */
    VADC_G0->QMR0 = XMC_VADC_GATEMODE_IGNORE << VADC_G_QMR0_ENGT_Pos |          /* Conversion requests are issued if a valid conversion request is pending in the queue 0 register or in the backup register */
                    1 << VADC_G_QMR0_ENTR_Pos |                                 /* Enable external trigger */
                    1 << VADC_G_QMR0_FLUSH_Pos;                                 /* Clear all queue entries */
    VADC_G0->QCTRL0 = MOTOR0_BLDC_SCALAR_VADC_QUEUE_0_TRIGGER_SIGNAL << VADC_G_QCTRL0_XTSEL_Pos |
                      XMC_VADC_TRIGGER_EDGE_RISING << VADC_G_QCTRL0_XTMODE_Pos |
                      1 << VADC_G_QCTRL0_XTWC_Pos;

    /***********************************************
     * ADC channel for kit select
     * Don't put kit select input to queue, will do it in read_kit_id_xmc14()
     **********************************************/

    /***************************************************
     * VADC group 1
     **************************************************/
    VADC_G1->ARBCFG = 3 << VADC_G_ARBCFG_ANONC_Pos |    /* 3: Analog part always active */
                      1 << VADC_G_ARBCFG_ARBM_Pos;      /* 1: Arbiter only runs if at least one conversion request is pending */
    /* Program the input classes for Group[1] */
    VADC_G1->ICLASS[0] = 1 << VADC_G_ICLASS_STCS_Pos;   /* 1 additional clock cycle for standard conversion */
    /* Initialize the Queue1 */
    VADC_G1->ARBPR = XMC_VADC_GROUP_RS_PRIORITY_1 << VADC_G_ARBPR_PRIO0_Pos |   /* Request Source priority */
                     XMC_VADC_STARTMODE_WFS << VADC_G_ARBPR_CSM0_Pos |          /* Conversion start mode = wait-for-start mode */
                     1 << VADC_G_ARBPR_ASEN0_Pos;                               /* Enable arbitration slot 0 */
    VADC_G1->QMR0 = XMC_VADC_GATEMODE_IGNORE << VADC_G_QMR0_ENGT_Pos |          /* Conversion requests are issued if a valid conversion request is pending in the queue 0 register or in the backup register */
                    1 << VADC_G_QMR0_ENTR_Pos |                                 /* Enable external trigger */
                    1 << VADC_G_QMR0_FLUSH_Pos;                                 /* Clear all queue entries */
    VADC_G1->QCTRL0 = MOTOR0_BLDC_SCALAR_VADC_QUEUE_1_TRIGGER_SIGNAL << VADC_G_QCTRL0_XTSEL_Pos |
                      XMC_VADC_TRIGGER_EDGE_RISING << VADC_G_QCTRL0_XTMODE_Pos |
                      1 << VADC_G_QCTRL0_XTWC_Pos;

    /***********************************************
     * ADC channel for DC link current sensing
     **********************************************/
    VADC_G1->CHCTR[IDC_LINK_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                      XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                      XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1 << VADC_G_CHCTR_BNDSELU_Pos |
                                      IDC_LINK_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                      XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = IDC_LINK_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     1 << VADC_G_QINR0_EXTR_Pos;
    SHS0->GNCTR10 = MotorParam.InternalGain << (IDC_LINK_CH_NUM << 2);

    /***********************************************
     * ADC channel for DC link voltage sensing
     **********************************************/
    VADC_G1->CHCTR[VDC_LINK_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                      XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                      XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELU_Pos |
                                      VDC_LINK_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                      XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = VDC_LINK_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     0 << VADC_G_QINR0_EXTR_Pos;

    /***********************************************
     * ADC channel for pot input
     **********************************************/
    VADC_G1->CHCTR[POT_INPUT_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELU_Pos |
                                       POT_INPUT_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                       XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = POT_INPUT_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     0 << VADC_G_QINR0_EXTR_Pos;

    /***********************************************
     * ADC channel for MCP9700 temperature sensor
     **********************************************/
    VADC_G1->CHCTR[TEMP_SENSE_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                        XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                        XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELU_Pos |
                                        TEMP_SENSE_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                        XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = TEMP_SENSE_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     0 << VADC_G_QINR0_EXTR_Pos;

    /***********************************************
     * ADC channel for User input 1
     **********************************************/
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF1_MEASUREMENT == 1U)
    VADC_G1->CHCTR[USER_DEF1_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELU_Pos |
                                       USER_DEF1_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                       XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = USER_DEF1_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     0 << VADC_G_QINR0_EXTR_Pos;
#endif

    /***********************************************
     * ADC channel for User input 2
     **********************************************/
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF2_MEASUREMENT == 1U)
    VADC_G1->CHCTR[USER_DEF2_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELU_Pos |
                                       USER_DEF2_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                       XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = USER_DEF2_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     0 << VADC_G_QINR0_EXTR_Pos;
#endif

    /***********************************************
     * ADC channel for User input 3
     **********************************************/
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF3_MEASUREMENT == 1U)
    VADC_G1->CHCTR[USER_DEF3_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                       XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELU_Pos |
                                       USER_DEF3_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                       XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    VADC_G1->QINR0 = USER_DEF3_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                     1 << VADC_G_QINR0_RF_Pos |
                     0 << VADC_G_QINR0_EXTR_Pos;
#endif

    /***************************************************
     * Initiate ADC calibration
     **************************************************/
    for (uint32_t i=0; i<10; i++) {};   /* Need a short delay */
    VADC->GLOBCFG |= (1U << VADC_GLOBCFG_SUCAL_Pos); /* Initiate the start-up calibration phase */
    while (VADC_G0->ARBCFG & VADC_G_ARBCFG_CAL_Msk); /* Wait for calibration to complete */
    while (VADC_G1->ARBCFG & VADC_G_ARBCFG_CAL_Msk); /* Wait for calibration to complete */
}

/* Initialize the required peripherals, modules and interrupts for BLDC control */
void peripheral_init_xmc14(void)
{
    /***********************************************************
     * Initialize peripheral pointer
     **********************************************************/
    peri_ptr_init();

    /***********************************************************
     * Configure GPIO
     * 6EDL7141 related IOs are configured by 6EDL gateway init
     **********************************************************/
    /* Motor PWM output */
    XMC_GPIO_Init(CYBSP_PWMUH_PORT, CYBSP_PWMUH_PIN, &GPIO_PhU_High_Config);
    XMC_GPIO_Init(CYBSP_PWMUL_PORT, CYBSP_PWMUL_PIN, &GPIO_PhU_Low_Config);
    XMC_GPIO_Init(CYBSP_PWMVH_PORT, CYBSP_PWMVH_PIN, &GPIO_PhV_High_Config);
    XMC_GPIO_Init(CYBSP_PWMVL_PORT, CYBSP_PWMVL_PIN, &GPIO_PhV_Low_Config);
    XMC_GPIO_Init(CYBSP_PWMWH_PORT, CYBSP_PWMWH_PIN, &GPIO_PhW_High_Config);
    XMC_GPIO_Init(CYBSP_PWMWL_PORT, CYBSP_PWMWL_PIN, &GPIO_PhW_Low_Config);

    /***********************************************************
     * Configure ERU
     **********************************************************/
    /* Initialize ERU1.ch2 to P3.4 for nFault detection
     * Initialize ERU1.OGU0 for POUT and IOUT
     * to interrupt ERU1 node for interrupt execution.
     **********************************************************/
    XMC_ERU_Enable(XMC_ERU1);
    uint8_t channel = 2;
    XMC_ERU1->EXISEL = (XMC_ERU1->EXISEL & ~(0xF << (channel * 4))) |
                       ERU1_ETL2_INPUTA_P3_4 << (channel * 4);
    XMC_ERU1->EXICON[channel] = true << ERU_EXICON_PE_Pos |
                                XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL << ERU_EXICON_LD_Pos |
                                XMC_ERU_ETL_EDGE_DETECTION_FALLING << ERU_EXICON_RE_Pos |
                                XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0 << ERU_EXICON_OCS_Pos |
                                XMC_ERU_ETL_SOURCE_A << ERU_EXICON_SS_Pos;
    XMC_ERU1->EXOCON[0] = 0 << ERU_EXOCON_ISS_Pos |                                        /* peripheral_trigger */
                          XMC_ERU_OGU_PATTERN_DETECTION_ENABLED << ERU_EXOCON_GEEN_Pos |   /* enable_pattern_detection */
                          XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER_AND_PATTERN_MATCH << ERU_EXOCON_GP_Pos |  /* service_request */
                          XMC_ERU_OGU_PATTERN_DETECTION_INPUT2 << ERU_EXOCON_IPEN0_Pos;    /* pattern_detection_input */

    /***********************************************************
     * Configure CCU8
     **********************************************************/
    ccu8_init();

    /***********************************************************
     * Configure CCU4 & POSIF
     **********************************************************/
    ccu4_posif_init();

    /***********************************************************
     * Configure VADC
     **********************************************************/
    vadc_init();

    /***********************************************************
     * Configure Interrupt
     **********************************************************/
    SCU_GENERAL->INTCR1 = (SCU_GENERAL->INTCR1 & 0xFF33F3FF) + 0x00440400; /* Interrupt node = 21, source = 1 (CCU41_SR0)
                                                                              Interrupt node = 25, source = 1 (CCU81_SR0)
                                                                              Interrupt node = 27, source = 1 (POSIF1_SR0) */
    NVIC_SetPriority(IRQ21_IRQn, 0);        /* Commutation ISR needs higher priority than Hall Event ISR */
    NVIC_SetPriority(IRQ25_IRQn, 2);
    NVIC_SetPriority(IRQ27_IRQn, 1);
    NVIC->ISER[0] = 1 << 25;                /* Enable CCU8 fast control loop interrupt */
    // NVIC->ISER[0] = 1 << 27;                /* Enable POSIF hall event interrupt */

    /* Start CCU8 PWM timers, set CCUCON trigger signal to high to start all slices synchronously */
    SCU_GENERAL->CCUCON |= 1 << (8 + CCU8_MODULE_NUM);
}

uint16_t read_kit_id_xmc14(void)
{
    /***************************************************
     * Enable the VADC module
     **************************************************/
    XMC_VADC_GLOBAL_EnableModule();
    VADC->CLC = 0 << VADC_CLC_DISR_Pos;             /* 0: Enable the module clock */
    VADC->GLOBCFG = 0 << VADC_GLOBCFG_DIVA_Pos |    /* 0: F_adci = F_adc/2 */
                    0 << VADC_GLOBCFG_DIVD_Pos |    /* 0: F_adcd = F_adc */
                    1 << VADC_GLOBCFG_DIVWC_Pos;

    /***************************************************
     * VADC KIT_SELECT group
     **************************************************/
    KIT_SELECT_GRP->ARBCFG = 3 << VADC_G_ARBCFG_ANONC_Pos | /* 3: Analog part always active */
                             1 << VADC_G_ARBCFG_ARBM_Pos;   /* 1: Arbiter only runs if at least one conversion request is pending */
    /* Program the input classes for Group[1] */
    KIT_SELECT_GRP->ICLASS[0] = 1 << VADC_G_ICLASS_STCS_Pos;   /* 1 additional clock cycle for standard conversion */
    /* Initialize the Queue1 */
    KIT_SELECT_GRP->ARBPR = XMC_VADC_GROUP_RS_PRIORITY_1 << VADC_G_ARBPR_PRIO0_Pos |   /* Request Source priority */
                            XMC_VADC_STARTMODE_WFS << VADC_G_ARBPR_CSM0_Pos |          /* Conversion start mode = wait-for-start mode */
                            1 << VADC_G_ARBPR_ASEN0_Pos;                               /* Enable arbitration slot 0 */
    KIT_SELECT_GRP->QMR0 = XMC_VADC_GATEMODE_IGNORE << VADC_G_QMR0_ENGT_Pos |   /* Conversion requests are issued if a valid conversion request is pending in the queue 0 register or in the backup register */
                           1 << VADC_G_QMR0_ENTR_Pos |                          /* Enable external trigger */
                           1 << VADC_G_QMR0_FLUSH_Pos;                          /* Clear all queue entries */

    /***************************************************
   * Initiate ADC calibration
   **************************************************/
    for (uint32_t i=0; i<10; i++) {};   /* Need a short delay */
    VADC->GLOBCFG |= (1U << VADC_GLOBCFG_SUCAL_Pos); /* Initiate the start-up calibration phase */
    while (KIT_SELECT_GRP->ARBCFG & VADC_G_ARBCFG_CAL_Msk); /* Wait for calibration to complete */
    while (VADC_G1->ARBCFG & VADC_G_ARBCFG_CAL_Msk); /* Wait for calibration to complete */
    for (uint32_t i=0; i<200; i++) {};   /* Need a short delay after ADC calibration */

    /* Add kit select channel into queue, no refill and trigger only once */
    KIT_SELECT_GRP->CHCTR[KIT_SELECT_CH_NUM] = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0 << VADC_G_CHCTR_ICLSEL_Pos |
                                               XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0 << VADC_G_CHCTR_BNDSELL_Pos |
                                               XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1 << VADC_G_CHCTR_BNDSELU_Pos |
                                               KIT_SELECT_CH_NUM << VADC_G_CHCTR_RESREG_Pos |
                                               XMC_VADC_RESULT_ALIGN_RIGHT << VADC_G_CHCTR_RESPOS_Pos;
    KIT_SELECT_GRP->QINR0 = KIT_SELECT_CH_NUM << VADC_G_QINR0_REQCHNR_Pos |
                            0 << VADC_G_QINR0_RF_Pos | /* No refill */
                            0 << VADC_G_QINR0_EXTR_Pos;
    KIT_SELECT_GRP->QMR0 |= 1 << VADC_G_QMR0_TREV_Pos; /* Trigger once */
    /* Keep reading result register until valid flag is set */
    uint32_t kit_select_result;
    while (((kit_select_result = KIT_SELECT_GRP->RES[KIT_SELECT_CH_NUM]) & VADC_G_RES_VF_Msk) == 0);
    /* Convert to kit id 0-31. 0~63->0; 64~191->1;... 3968~4095->31 */
    kit_select_result = ((uint16_t)kit_select_result + 0x40) >> 7;
    return (uint16_t)kit_select_result;
}

void idc_trigger_conversion_xmc14(void)
{
    IDC_LINK_GRP->QMR0 |= (uint32_t)((uint32_t)1 << VADC_G_QMR0_TREV_Pos);
}

uint32_t check_chip_id_xmc14(void)
{
    uint32_t chip_id;
    chip_id = SCU_GENERAL->IDCHIP;
    /* Read chip ID, XMC1400: 0x00014xxx, IMD700A: 0x270x100A */
    if (((chip_id >> 12) != 0x14) && ((chip_id & 0xFFF0FFFF) != 0x2700100A))
    {
        /* chip_id is wrong, set chip_id to 0 */
        chip_id = 0;
    }
    return chip_id;
}
