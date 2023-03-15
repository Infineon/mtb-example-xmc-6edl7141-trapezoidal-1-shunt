/******************************************************************************
* File Name: peripheral_xmc14.h
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
#pragma once

#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_vadc.h>
#include <xmc_gpio.h>
#include <xmc_posif.h>
#include <xmc_scu.h>
#include <xmc_eru.h>
#include <cycfg.h>

/** CPU clock frequency in MHz*/
#define CPU_FREQ_MHZ                        (48.0F)
/** CCU module frequency in MHz*/
#define PWM_TIMER_FREQ_MHZ                  (96.0F)

#define CCU4_PRESCALER                      (4U)
#define CCU4_FREQUENCY_MHZ                  (PWM_TIMER_FREQ_MHZ / (1 << CCU4_PRESCALER))

/* Ctrap protection */
#define ENABLE_CTRAP                        (1U)       /*!< Enable (1)/disable (0) ctrap protection */

#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF1_MEASUREMENT           (1U)  /*!< Enable (1)/disable (0)user defined measurement
                                                                              Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF2_MEASUREMENT           (1U)  /*!< Enable (1)/disable (0)user defined measurement
                                                                              Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF3_MEASUREMENT           (1U)  /*!< Enable (1)/disable (0)user defined measurement
                                                                              Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/
//#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF4_MEASUREMENT           (1U)  /*!< Enable (1)/disable (0)user defined measurement
//                                                                              Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/

/***********************************************************************************************************************
 * PWM Generation
 **********************************************************************************************************************/
/**
 * PWM output pins (#define in cycfg_pins.h)
 */

/** ALT mode is configured as CCU8 controlled output */
/* Refer register Px_IOCR in the reference manual */
#define GPIO_PH_U_HS_MODE    (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8)
#define GPIO_PH_U_LS_MODE    (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8)
#define GPIO_PH_V_HS_MODE    (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8)
#define GPIO_PH_V_LS_MODE    (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8)
#define GPIO_PH_W_HS_MODE    (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8)
#define GPIO_PH_W_LS_MODE    (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8)

/** Three slices of the CCU8 which are connected to Phase U, V and W */
#define CCU8_MODULE_NUM      (1U)   /* CCU81 */
#define CCU8_PH_U_SLICE_NUM  (1U)   /* CCU81_CC81 */
#define CCU8_PH_V_SLICE_NUM  (2U)   /* CCU81_CC82 */
#define CCU8_PH_W_SLICE_NUM  (0U)   /* CCU81_CC80 */

/** Maximum number of phase count */
#define CCU8_MAXPHASE_COUNT                     (3U)

/***********************************************************************************************************************
 * Multi-channel pattern based on CCU8 slices
 **********************************************************************************************************************/
/** Both high and low side switches are OFF */
#define   MCM_OFF								(0U)	/* 000 */
/** High side is ON */
#define   MCM_HS_ON								(1U)	/* 001 */
/** Low side is ON */
#define   MCM_LS_ON								(2U)	/* 010 */
/** High side is switching */
#define   MCM_HS_SW								(5U)	/* 101 */
/** Low side is switching */
#define   MCM_LS_SW								(6U)	/* 110 */
/** Complementary switching */
#define   MCM_HS_COMP							(3U)	/* 011 */
/** Complementary switching */
#define   MCM_LS_COMP							(7U)	/* 111 */

/***********************************************************************************************************************
 * CTRAP - CCU8 event 2
 **********************************************************************************************************************/
/** Trap pin (P3.4) <-> ERU1.2A3->ERU1.PDOUT0 <-> CCU8 input event 2 multiplexer configuration */

/* Refer CCU8 INS.EV2IS in reference manual */
#define CCU8_PH_U_CTRAP_EVT2_IN      (CCU81_IN1_ERU1_PDOUT0)
#define CCU8_PH_V_CTRAP_EVT2_IN      (CCU81_IN2_ERU1_PDOUT0)
#define CCU8_PH_W_CTRAP_EVT2_IN      (CCU81_IN0_ERU1_PDOUT0)

/***********************************************************************************************************************
 * Synchronous start - CCU8 event 0
 **********************************************************************************************************************/
/**
 * Three CCU8 slices are started synchronously with CCUCON bit set
 * Refer CCU8 INS.EV0IS in reference manual
 */
#define CCU8_PH_U_EXTSTART_EVT0_IN   (CCU81_IN1_SCU_GSC80)
#define CCU8_PH_V_EXTSTART_EVT0_IN   (CCU81_IN2_SCU_GSC80)
#define CCU8_PH_W_EXTSTART_EVT0_IN   (CCU81_IN0_SCU_GSC80)

/***********************************************************************************************************************
 * Synchronous stop - CCU8 event 1
 **********************************************************************************************************************/
/**
 * Three CCU8 slices are stopped synchronously with CCUCON bit reset
 * Refer INS.EV1IS in reference manual
 */
#define CCU8_PH_U_EXTSTART_EVT1_IN   (CCU81_IN1_SCU_GSC80)
#define CCU8_PH_V_EXTSTART_EVT1_IN   (CCU81_IN2_SCU_GSC80)
#define CCU8_PH_W_EXTSTART_EVT1_IN   (CCU81_IN0_SCU_GSC80)

/***********************************************************************************************************************
 * HALL sensor interface
 **********************************************************************************************************************/
/** POSIF module number */
#define POSIF_MODULE_NUM         (1)        /* POSIF1 */
// /**
//  * Hall sensor input pins selection
//  * For different pins than default configuration,
//  * Change the POSIF input multiplexer configuration accordingly
//  */
// #define GPIO_HALL_1              P4_1
// #define GPIO_HALL_2              P4_2
// #define GPIO_HALL_3              P4_3

/** Input selector multiplexer */
/** Refer POSIF PCONF.INSEL0 in reference manual*/
#define POSIF_HALL_0_INSEL      (XMC_POSIF_INPUT_PORT_B)
/** Refer POSIF PCONF.INSEL1 in reference manual*/
#define POSIF_HALL_1_INSEL      (XMC_POSIF_INPUT_PORT_B)
/** Refer POSIF PCONF.INSEL2 in reference manual*/
#define POSIF_HALL_2_INSEL      (XMC_POSIF_INPUT_PORT_B)

/***********************************************************************************************************************
 * CCU4 configuration for blanking time and speed capture
 **********************************************************************************************************************/
/**
 * CCU4 module and slice numbers selection
 * For different hall pins than default configuration, Change the POSIF and CCU4 interconnects accordingly
 */
#define CCU4_MODULE_NUM        (1)    /* CCU41 */
/** Slice used for speed capture */
#define CCU4_CAPTURE_SLICE_NUM (2)    /* CCU41_CC42 */
/*!< Slice used for multi-channel pattern synchronization */
#define CCU4_MCMSYNC_SLICE_NUM (1)    /* CCU41_CC41 */

/***********************************************************************************************************************
 * Interrupt Configurations
 **********************************************************************************************************************/
/* Interrupt handlers */
#define Motor0_BLDC_SCALAR_ControlLoop_ISR           (IRQ25_Handler)  // CCU81.SR0 IRQHandler
#define Motor0_BLDC_SCALAR_HallEvent_ISR             (IRQ27_Handler)  // POSIF1.SR0 IRQHandler
#define Commutation_ISR								 (IRQ21_Handler)  // CCU41.SR0 IRQHandler

/***********************************************************************************************************************
 * VADC configurations
 * +---------------+-------+
 * |  Pin  |  G0   |  G1   |
 * +-------+-------+-------+
 * | P2.0  | G0CH5 |       |
 * | P2.1  | G0CH6 |       |
 * | P2.2  | G0CH7 |       |
 * | P2.3  |       | G1CH5 |
 * | P2.4  |       | G1CH6 |
 * | P2.5  |       | G1CH7 |
 * | P2.6  | G0CH0 |       |
 * | P2.7  |       | G1CH1 |
 * | P2.8  | G0CH1 | G1CH0 |
 * | P2.9  | G0CH2 | G1CH4 |
 * | P2.10 | G0CH3 | G1CH2 |
 * | P2.11 | G0CH4 | G1CH3 |
 * +-------+-------+-------+
 **********************************************************************************************************************/
/* Change VADC channel number and group  number as per the pins */

/** Configuration Macros for DC link current measurement */
#define IDC_LINK_GRP                 (VADC_G1)
#define IDC_LINK_CH_NUM              (2U)    /* P2.10, G1CH2 */

/** Configuration Macros for DC link voltage measurement */
#define VDC_LINK_GRP                 (VADC_G1)
#define VDC_LINK_CH_NUM              (5U)    /* P2.3, G1CH5 */

/** Configuration Macros for Analog input - POT measurement */
#define POT_INPUT_GRP                (VADC_G1)
#define POT_INPUT_CH_NUM             (7U)    /* P2.5, G1CH7 */

/** Configuration Macros for temperature sensor Channel */
#define TEMP_SENSE_GRP               (VADC_G1)
#define TEMP_SENSE_CH_NUM            (6U)    /* P2.4, G1CH6 */

/** Configuration Macros for reference kit select Channel */
#define KIT_SELECT_GRP               (VADC_G0)
#define KIT_SELECT_CH_NUM            (0U)    /* P2.6, G0CH0 */

/** Configuration Macros for VADC Channel */
#define USER_DEF1_GRP                (VADC_G1)
#define USER_DEF1_CH_NUM             (6U)    /* P2.4 VADC Pin */

/** Configuration Macros for VADC Channel */
#define USER_DEF2_GRP                (VADC_G1)
#define USER_DEF2_CH_NUM             (1U)

/** Configuration Macros for VADC Channel */
#define USER_DEF3_GRP                (VADC_G1)
#define USER_DEF3_CH_NUM             (3U)

/*
 * Queue configuration - Queue configuration will be applicable to corresponding group.
 * For Example Queue0 configuration is applicable for Group0 channels conversion.
 */
/** VADC Queue0 - Gating and trigger configuration */
/* CCU8x compare match signal used for queue0 trigger */
#define MOTOR0_BLDC_SCALAR_VADC_QUEUE_0_TRIGGER_SIGNAL    (10) //(XMC_CCU_81_SR2)-is not in map file but exists in concept diagrams

/** VADC Queue1 - Gating and trigger configuration */
/* CCU8x compare match signal used for queue1 trigger */
#define MOTOR0_BLDC_SCALAR_VADC_QUEUE_1_TRIGGER_SIGNAL    (10) //(XMC_CCU_81_SR2)-is not in map file but exists in concept diagrams

/**********************************************************************************************************************
 * Data Structures
 **********************************************************************************************************************/
/**
 *  @brief This structure holds, configuration structure and parameters for CCU8 slice configuration.
 * Object of this structure is used for PWM init configuration.
 */
typedef struct
{
    CCU8_GLOBAL_TypeDef     *ccu8;                 /* CCU8 pointer */
    XMC_CCU8_SLICE_t        *ccu8_slice[3];        /* CCU8 phase slice pointer */
    uint8_t                 ccu8_slice_num[3];     /* CCU8 phase slice number */
    uint16_t                shadow_transfer;       /* CCU8 shadow transfer value */
    CCU4_GLOBAL_TypeDef     *ccu4;                 /* CCU4 pointer */
    uint8_t                 capture_slice_num;
    uint8_t                 mcmsync_slice_num;
    XMC_CCU4_SLICE_t        *capture_slice;        /* CCU4 capture slice for hall speed calculation */
    XMC_CCU4_SLICE_t        *mcmsync_slice;        /* CCU4 delay timer slice for multi-channel pattern update */
    POSIF_GLOBAL_TypeDef    *posif;                /* POSIF pointer */
} bldc_peripheral_ptr_t;
extern bldc_peripheral_ptr_t PeriPtr;

/***********************************************************************************************************************
 * API PROTOTYPES
 **********************************************************************************************************************/
/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Initializes required peripherals (CCU8, CCU4, POSIF, SYSTICK) and prepares for
 * the BLDC motor control
 */
void peripheral_init_xmc14(void);
uint16_t read_kit_id_xmc14(void);
uint32_t check_chip_id_xmc14(void);
void idc_trigger_conversion_xmc14(void);

__STATIC_INLINE uint16_t get_adc_i_shunt_result_xmc14(void)
{
    return IDC_LINK_GRP->RES[IDC_LINK_CH_NUM];
}

__STATIC_INLINE uint16_t get_adc_vbus_result_xmc14(void)
{
    return VDC_LINK_GRP->RES[VDC_LINK_CH_NUM];
}

__STATIC_INLINE uint16_t get_adc_temp_result_xmc14(void)
{
    return TEMP_SENSE_GRP->RES[TEMP_SENSE_CH_NUM];
}

__STATIC_INLINE uint16_t get_adc_pot_result_xmc14(void)
{
    return POT_INPUT_GRP->RES[POT_INPUT_CH_NUM];
}

__STATIC_INLINE void set_gpio_output_level_xmc14(XMC_GPIO_PORT_t *const port, const uint8_t pin, const uint8_t level)
{
    port->OUT = level << pin;
}
