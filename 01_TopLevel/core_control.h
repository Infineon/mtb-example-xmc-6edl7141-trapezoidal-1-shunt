/******************************************************************************
* File Name: core_control.h
*
* Description: This header file contains high level feature settings 
*              and data structure
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

#include "peripheral_xmc14.h"


/*********************************************************************************************************************
 *                                      Control configuration
 ********************************************************************************************************************/
#define DISABLED            (0U)
#define ENABLED             (1U)

#define MOTOR_START_CMD     (1U)
#define MOTOR_STOP_CMD      (0U)

/*********************************************************************************************************************
 *                                      Configuration of 6EDL7141 & MCU
 ********************************************************************************************************************/

#define MOTOR_PARAM_VERSION             (11)
#define FIRMWARE_VERSION                (0x0003)

/* Parameter block address in flash memory */
#define MotorParameterBlock_Addr        (uint32_t*)0x10010100
#define Edl7141ParameterBlock_Addr      (uint32_t*)0x10010000

/**
 * This enumerates the error codes of the control which can occur during run-time.
 */
typedef enum BLDC_SCALAR_EID
{
    BLDC_SCALAR_EID_CTRAP_ERROR = 0U,               /*!< Trap condition is detected.  error_status = 1*/
    BLDC_SCALAR_EID_WRONGHALL = 3U,                 /*!< Wrong hall is detected. error_status = 8 */
    BLDC_SCALAR_EID_HALL_LEARNING_TIMEOUT = 4U,     /*!< Hall pattern capturing failed. error_status = 16 */
    BLDC_SCALAR_EID_STALL = 5U,                     /*!< Motor stall is detected. error_status = 32 */
    BLDC_SCALAR_EID_OVER_VOLTAGE = 6U,              /*!< Over voltage is detected. error_status = 64 */
    BLDC_SCALAR_EID_UNDER_VOLTAGE = 10U,            /*!< DC link under voltage, error_status = 1024 */
    BLDC_SCALAR_EID_SPI_FAULT = 11U,                /*!< SPI communication with 6EDL7141 fault, error status = 2048 */
    BLDC_SCALAR_EID_GD_RESET = 12U,                 /*!< 6EDL7141 reset fault, error status = 4096 */
} BLDC_SCALAR_EID_t;

typedef struct
{
    uint32_t  chip_id;                  /* MCU device ID */
    uint16_t  parameter_version;        /* Parameter version, BLDC control starts from 0x0000, FOC control starts from 0x1000 */
    uint16_t  firmware_version;         /* Firmware version */
    uint8_t   kit_id;                   /* Hardware kit ID: 0: Unidentified, 1: 1S 6EDL7141 v2, 2: 3S 6EDL7141 */
} mc_info_t;
extern mc_info_t MC_INFO;

typedef struct
{
    union
    {
        struct
        {
            uint32_t MotorConfigured:1;	    /* This bit indicates parameters are configured either from flash or configured by GUI */
            uint32_t Edl7141Configured:1;   /* This bit indicates 6EDL7141 configure registers are configured either from flash or by GUI */
            uint32_t OffsetCalibrated:1;    /* This bit indicates current sense offset is calibrated */
            uint32_t ReInitParam:1;         /* This bit can be set by GUI or other code re-init peripheral configuration */
        };
        uint32_t Status;
    };
    void (*ControlLoop)(void);
    uint32_t GlobalTimer;                   /* Free running timer counts up every systick */
} system_var_t;
extern system_var_t SystemVar;

void load_motor_flash_parameter(void);
