/******************************************************************************
* File Name: register_map.c
*
* Description: Define signals available for display in GUI's oscilloscope
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
#include "02_Lib_6EDL7141/6EDL_gateway.h"
#include "02_GUI_Interface/uCProbe.h"
#include "register_map.h"
#include "01_TopLevel/core_control.h"
#include "04_Lib_BLDC/lib_bldc.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/


/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/


/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
uint32_t Reserved = 0;
void* const RegisterAddrMap[REGISTER_MAP_SIZE] =
{
/*************************** BLDC motor control registers ***********************/
  /*0*/   (void*)&MotorVar.hall_pattern, /* Hall pattern */
  /*1*/   (void*)&Motor0_BLDC_SCALAR_ucprobe.motor_current_mA, /* Motor current (mA) */
  /*2*/   (void*)&Motor0_BLDC_SCALAR_ucprobe.motor_speed_RPM, /* Motor speed (RPM) */
  /*3*/   (void*)&Motor0_BLDC_SCALAR_ucprobe.dclinkgvoltage_Volt, /* DC voltage */
  /*4*/   (void*)&MotorVar.TargetValue, /* Target value */
  /*5*/   (void*)&MotorVar.voltage_command, /* Voltage output */
  /*6*/   (void*)&Motor0_BLDC_SCALAR.t_sensor.output, /* Temperature sensor, degree C in Q4 */
  /*7*/   (void*)&Reserved,
  /*8*/   (void*)&Reserved,
  /*9*/   (void*)&Reserved,
  /*10*/  (void*)&Reserved,
  /*11*/  (void*)&Reserved,
  /*12*/  (void*)&Reserved,
  /*13*/  (void*)&Reserved,
  /*14*/  (void*)&Reserved,
  /*15*/  (void*)&Reserved,
  /*16*/  (void*)&Reserved,
  /*17*/  (void*)&Reserved,
  /*18*/  (void*)&Reserved,
  /*19*/  (void*)&Reserved,
  /*20*/  (void*)&Reserved,
  /*21*/  (void*)&Reserved,
  /*22*/  (void*)&Reserved,
  /*23*/  (void*)&Reserved,
  /*24*/  (void*)&Reserved,
  /*25*/  (void*)&Reserved,
  /*26*/  (void*)&Reserved,
  /*27*/  (void*)&Reserved,
  /*28*/  (void*)&Reserved,
  /*29*/  (void*)&Reserved,
  /*30*/  (void*)&Reserved,
  /*31*/  (void*)&Reserved,
  /*32*/  (void*)&Reserved,
  /*33*/  (void*)&Reserved,
  /*34*/  (void*)&Reserved,
  /*35*/  (void*)&Reserved,
  /*36*/  (void*)&Reserved,
  /*37*/  (void*)&Reserved,
  /*38*/  (void*)&Reserved,
  /*39*/  (void*)&Reserved,
  /*40*/  (void*)&Reserved,
  /*41*/  (void*)&Reserved,
  /*42*/  (void*)&Reserved,
  /*43*/  (void*)&Reserved,
  /*44*/  (void*)&Reserved,
  /*45*/  (void*)&Reserved,
  /*46*/  (void*)&Reserved,
  /*47*/  (void*)&Reserved,
  /*48*/  (void*)&Reserved,
  /*49*/  (void*)&Reserved,
  /*50*/  (void*)&Reserved,
  /*************************** 6EDL7141 read registers ***********************/
  /*51*/  (void*)&Edl7141Reg.table[0x00],      /*  0. Read FAULT_ST register */
  /*52*/  (void*)&Edl7141Reg.table[0x01],      /*  1. Read TEMP_ST register */
  /*53*/  (void*)&Edl7141Reg.table[0x02],      /*  2. Read SUPPLY_ST register */
  /*54*/  (void*)&Edl7141Reg.table[0x03],      /*  3. Read FUNCT_ST register */
  /*55*/  (void*)&Edl7141Reg.table[0x04],      /*  4. Read OTP_ST register */
  /*56*/  (void*)&Edl7141Reg.table[0x05],      /*  5. Read ADC_ST register */
  /*57*/  (void*)&Edl7141Reg.table[0x06],      /*  6. Read CP_ST register */
  /*58*/  (void*)&Edl7141Reg.table[0x07],      /*  7. Read DEVICE_ID register */
  /*************************** 6EDL7141 write registers ***********************/
  /*59*/  (void*)&Edl7141Reg.table[0x10],      /* 16. Read FAULT_CLR register */
  /*60*/  (void*)&Edl7141Reg.table[0x11],      /* 17. Read SUPPLY_CFG register */
  /*61*/  (void*)&Edl7141Reg.table[0x12],      /* 18. Read ADC_CFG register */
  /*62*/  (void*)&Edl7141Reg.table[0x13],      /* 19. Read PWM_CFG register */
  /*63*/  (void*)&Edl7141Reg.table[0x14],      /* 20. Read SENSOR_CFG register */
  /*64*/  (void*)&Edl7141Reg.table[0x15],      /* 21. Read WD_CFG register */
  /*65*/  (void*)&Edl7141Reg.table[0x16],      /* 22. Read WD_CFG2 register */
  /*66*/  (void*)&Edl7141Reg.table[0x17],      /* 23. Read IDRIVE_CFG register */
  /*67*/  (void*)&Edl7141Reg.table[0x18],      /* 24. Read IDRIVE_PRE_CFG register */
  /*68*/  (void*)&Edl7141Reg.table[0x19],      /* 25. Read TDRIVE_SRC_CFG register */
  /*69*/  (void*)&Edl7141Reg.table[0x1A],      /* 26. Read TDRIVE_SINK_CFG register */
  /*70*/  (void*)&Edl7141Reg.table[0x1B],      /* 27. Read DT_CFG register */
  /*73*/  (void*)&Edl7141Reg.table[0x1C],      /* 28. Read CP_PROG register */
  /*71*/  (void*)&Edl7141Reg.table[0x1D],      /* 29. Read CSAMP_CFG register */
  /*72*/  (void*)&Edl7141Reg.table[0x1E],      /* 30. Read CSAMP_CFG2 register */
  /*74*/  (void*)&Edl7141Reg.table[0x1F],      /* 31. Read OTP_PROG register */
};

enum {SCOPE_INT8U, SCOPE_INT8S, SCOPE_INT16U, SCOPE_INT16S, SCOPE_INT32U, SCOPE_INT32S, SCOPE_FP32};
register_type_map_t const RegisterTypeMap[REGISTER_MAP_SIZE] =
{
/*************************** BLDC motor control registers ***********************/
  /*0*/   {SCOPE_INT8U, DISABLED, 0},
  /*1*/   {SCOPE_INT32U, DISABLED, 0},
  /*2*/   {SCOPE_INT32U, DISABLED, 0},
  /*3*/   {SCOPE_INT32U, DISABLED, 0},
  /*4*/   {SCOPE_INT16S, DISABLED, 0},
  /*5*/   {SCOPE_INT16S, DISABLED, 0},
  /*6*/   {SCOPE_INT16S, DISABLED, 0},
  /*7*/   {SCOPE_INT32U, DISABLED, 0},
  /*8*/   {SCOPE_INT32U, DISABLED, 0},
  /*9*/   {SCOPE_INT32U, DISABLED, 0},
  /*10*/  {SCOPE_INT32U, DISABLED, 0},
  /*11*/  {SCOPE_INT32U, DISABLED, 0},
  /*12*/  {SCOPE_INT32U, DISABLED, 0},
  /*13*/  {SCOPE_INT32U, DISABLED, 0},
  /*14*/  {SCOPE_INT32U, DISABLED, 0},
  /*15*/  {SCOPE_INT32U, DISABLED, 0},
  /*16*/  {SCOPE_INT32U, DISABLED, 0},
  /*17*/  {SCOPE_INT32U, DISABLED, 0},
  /*18*/  {SCOPE_INT32U, DISABLED, 0},
  /*19*/  {SCOPE_INT32U, DISABLED, 0},
  /*20*/  {SCOPE_INT32U, DISABLED, 0},
  /*21*/  {SCOPE_INT32U, DISABLED, 0},
  /*22*/  {SCOPE_INT32U, DISABLED, 0},
  /*23*/  {SCOPE_INT32U, DISABLED, 0},
  /*24*/  {SCOPE_INT32U, DISABLED, 0},
  /*25*/  {SCOPE_INT32U, DISABLED, 0},
  /*26*/  {SCOPE_INT32U, DISABLED, 0},
  /*27*/  {SCOPE_INT32U, DISABLED, 0},
  /*28*/  {SCOPE_INT32U, DISABLED, 0},
  /*29*/  {SCOPE_INT32U, DISABLED, 0},
  /*30*/  {SCOPE_INT32U, DISABLED, 0},
  /*31*/  {SCOPE_INT32U, DISABLED, 0},
  /*32*/  {SCOPE_INT32U, DISABLED, 0},
  /*33*/  {SCOPE_INT32U, DISABLED, 0},
  /*34*/  {SCOPE_INT32U, DISABLED, 0},
  /*35*/  {SCOPE_INT32U, DISABLED, 0},
  /*36*/  {SCOPE_INT32U, DISABLED, 0},
  /*37*/  {SCOPE_INT32U, DISABLED, 0},
  /*38*/  {SCOPE_INT32U, DISABLED, 0},
  /*39*/  {SCOPE_INT32U, DISABLED, 0},
  /*40*/  {SCOPE_INT32U, DISABLED, 0},
  /*41*/  {SCOPE_INT32U, DISABLED, 0},
  /*42*/  {SCOPE_INT32U, DISABLED, 0},
  /*43*/  {SCOPE_INT32U, DISABLED, 0},
  /*44*/  {SCOPE_INT32U, DISABLED, 0},
  /*45*/  {SCOPE_INT32U, DISABLED, 0},
  /*46*/  {SCOPE_INT32U, DISABLED, 0},
  /*47*/  {SCOPE_INT32U, DISABLED, 0},
  /*48*/  {SCOPE_INT32U, DISABLED, 0},
  /*49*/  {SCOPE_INT32U, DISABLED, 0},
  /*50*/  {SCOPE_INT32U, DISABLED, 0},
  /*************************** 6EDL7141 registers ***********************/
  /*51*/  {SCOPE_INT16U, DISABLED, 0},
  /*52*/  {SCOPE_INT16U, DISABLED, 0},
  /*53*/  {SCOPE_INT16U, DISABLED, 0},
  /*54*/  {SCOPE_INT16U, DISABLED, 0},
  /*55*/  {SCOPE_INT16U, DISABLED, 0},
  /*56*/  {SCOPE_INT16U, DISABLED, 0},
  /*57*/  {SCOPE_INT16U, DISABLED, 0},
  /*58*/  {SCOPE_INT16U, DISABLED, 0},
  /*59*/  {SCOPE_INT16U, DISABLED, 0},
  /*60*/  {SCOPE_INT16U, DISABLED, 0},
  /*61*/  {SCOPE_INT16U, DISABLED, 0},
  /*62*/  {SCOPE_INT16U, DISABLED, 0},
  /*63*/  {SCOPE_INT16U, DISABLED, 0},
  /*64*/  {SCOPE_INT16U, DISABLED, 0},
  /*65*/  {SCOPE_INT16U, DISABLED, 0},
  /*66*/  {SCOPE_INT16U, DISABLED, 0},
  /*67*/  {SCOPE_INT16U, DISABLED, 0},
  /*68*/  {SCOPE_INT16U, DISABLED, 0},
  /*69*/  {SCOPE_INT16U, DISABLED, 0},
  /*70*/  {SCOPE_INT16U, DISABLED, 0},
  /*71*/  {SCOPE_INT16U, DISABLED, 0},
  /*72*/  {SCOPE_INT16U, DISABLED, 0},
  /*73*/  {SCOPE_INT16U, DISABLED, 0},
  /*74*/  {SCOPE_INT16U, DISABLED, 0},
};
