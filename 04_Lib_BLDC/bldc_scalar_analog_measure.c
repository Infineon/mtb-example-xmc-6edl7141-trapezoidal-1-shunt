/******************************************************************************
* File Name: bldc_scalar_analog_measure.c
*
* Description: User defined voltage measurement
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
#include "03_Application_BLDC/application_bldc.h"
#include "04_Lib_BLDC/lib_bldc.h"

/***********************************************************************************************************************
 * GLOBAL DATA
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
/**
 * @param dclink_current Motor DC link current value in Q14 format
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Updates the reference variable with VADC conversion result for DC link current measurement.
 * DC link current measurement is skipped for DEMAG_BLANKING_COUNT to avoid wrong readings at the commutation point.
 *
 */
void Motor0_BLDC_SCALAR_GetCurrentValue(void)
{
    /*
     * Skip the direct DC link current measurement at commutation point based on de-magnetization count
     * This is required only if the SW filter is disabled
     */
    if (MotorVar.demag_blanking_counter != 0)
    {
        MotorVar.demag_blanking_counter--;
    }
    else
    {
        int32_t current_value;
        current_value = get_adc_i_shunt_result_xmc14();
        current_value = (int32_t) (((current_value - MotorVar.direct_dc_amplifier_offset) * (int32_t) MotorParam.CURRENT_ADC_SCALE) >> 14);

      if (MotorVar.voltage_command_prev >= 0)
      {
          MotorVar.MotorCurrent = current_value;
      }
      else
      {
          MotorVar.MotorCurrent = current_value * -1;
      }

      /* Low pass filter */
      MotorVar.DcCurrentFilter.sum += ((MotorVar.MotorCurrent - MotorVar.DcCurrentFilter.output) * MotorParam.IDC_FILTER_GAIN) << 2;
    }
}

/* Current sense amplifier offset calibration */
#define BLDC_SCALAR_ADCCAL_COUNT            (16U)
#define BLDC_SCALAR_VADC_CONVERSION_DELAY   (0xFU)      /* VADC conversion delay time */
void offset_calibration(void)
{
    uint32_t amp_offset = 0U;   /* current amplifier offset value */
    uint32_t count;             /* for loop count */
    uint32_t delay;             /* ADC conversion delay counter */

    /* Direct DC link current */
    amp_offset = 0U;
    /* SW trigger for direct DC link current channel and measure the amplifier bias voltage */
    /* Calibration is done with average of 16 measurements */
    for (count = 0U; count < BLDC_SCALAR_ADCCAL_COUNT; count++)
    {
        idc_trigger_conversion_xmc14();
        /* VADC queue measurements conversion time delay */
        for (delay = 0U; delay < BLDC_SCALAR_VADC_CONVERSION_DELAY; delay++)
        {
  
        }
        amp_offset += get_adc_i_shunt_result_xmc14();
    }
    amp_offset >>= 4;       /* Average 16 samples */
    MotorVar.direct_dc_amplifier_offset = amp_offset;
}

void read_inverter_temperature(void)
{
    /*****************************************************
     * Read analog input from MCP9700 temperature sensor
     *     0°C = 500mV
     *     sensitivity = 10mV/°C
     * For DVDD=3.3V, 10mV = 12.41212 (ADC cts)
     * For DVDD=5.0V, 10mV = 8.192 (ADC cts)
     * t_sensor is °C in Q4, conversion from ADC counts to t_sensor:
     *     DVDD=3.3V, 16/12.41212 = 1.289 ~= 660 >> 9
     *     DVDD=5.0V, 16/8.192 = 1.953125 = 125 >> 6
     ****************************************************/
    int16_t temp_sense_adc_raw = get_adc_temp_result_xmc14();
    int16_t temp_sense_degree_c;
    if (MotorParam.Vadc_Ref_Voltage == 33)
    {
        /* DVDD = 3.3V */
        temp_sense_degree_c = ((temp_sense_adc_raw - 620) * 660) >> 9; /* 660/512=1.289063 */
    }
    else
    {
        /* DVDD = 5.0V */
        temp_sense_degree_c = ((temp_sense_adc_raw - 409) * 125) >> 6; /* 125/64=1.953125 */
    }
    Motor0_BLDC_SCALAR.t_sensor.sum += (temp_sense_degree_c - Motor0_BLDC_SCALAR.t_sensor.output) << 10; /* simple low pass filter, time constant = 64 tics */
}
