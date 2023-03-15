/******************************************************************************
* File Name: 6EDL_gateway.c
*
* Description: UART communication to GUI
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
#include <xmc_flash.h>
#include "01_TopLevel/user_input_config.h"
#include "02_Lib_6EDL7141/6EDL_gateway.h"
#include "02_Lib_6EDL7141/6EDL_peripheral_xmc14.h"

edl_io_control_t EdlIo;
edl7141_register_t Edl7141Reg;

#if (PROGRAM_DEFAULT_PARAM == 1)
void edl7141_parameter_set_default(void)
{
    Edl7141Reg.FAULTS_CLR   = 0                   << FAULTS_CLR_CLR_FLTS_Pos |
                              0                   << FAULTS_CLR_CLR_LATCH_Pos;
    Edl7141Reg.SUPPLY_CFG   = PVCC_12V            << SUPPLY_CFG_PVCC_SETPT_Pos |
                              VREF_DVDD_1_4       << SUPPLY_CFG_CS_REF_CFG_Pos |
                              OCP_THR_450         << SUPPLY_CFG_DVDD_OCP_CFG_Pos |
                              DVDD_SFTSTRT_100us  << SUPPLY_CFG_DVDD_SFTSTRT_Pos |
                              DVDD_VSENSE         << SUPPLY_CFG_DVDD_SETPT_Pos |
                              BK_FREQ_500K        << SUPPLY_CFG_BK_FREQ_Pos;
    Edl7141Reg.ADC_CFG      = 0                   << ADC_CFG_ADC_OD_REQ_Pos |
                              ADC_IN_IDIGITAL     << ADC_CFG_ADC_OD_INSEL_Pos |
                              0                   << ADC_CFG_ADC_EN_FILT_Pos |
                              ADC_FILT_SAMP_8     << ADC_CFG_ADC_FILT_CFG_Pos |
                              ADC_FILT_PSAMP_32   << ADC_CFG_ADC_FILT_CFG_PVDD_Pos;
    Edl7141Reg.PWM_CFG      = PWM_COMM_MODE_6     << PWM_CFG_PWM_MODE_Pos |
                              ACTIVE_FREEW        << PWM_CFG_PWM_FREEW_CFG_Pos |
                              BRAKE_LOW           << PWM_CFG_BRAKE_CFG_Pos |
                              BRAKE_RECIRC_DIS    << PWM_CFG_PWM_RECIRC_Pos;
    Edl7141Reg.SENSOR_CFG   = HALL_DEGLITCH_640ns << SENSOR_CFG_HALL_DEGLITCH_Pos |
                              OTEMP_PROT_EN       << SENSOR_CFG_OTS_DIS_Pos;
    Edl7141Reg.WD_CFG       = WD_DIS              << WD_CFG_WD_EN_Pos |
                              WDIN_DRV            << WD_CFG_WD_INSEL_Pos |
                              WDOUT_STATUS        << WD_CFG_WD_FLTCFG_Pos |
                              WD_PERIOD_100us     << WD_CFG_WD_TIMER_T_Pos;
    Edl7141Reg.WD_CFG2      = WD_REACT_NFAULT     << WD_CFG2_WD_BRAKE_Pos |
                              WD_FLT_LATCH_DIS    << WD_CFG2_WD_EN_LATCH_Pos |
                              RSTRT_ATT0          << WD_CFG2_WD_DVDD_RSTRT_ATT_Pos |
                              DVDD_RSTRT_DLY_0ms5 << WD_CFG2_WD_DVDD_RSTRT_DLY_Pos |
                              RLOCK_DIS           << WD_CFG2_WD_RLOCK_EN_Pos |
                              RLOCK_WD_TMOUT_1s   << WD_CFG2_WD_RLOCK_T_Pos |
                              BUCK_EN             << WD_CFG2_WD_BK_DIS_Pos;
    Edl7141Reg.IDRIVE_CFG   = IDRIVE_150mA         << IDRIVE_CFG_IHS_SRC_Pos |
                              IDRIVE_200mA        << IDRIVE_CFG_IHS_SINK_Pos |
                              IDRIVE_150mA         << IDRIVE_CFG_ILS_SRC_Pos |
                              IDRIVE_200mA        << IDRIVE_CFG_ILS_SINK_Pos;
    Edl7141Reg.IDRIVE_PRE_CFG  = IDRIVE_PRE_300mA << IDRIVE_PRE_CFG_I_PRE_SRC_Pos |
                                 IDRIVE_PRE_500mA << IDRIVE_PRE_CFG_I_PRE_SINK_Pos |
                                 PRECHAR_MODE_EN  << IDRIVE_PRE_CFG_I_PRE_EN_Pos;
    Edl7141Reg.TDRIVE_SRC_CFG  = TDRIVE1_100ns      << TDRIVE_SRC_CFG_TDRIVE1_Pos |
                                 TDRIVE2_550ns    << TDRIVE_SRC_CFG_TDRIVE2_Pos;
    Edl7141Reg.TDRIVE_SINK_CFG = TDRIVE3_500ns      << TDRIVE_SINK_CFG_TDRIVE3_Pos |
                                 TDRIVE4_200ns    << TDRIVE_SINK_CFG_TDRIVE4_Pos;
    Edl7141Reg.DT_CFG       = DT_120ns            << DT_CFG_DT_RISE_Pos |
                              DT_120ns            << DT_CFG_DT_FALL_Pos;
    Edl7141Reg.CP_CFG       = CP_CLK_781_25kHz    << CP_CFG_CP_CLK_CFG_Pos |
                              CP_CLK_SS_EN        << CP_CFG_CP_CLK_SS_DIS_Pos;
    Edl7141Reg.CSAMP_CFG    = CS_GAIN_12V         << CSAMP_CFG_CS_GAIN_Pos |
                              CS_GAIN_PROG_DIG    << CSAMP_CFG_CS_GAIN_ANA_Pos |
                              CS_A_DIS_B_EN_C_DIS << CSAMP_CFG_CS_EN_Pos |
                              CS_BLANK_1us        << CSAMP_CFG_CS_BLANK_Pos |
                              CS_CALIB_DIS        << CSAMP_CFG_CS_EN_DCCAL_Pos |
                              CS_DEGLITCH_8us     << CSAMP_CFG_CS_OCP_DEGLITCH_Pos |
                              OCP_FLT_TRIG_NONE   << CSAMP_CFG_CS_OCPFLT_CFG_Pos;
    Edl7141Reg.CSAMP_CFG2   = OCP_POS_THR_70mV    << CSAMP_CFG2_CS_OCP_PTHR_Pos |
                              OCP_NEG_THR_300mV   << CSAMP_CFG2_CS_OCP_NTHR_Pos |
                              OCP_FLT_LATCH_DIS   << CSAMP_CFG2_CS_OCP_LATCH_Pos |
                              CS_SENSE_SHUNT_RES  << CSAMP_CFG2_CS_MODE_Pos |
                              OCP_FLT_BRAKE_DIS   << CSAMP_CFG2_CS_OCP_BRAKE_Pos |
                              OCP_PWM_TRUNC_DIS   << CSAMP_CFG2_CS_TRUNC_DIS_Pos |  /* OCP_PWM_TRUNC_EN_POS: Enable cycle-by-cycle current limit */
                              CS_VREF_INT         << CSAMP_CFG2_VREF_INSEL_Pos |
                              OCP_NEG_DIS         << CSAMP_CFG2_CS_NEG_OCP_DIS_Pos |
                              CS_AUTOZERO_INT     << CSAMP_CFG2_CS_AZ_CFG_Pos;
    Edl7141Reg.OTP_PROG     = 0                   << OTP_PROG_OTP_PROG_Pos |
                              0                   << OTP_PROG_USER_ID_Pos;          /* write a value different to the value in OTP, this helps to detect unexpected IC reset */
    Edl7141Reg.chip_version = EDL7141_CHIP_VERSION;
}
#endif //PROGRAM_DEFAULT_PARAM

void load_edl7141_flash_parameter(void)
{
    /* MC_INFO structure contains expected parameter version for the firmware. Head of parameter block should contain
     * same value, otherwise parameter block won't be loaded
     */
#if (PROGRAM_DEFAULT_PARAM == 1)
    /* For test only, program the parameter block with default value */
    /* Configure with default parameter */
    edl7141_parameter_set_default();
    /* Write default parameter into parameter block */
    XMC_FLASH_ProgramVerifyPage(Edl7141ParameterBlock_Addr, (uint32_t*)&Edl7141Reg);
    SystemVar.Edl7141Configured = 1;
#else
    Edl7141Reg.chip_version = ((edl7141_register_t*)Edl7141ParameterBlock_Addr)->chip_version;
    if (Edl7141Reg.chip_version != EDL7141_CHIP_VERSION)
    {
        /* If there is no valid parameter block */
        /* Do nothing, state machine will stay at IDLE state, until all parameters (including chip_version) are configured by master control */
        SystemVar.Edl7141Configured = 0;
    }
    else
    {
        /* Found parameter block with matching version, copy parameter block data into Edl7141Reg structure */
        edl7141_register_t *parameter_block = (edl7141_register_t*)Edl7141ParameterBlock_Addr;
        for (int32_t i = 0; i <= CFG_ADDR_MAX; i++)
        {
            Edl7141Reg.table[i] = parameter_block->table[i];
        }
        SystemVar.Edl7141Configured = 1;
    }
#endif //PROGRAM_DEFAULT_PARAM
}


/* To read all the 6EDL7141 registers when startup*/
void spi_read_6EDL7141_registers(void)
{
    /* Read all status registers and configure registers */
    for (uint32_t addr = ST_ADDR_MIN; addr < CFG_ADDR_MAX; addr++)
    {
        if (addr == ST_ADDR_MAX + 1)
        {
            addr = CFG_ADDR_MIN;    /* skip the gap between status register and configure register */
        }
        Edl7141Reg.table[addr] = Read_Word_16b(addr);
    }
}

/* To write the 6EDL7141 configuration registers to default values */
void spi_write_6EDL7141_registers(void)
{
    uint32_t addr;
    /* Write default parameters, exclude: FAULTS_CLR & OTP_PROG */
    for (addr = CFG_ADDR_MIN + 1; addr < CFG_ADDR_MAX; addr++)
    {
        Write_Word_16b(addr, Edl7141Reg.table[addr]);
    }
    addr = ADDR_FAULTS_CLR;
    Write_Word_16b(addr, 3);    /* clear fault */
}

/* Initialize SPI for 6EDL communication, write configure register value into device, read back all register values */
void edl7141_gateway_init(void)
{
    EdlIo.en_drv_level = 0;
    EdlIo.nbrake_level = 1;

    /* Initialize pins interfacing 6EDL7141 */
    gate_driver_gpio_init();
    /* Initializes SPI for 6EDL7141 */
    gate_driver_spi_master_init();
}

extern uint8_t error_SPI;
static uint8_t StatusRegister_count = 0;
static uint8_t ConfigRegister_count = 0;
uint8_t GUIwrReg_6EDL7141_addr = {0x55};
uint8_t WriteReg_6EDL7141_addr = {0x55};
uint16_t GUIwrReg_6EDL7141_data;
uint16_t WriteReg_6EDL7141_data;
uint8_t GuiMonitor_6EDL7141_addr;
uint16_t GuiMonitor_6EDL7141_value;

/* To communicate with PC GUI 6EDL Configurator */
void edl7141_update(void)
{
    uint16_t data;
    uint8_t addr;

    if (GUIwrReg_6EDL7141_addr <= CFG_ADDR_MAX)         /* Handle write request from GUI */
    {
        addr = GUIwrReg_6EDL7141_addr;
        Write_Word_16b(addr, GUIwrReg_6EDL7141_data);   /* Write to 6EDL7141 */
        data = Read_Word_16b(addr);                     /* Read back from 6EDL7141 */
        Edl7141Reg.table[addr] = data;                  /* Update register table */
        GUIwrReg_6EDL7141_addr = 0x55;                  /* Indicate write operation is done */
    }
    if (WriteReg_6EDL7141_addr <= CFG_ADDR_MAX)         /* Handle write request from control */
    {
        addr = WriteReg_6EDL7141_addr;
        Write_Word_16b(addr, WriteReg_6EDL7141_data);   /* Write to 6EDL7141 */
        data = Read_Word_16b(addr);                     /* Read back from 6EDL7141 */
        Edl7141Reg.table[addr] = data;                  /* Update register table */
        WriteReg_6EDL7141_addr = 0x55;                  /* Indicate write operation is done */
    }

    /* Every 1ms, always read fault status register */
    Edl7141Reg.table[ADDR_FAULT_ST] = Read_Word_16b(ADDR_FAULT_ST);

    if (StatusRegister_count <= 6)
    {
        /* Every 1ms, read one of remaining 7 status register from address 1-7 */
        StatusRegister_count++;
        Edl7141Reg.table[StatusRegister_count] = Read_Word_16b(StatusRegister_count);
    }
    else
    {
        /* After read all the status registers, read one of configure registers */
        StatusRegister_count = 0;
        addr = ConfigRegister_count + CFG_ADDR_MIN;
        Edl7141Reg.table[addr] = Read_Word_16b(addr);   /* Read one of configure registers */

        if (ConfigRegister_count >= CFG_ADDR_MAX - CFG_ADDR_MIN)
        {
            /* Reached end of configure register, reset counter */
            ConfigRegister_count = 0;
            /* Read OTP_PROG[USER_ID], if value been reset to OTP value, there is 6EDL chip reset, report fault */
            if (Edl7141Reg.OTP_PROG_USER_ID != 0xF)
            {
                /* report 6EDL chip reset fault if not in IDLE state */
                if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_IDLE)
                {
                    MotorVar.error_status |= 1 << BLDC_SCALAR_EID_GD_RESET;
                }
            }
        }
        else
        {
            /* Advance to next configure register */
            ConfigRegister_count++;
        }

        /* Update 6EDL GUI monitor value */
        GuiMonitor_6EDL7141_value = Edl7141Reg.table[GuiMonitor_6EDL7141_addr];
    }

    /* fault handling */
    if (error_SPI)
    {
        MotorVar.error_status |= 1 << BLDC_SCALAR_EID_SPI_FAULT;
        error_SPI = 0;
    }

    /* Control EN_DRV pin output */
    set_gpio_output_level_xmc14(CYBSP_GD_EN_DRV_PORT, CYBSP_GD_EN_DRV_PIN, EdlIo.en_drv_level);
    set_gpio_output_level_xmc14(CYBSP_GD_nBRAKE_PORT, CYBSP_GD_nBRAKE_PIN, EdlIo.nbrake_level);

    /* Toggle EN_DRV_CLK output as 500Hz 6EDL7141 watchdog clock */
    XMC_GPIO_ToggleOutput(CYBSP_GD_WD_CLK_PORT, CYBSP_GD_WD_CLK_PIN);
}

