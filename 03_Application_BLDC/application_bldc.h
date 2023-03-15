/******************************************************************************
* File Name: application_bldc.h
*
* Description: Control algorithm and MCU peripheral configurations 
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

#include  <stdint.h>

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
/** PWM active level */
#define   PWM_ACTIVE_HIGH               (0U)
#define   PWM_ACTIVE_LOW                (1U)

/* Control scheme */
#define   VOLTAGE_CONTROL               (0U)         /*!< Open loop voltage control */
#define   SPEED_CONTROL                 (1U)         /*!< Closed loop speed control - duty is controlled as per speed feedback */
#define   CURRENT_CONTROL               (2U)         /*!< Closed loop current control - duty is controlled as per current feedback */
#define   SPEED_CURRENT_CONTROL         (3U)         /*!< Closed loop current control - duty is controlled as per speed and current feedback */

/* PWM Modulation scheme */
#define   PWM_HIGHSIDE_120_DEG          (0U)         /* high side 120° complementary switching, high side switch is modulated along with complementary PWM to same phase low side switch and second phase low side switch is always ON */
#define   PWM_LOWSIDE_120_DEG           (1U)         /* low side 120° complementary switching, low side switch is modulated along with complementary PWM to same phase high side switch and second phase high side switch is always ON  */
#define   PWM_HIGHSIDE_60_DEG           (2U)         /* high side 60° complementary switching, high side switch is modulated along with complementary PWM to same phase low side switch and second phase low side switch is always ON */

#define   SPEED_PI_SCALE                (15U)        /*!< Scale of Kp and Ki parameters.*/
#define   CURRENT_PI_SCALE              (14U)        /*!< Scale of Kp and Ki parameters */

typedef struct
{
    union
    {
        struct
        {
            uint16_t precision;
            int16_t  output;
        };
        int32_t sum;
    };
} filter_type_t;


/**
 * States of the control algorithm
 */
typedef enum BLDC_SCALAR_MSM
{
    BLDC_SCALAR_MSM_STOP = 0U,                  /*!< Motor Stop state. This is the state after power on reset. */
    BLDC_SCALAR_MSM_BOOTSTRAP = 3U,             /*!< Bootstrap state to charge bootstrap capacitors */
    BLDC_SCALAR_MSM_HALL_LEARNING = 4U,         /*!< Hall pattern detection to find the commutation table*/
    BLDC_SCALAR_MSM_OFFSET_CALIBRATION = 6U,    /*!< Current sense offset calibration */
    BLDC_SCALAR_MSM_NORMAL_OPERATION = 7U,      /*!< Closed loop control */
    BLDC_SCALAR_MSM_ERROR = 8U,                 /*!< Error state. Motor stops running */
    BLDC_SCALAR_MSM_IDLE = 9U                   /*!< Idle state - first state after power up (before parameters are configured */
} BLDC_SCALAR_MSM_t;


/**
 *  @brief Structure contains the motor parameters, measured values
 */
typedef struct BLDC_SCALAR
{
    volatile BLDC_SCALAR_MSM_t msm_state;   /*!< motor state as per the state machine */
    int32_t motor_speed;                    /*!< calculated mechanical motor speed in Q14 format */
    int32_t abs_motor_speed;                /*!< motor speed absolute value in Q14 format */
    int32_t motor_current;                  /*!< measured DC link instantaneous current in Q14 format */
    int32_t motor_average_current;          /*!< measured DC link average current in Q14 format */
    int8_t actual_motor_direction;          /*!< Actual motor direction */
    uint8_t control_loop_exec_counter;      /*!<Control loop execution counter*/
    uint8_t speedcontrol_rate_counter;      /*!< run time counter for speed control execution rate */
    int32_t analogip_val;                   /*!< measured analog input value in Q14 format */

    /* temperature sensor */
    filter_type_t t_sensor;				    /* temperature sensor, degree C in Q4 */
} BLDC_SCALAR_t;
extern BLDC_SCALAR_t Motor0_BLDC_SCALAR;


/*******************************************************************************
 * MotorVar is the structure for motor related running variables.
 ******************************************************************************/
typedef struct
{
    uint8_t Command;
    uint8_t fault_clear;
    uint8_t hall_learning_flag;             /*!< hall pattern detection done*/
    uint8_t hall_pattern;                   /*!< to store previous captured hall pattern - used in wrong hall event to identify hall failure */
    uint8_t hall_sector;
    uint8_t pwm_sector;
    uint8_t phase_advance_flag;             /* if phase advance is 1 mean it is active now */
    uint8_t commutation_flag;               /* 1: indicates a pending commutation */
    uint8_t PwmScheme;                      /* Pwm modulation scheme */
    /* Speed PI variables */
    uint8_t speed_sat;                      /* Speed PI saturation flag */
    int16_t speed_error;
    int16_t speed_pi_output;
    int32_t speed_integral;                 /* Speed PI integral */
    /* Current PI variables */
    uint8_t current_sat;                    /* Current PI saturation flag */
    int16_t current_error;
    int16_t current_pi_output;
    int32_t current_integral;               /* Current PI integral */

    int16_t TargetValue;
    int16_t speed_ref;
    int16_t current_ref;
    int16_t voltage_command;
    int16_t voltage_command_prev;

    uint16_t error_status;
    union
    {
        struct
        {
            uint16_t Trap:1;                /* [0] */
            uint16_t :2;                    /* [2:1] */
            uint16_t WrongHall:1;           /* [3] */
            uint16_t HallLearning:1;        /* [4] */
            uint16_t Stall:1;               /* [5] */
            uint16_t OverVoltage:1;         /* [6] */
            uint16_t :3;                    /* [9:7] */
            uint16_t UnderVoltage:1;        /* [10] */
            uint16_t SpiError:1;            /* [11] */
            uint16_t GdReset:1;             /* [12] */
            uint16_t :2;                    /* [14:13] */
            uint16_t ParameterLoad:1;       /* [15] */
        };
        uint16_t Value;
    } MaskedFault;
    uint16_t PWM_TIMER_RELOAD;              /* PWM timer period reload register value */
    uint16_t PWM_PERIOD;                    /* PWM period in number of clock cycle */
    uint16_t amplitude;                     /*!< actual amplitude in Q14 format */
    uint16_t direct_dc_amplifier_offset;	/*!< external amplifier offset for direct DC link current*/
    uint16_t avg_dc_amplifier_offset;    	/*!< external amplifier offset for average current*/
    int16_t MotorCurrent;                   /* Motor current raw value in Q14 */
    filter_type_t DCLinkVoltageFilter;      /*!< measured DC link voltage in Q14 format, after LPF filter */
    filter_type_t DcCurrentFilter;          /* Filtered DC link current in Q14 */
    filter_type_t PotentiometerFilter;      /* Filtered potentiometer value in ADC counts */
    uint16_t demag_blanking_counter;        /* Demag blanking time counter */
    uint16_t stall_detection_counter;       /* run time stall counter */
    uint16_t bootstrap_counter;             /* Run time bootstrap counter */
    uint32_t phase_output_map;              /* Map from pwm_sector(0-5) to u/v/w output pattern (off, on or switching) */
    uint16_t compareval[4];                 /* Compare value array */
} motor_var_t;
extern motor_var_t MotorVar;


/*******************************************************************************
 * MotorParam is the structure for motor related running parameters. These parameters
 * are derived from user input parameters.
 ******************************************************************************/
#define MOTOR_PARAM_SIZE        (sizeof(MotorParam))
typedef struct
{
    uint16_t ParamVersion;
    // HwConfig
    uint8_t InternalGain;                   /* 0: gain=1; 1: gain=3; 2: gain=6; 3: gain=12 */
    // SysConfig;
    uint8_t Vadc_Ref_Voltage;               /* VADC reference voltage in 0.1V */
    uint8_t AnalogControl;                  /* 0: Register; 1: Potentiometer */
    uint8_t ControlScheme;                  /* 0: Voltage; 1: Speed; 2: Current; 3: Speed current */
    uint8_t EnableCatchSpin;                /* 1: Enable catch spin */
    uint8_t PwmModulationType;              /* 0: PWM_HIGHSIDE_120_DEG, 1: PWM_LOWSIDE_120_DEG, 2: PWM_HIGHSIDE_60_DEG */
    uint8_t DcBusCompensation;              /* 0: Disable DC bus compensation; 1: Enable DC bus compensation */
    union
    {
        struct
        {
            uint16_t Trap:1;
            uint16_t :1;
            uint16_t :1;
            uint16_t WrongHall:1;
            uint16_t HallLearning:1;
            uint16_t Stall:1;
            uint16_t OverVoltage:1;
            uint16_t :1;
            uint16_t :1;
            uint16_t :1;
            uint16_t UnderVoltage:1;
            uint16_t SpiError:1;
            uint16_t GdReset:1;
        };
        uint16_t Value;
    } EnableFault;
    uint16_t MAX_CURRENT;                   /* Maximum current that can be measured with the power board. */
    uint32_t BASE_SPEED_MECH_RPM;
    uint32_t BASE_VOLTAGE;
    uint32_t BASE_CURRENT;
    uint16_t PWM_FREQUENCY_10HZ;            /* PWM frequency in 10Hz */
    uint8_t  RISING_DEAD_TIME;
    uint8_t  FALLING_DEAD_TIME;
    uint32_t CURRENT_SCALE;                 /*!< Current scale to convert the motor current to target value in Q14 format*/
    uint32_t VOLTAGE_SCALE;                 /*!< Voltage scale to convert the motor voltage to target value in Q14 format*/
    uint16_t SPEED_SCALE;                   /*!< Speed scale to convert the motor speed to target value in Q14 format*/
    uint32_t SPEED_MECH_SCALE;              /*!< Scale to convert the electrical speed to mechanical speed in Q14 format*/
    uint32_t VOLTAGE_ADC_SCALE;             /* convert DC bus ADC counts into nominal DC link voltage in Q14 format */
    uint32_t CURRENT_ADC_SCALE;
    uint32_t RAMP_INITIAL_VALUE;
    uint32_t RAMP_UP_RATE;
    uint32_t RAMP_DOWN_RATE;
    uint32_t RAMP_DOWN_VOLTAGE_LIMIT;
    uint32_t MAX_SPEED;
    uint16_t CURRENT_TRIGGER;
    uint16_t OVER_VOLTAGE_THRESHOLD;
    uint16_t UNDER_VOLTAGE_THRESHOLD;
    uint16_t STALL_DETECTION_TIME;          /* in ms, set 0 to disable stall detection */
    uint16_t STALL_MIN_AMPLITUDE;           /*!< Minimum amplitude below which stall is not detected */
    uint16_t ANALOG_LOW_LIMIT;              /* Q14, Analog value is considered as 0 below this limit. */
    uint16_t BOOTSTRAP_COUNT;
    uint16_t MAX_DUTY_CYCLE;
    uint16_t DEMAG_BLANKING_COUNT;          /* Demagnitization blanking time in number of PWM cycles */
    uint16_t COMMUTATION_DELAY;
    uint16_t PHADV_START_SPEED;             /* Phase advance speed threshold */
    uint16_t PHADV_START_SCALE;             /* Phase advance speed scale factor , Q14 */
    uint16_t PHADV_MAX_ANGLE;               /* Phase advance maximum angle in Q8, 256=1° */
    uint8_t  SPEED_CONTROL_RATE;
    uint8_t  CURRENT_CONTROL_RATE;
    uint16_t SPEED_KP;
    uint16_t SPEED_KI;
    uint16_t SPEED_PI_LIMIT;
    uint16_t CURRENT_KP;
    uint16_t CURRENT_KI;
    uint16_t CURRENT_PI_LIMIT;
    uint16_t VDC_FILTER_GAIN;               /* DC link voltage filter time constant, Tau = 16384/Gain, in Tpwm */
    uint16_t IDC_FILTER_GAIN;               /* DC link current filter time constant, Tau = 16384/Gain, in Tpwm */
    uint16_t HALL_LEARNING_OL_DUTY;
    uint16_t HALL_LEARNING_OL_COUNT;
    /*********************************************************************
     *  +--------------+--------+------+------+------+------+------+------+
     *  |              |   H1 __+--------------------+____________________+--
     *  |  Hall Input  |   H2 ________________+--------------------+_________
     *  |              |   H3 ---------+____________________+----------------
     *  +--------------+--------+------+------+------+------+------+------+
     *  | Hall pattern | H3H2H1 | 101  | 001  | 011  | 010  | 110  | 100  |
     *  |              |  value |  5   |  1   |  3   |  2   |  6   |  4   |
     *  +--------------+--------+------+------+------+------+------+------+
     *  | Sequence = 0 | Sector |  0   |  1   |  2   |  3   |  4   |  5   |
     *  +--------------+--------+------+------+------+------+------+------+
     *  | Sequence = 1 | Sector |  0   |  5   |  4   |  3   |  2   |  1   |
     *  +--------------+--------+------+------+------+------+------+------+
     *
     *    Not conducting: x (0)
     *           High on: + (1)
     *            Low on: - (2)
     *  +--------+-------------+-----------------------+
     *  |        |             |    mc_pattern w/v/u   |
     *  |  Hall  | Rotor angle |-----------+-----------|
     *  | sector |             |  Forward  |  Reverse  |
     *  +--------+-------------+-----------+-----------+
     *  |    0   | 330° -  30° | -+x (210) | +-x (120) |
     *  |    1   |  30° -  90° | x+- (012) | x-+ (021) |
     *  |    2   |  90° - 150° | +x- (102) | -x+ (201) |
     *  |    3   | 150° - 210° | +-x (120) | -+x (210) |
     *  |    4   | 210° - 270° | x-+ (021) | x+- (012) |
     *  |    5   | 270° - 330° | -x+ (201) | +x- (102) |
     *  +--------+-------------+-----------+-----------+
     *
    **********************************************************************/
    uint32_t HALL_SECTOR_MAP;               /* hall pattern --> hall sector (0-5) */
} motor_par_t;
extern motor_par_t MotorParam;

void parameter_set_default(void);
void Motor0_BLDC_SCALAR_GetCommand(void);
void ClearCommand(void);
void Motor0_BLDC_SCALAR_MSM(void);
