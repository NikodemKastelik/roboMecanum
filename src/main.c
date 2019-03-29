#include <hal_common.h>
#include <hal_gpio.h>
#include <hal_rcc.h>
#include <hal_usart.h>
#include <hal_tim.h>
#include <hal_dwt.h>
#include <drv_usart.h>

#include <mini_printf.h>
#include <mini_string.h>
#include <strx_mngr.h>
#include <logger.h>
#include <pid.h>

#include <stepper.h>
#include <ultrasonic.h>

#include <pinout.h>

#define STR_TO_PROCESS_SIZE  32
#define STRX_MNGR_BUFSIZE  128

typedef struct
{
    GPIO_TypeDef * port;
    uint8_t        pin_num;
    uint8_t        af_num;
} robot_pin_desc_t;

const robot_pin_desc_t robot_pins_desc[] =
{
    /* Front-right wheel PWM pins description */
    {PWM_FRONT_PIN_PORT, PWM_FR_PIN_CH_CW,  PWM_FRONT_PIN_AF},
    {PWM_FRONT_PIN_PORT, PWM_FR_PIN_CH_CCW, PWM_FRONT_PIN_AF},

    /* Front-left wheel PWM pins description */
    {PWM_FRONT_PIN_PORT, PWM_FL_PIN_CH_CW,  PWM_FRONT_PIN_AF},
    {PWM_FRONT_PIN_PORT, PWM_FL_PIN_CH_CCW, PWM_FRONT_PIN_AF},

    /* Rear-right wheel PWM pins description */
    {PWM_REAR_PIN_PORT,  PWM_RR_PIN_CH_CW,  PWM_REAR_PIN_AF},
    {PWM_REAR_PIN_PORT,  PWM_RR_PIN_CH_CCW, PWM_REAR_PIN_AF},

    /* Rear-left wheel PWM pins description */
    {PWM_REAR_PIN_PORT,  PWM_RL_PIN_CH_CW,  PWM_REAR_PIN_AF},
    {PWM_REAR_PIN_PORT,  PWM_RL_PIN_CH_CCW, PWM_REAR_PIN_AF},

    /* Front-right wheel Encoder pins description */
    {ENC_FR_PIN_CHA_PORT, ENC_FR_PIN_CHA, ENC_FR_PIN_AF},
    {ENC_FR_PIN_CHB_PORT, ENC_FR_PIN_CHB, ENC_FR_PIN_AF},

    /* Front-left wheel Encoder pins description */
    {ENC_FL_PIN_PORT, ENC_FL_PIN_CHA, ENC_FL_PIN_AF},
    {ENC_FL_PIN_PORT, ENC_FL_PIN_CHB, ENC_FL_PIN_AF},

    /* Rear-right wheel Encoder pins description */
    {ENC_RR_PIN_PORT, ENC_RR_PIN_CHA, ENC_RR_PIN_AF},
    {ENC_RR_PIN_PORT, ENC_RR_PIN_CHB, ENC_RR_PIN_AF},

    /* Rear-left wheel Encoder pins description */
    {ENC_RL_PIN_PORT, ENC_RL_PIN_CHA, ENC_RL_PIN_AF},
    {ENC_RL_PIN_PORT, ENC_RL_PIN_CHB, ENC_RL_PIN_AF},
};

typedef enum
{
    MOTOR_FR_IDX = 0,
    MOTOR_FL_IDX,
    MOTOR_RR_IDX,
    MOTOR_RL_IDX,
    MOTOR_COUNT
} robot_motor_idx_t;

typedef struct
{
    const char *  desc;
    TIM_TypeDef * enc_timer;
    TIM_TypeDef * pwm_timer;
    uint8_t       pwm_ch_cw;
    uint8_t       pwm_ch_ccw;
    bool          inverted;
} robot_motor_cfg_t;

typedef struct
{
    pid_t                     pid;
    volatile int32_t          new_pwm;
    const robot_motor_cfg_t * config;
} robot_motor_t;

const robot_motor_cfg_t robot_motor_cfgs[MOTOR_COUNT] =
{
    {
        .desc       = MOTOR_FR_DESC,
        .enc_timer  = ENC_FR_TIMER,
        .pwm_timer  = PWM_FRONT_TIMER,
        .pwm_ch_cw  = PWM_FR_CH_CW,
        .pwm_ch_ccw = PWM_FR_CH_CCW,
        .inverted   = true,
    },
    {
        .desc       = MOTOR_FL_DESC,
        .enc_timer  = ENC_FL_TIMER,
        .pwm_timer  = PWM_FRONT_TIMER,
        .pwm_ch_cw  = PWM_FL_CH_CW,
        .pwm_ch_ccw = PWM_FL_CH_CCW,
        .inverted   = false,
    },
    {
        .desc       = MOTOR_RR_DESC,
        .enc_timer  = ENC_RR_TIMER,
        .pwm_timer  = PWM_REAR_TIMER,
        .pwm_ch_cw  = PWM_RR_CH_CW,
        .pwm_ch_ccw = PWM_RR_CH_CCW,
        .inverted   = true,
    },
    {
        .desc       = MOTOR_RL_DESC,
        .enc_timer  = ENC_RL_TIMER,
        .pwm_timer  = PWM_REAR_TIMER,
        .pwm_ch_cw  = PWM_RL_CH_CW,
        .pwm_ch_ccw = PWM_RL_CH_CCW,
        .inverted   = false,
    },
};

robot_motor_t robot_motors[MOTOR_COUNT];

char strx_mngr_buffer[STRX_MNGR_BUFSIZE];
strx_mngr_t strx_mngr;

drv_usart_t vcp_usart  = DRV_USART_INSTANCE_GET(2);
uint8_t recv_byte;

volatile bool pid_proced;

uint8_t robot_idx_from_desc_get(char * desc)
{
    for (uint8_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        if (mini_strstartswith(desc, robot_motors[idx].config->desc))
        {
            return idx;
        }
    }
    return UINT8_MAX;
}

void robot_zigbee_init()
{
    hal_gpio_mode_cfg(ZIGBEE_PIN_PORT, ZIGBEE_PIN_RESET, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(ZIGBEE_PIN_PORT, ZIGBEE_PIN_KEY, HAL_GPIO_MODE_OUTPUT);

    hal_gpio_out_clr(ZIGBEE_PIN_PORT, ZIGBEE_PIN_RESET);
    hal_gpio_out_clr(ZIGBEE_PIN_PORT, ZIGBEE_PIN_KEY);
    hal_dwt_delay_us(200000);
    hal_gpio_out_set(ZIGBEE_PIN_PORT, ZIGBEE_PIN_RESET);
    hal_gpio_out_set(ZIGBEE_PIN_PORT, ZIGBEE_PIN_KEY);
}

void robot_set_pwm(robot_motor_t * p_robot_motor)
{
    TIM_TypeDef * pwm_timer = p_robot_motor->config->pwm_timer;
    hal_tim_ch_t pwm_ch_cw = p_robot_motor->config->pwm_ch_cw;
    hal_tim_ch_t pwm_ch_ccw = p_robot_motor->config->pwm_ch_ccw;
    int32_t pwm_signal = p_robot_motor->new_pwm;

    /* Make sure that only one MOSFET out of two will be enabled at time */
    hal_tim_cc_set(pwm_timer, pwm_ch_cw, 0);
    hal_tim_cc_set(pwm_timer, pwm_ch_ccw, 0);
    hal_dwt_delay_us(500);

    if (pwm_signal < 0)
    {
        pwm_signal = -pwm_signal;
        hal_tim_cc_set(pwm_timer, pwm_ch_ccw, pwm_signal);
    }
    else
    {
        hal_tim_cc_set(pwm_timer, pwm_ch_cw, pwm_signal);
    }
}

static void robot_pid_proc(robot_motor_t * p_robot_motor, int16_t enc_latest_read)
{
    pid_t * p_robot_pid = &(p_robot_motor->pid);

    int16_t speed_imp_per_sec = enc_latest_read * (WALLCLOCK_TIMER_FREQ / WALLCLOCK_TIMER_TICKS);

    if (p_robot_motor->config->inverted)
    {
        speed_imp_per_sec =- speed_imp_per_sec;
    }

    int32_t steering_signal = pid_positional_proc(p_robot_pid, speed_imp_per_sec);

    p_robot_motor->new_pwm = steering_signal;
}

void ultrasonic_data_handler(uint16_t distance_cm)
{
    log_msg("Distance=%d=%u\n", stepper_position_get(), distance_cm);
}

void log_out_handler(char * buf, uint32_t len)
{
    drv_usart_tx(&vcp_usart, (uint8_t *)buf, len);
}

void wallclock_handler(void)
{
    int16_t enc_readings[MOTOR_COUNT];

    /* Read encoders as fast as possible */
    for (uint8_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        TIM_TypeDef * enc_timer = robot_motors[idx].config->enc_timer;

        int16_t enc_latest_read = (int16_t)hal_tim_count_get(enc_timer);
        hal_tim_count_clear(enc_timer);

        enc_readings[idx] = enc_latest_read;
    }

    for (uint8_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        robot_pid_proc(&robot_motors[idx], enc_readings[idx]);
    }

    pid_proced = true;

    hal_gpio_out_toggle(LED_PIN_PORT, LED_PIN_ORANGE);

    hal_tim_evt_clear(WALLCLOCK_TIMER, HAL_TIM_EVT_UPDATE);
}

void vcp_usart_handler(drv_usart_evt_t evt, uint8_t const * buf)
{
    switch(evt)
    {
        case DRV_USART_EVT_RXDONE:
            hal_gpio_out_toggle(LED_PIN_PORT, LED_PIN_GREEN);

            uint8_t chr = buf[0];

            drv_usart_rx(&vcp_usart, &recv_byte, 1);

            strx_mngr_feed(&strx_mngr, chr);

            break;

        default:
            break;
    }
}

int main(void)
{
    hal_rcc_enable(HAL_RCC_GPIOA);
    hal_rcc_enable(HAL_RCC_GPIOB);
    hal_rcc_enable(HAL_RCC_GPIOC);
    hal_rcc_enable(HAL_RCC_GPIOD);
    hal_rcc_enable(HAL_RCC_GPIOE);

    hal_rcc_enable(PWM_FRONT_TIMER_RCC);
    hal_rcc_enable(PWM_REAR_TIMER_RCC);
    hal_rcc_enable(ENC_FR_TIMER_RCC);
    hal_rcc_enable(ENC_FL_TIMER_RCC);
    hal_rcc_enable(ENC_RR_TIMER_RCC);
    hal_rcc_enable(ENC_RL_TIMER_RCC);
    hal_rcc_enable(WALLCLOCK_TIMER_RCC);
    hal_rcc_enable(STEPPER_TIMER_RCC);
    hal_rcc_enable(ULTRASONIC_TIMER_RCC);

    log_init(log_out_handler);

    // Kp = 600, Ki = 300, for PWM_Freq = 8 MHz
    const pid_cfg_t pidcfg =
    {
        .kp_q = 600,
        .ki_q = 300,
        .kd_q = 0,
        .q    = 8,
        .output_min = 0,
        .output_max = PWM_TICKS,
    };
    for (uint8_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        robot_motors[idx].config = &robot_motor_cfgs[idx];
        pid_init(&(robot_motors[idx].pid), &pidcfg);
    }

    strx_mngr_init(&strx_mngr, strx_mngr_buffer, STRX_MNGR_BUFSIZE);

    hal_dwt_init();

    hal_gpio_mode_cfg(LED_PIN_PORT, LED_PIN_BLUE,   HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(LED_PIN_PORT, LED_PIN_ORANGE, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(LED_PIN_PORT, LED_PIN_GREEN,  HAL_GPIO_MODE_OUTPUT);

    hal_gpio_mode_cfg(VCP_PIN_PORT, VCP_PIN_TX, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(VCP_PIN_PORT, VCP_PIN_RX, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(VCP_PIN_PORT, VCP_PIN_TX, VCP_PIN_AF);
    hal_gpio_af_cfg(VCP_PIN_PORT, VCP_PIN_RX, VCP_PIN_AF);

    /* Configure robot pins */
    for (uint8_t idx = 0; idx < ARRAY_SIZE(robot_pins_desc); idx++)
    {
        GPIO_TypeDef * port = robot_pins_desc[idx].port;
        uint32_t       pin  = robot_pins_desc[idx].pin_num;
        hal_gpio_af_t  af   = robot_pins_desc[idx].af_num;

        hal_gpio_mode_cfg(port, pin, HAL_GPIO_MODE_AF);
        hal_gpio_af_cfg(port, pin, af);
    }

    for (uint8_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        /* Configure PWM timers */
        TIM_TypeDef * pwm_timer = robot_motor_cfgs[idx].pwm_timer;
        hal_tim_ch_t pwm_ch_cw = robot_motor_cfgs[idx].pwm_ch_cw;
        hal_tim_ch_t pwm_ch_ccw = robot_motor_cfgs[idx].pwm_ch_ccw;

        hal_tim_timer_cfg(pwm_timer, HAL_TIM_FREQ_8MHz, PWM_TICKS);
        hal_tim_pwm_cfg(pwm_timer, pwm_ch_cw, HAL_TIM_PWM_POL_H1L0);
        hal_tim_pwm_cfg(pwm_timer, pwm_ch_ccw, HAL_TIM_PWM_POL_H1L0);
        hal_tim_start(pwm_timer);

        /* Configure ENC timers */
        TIM_TypeDef * enc_timer = robot_motors[idx].config->enc_timer;
        hal_tim_counter_cfg(enc_timer, HAL_TIM_PRESCALER_DISABLED, HAL_TIM_CAPACITY_MAXIMUM);
        hal_tim_encoder_cfg(enc_timer, HAL_TIM_ENC_MODE3, HAL_TIM_ENC_POL_H1L0);
        hal_tim_start(enc_timer);
    }

    NVIC_EnableIRQ(WALLCLOCK_TIMER_IRQn);
    hal_tim_timer_cfg(WALLCLOCK_TIMER, WALLCLOCK_TIMER_FREQ, WALLCLOCK_TIMER_TICKS);
    hal_tim_int_enable(WALLCLOCK_TIMER, HAL_TIM_INT_UPDATE);
    hal_tim_start(WALLCLOCK_TIMER);

    NVIC_EnableIRQ(VCP_USART_IRQn);
    drv_usart_cfg_t cfg = DRV_USART_DEFAULT_CONFIG;
    drv_usart_init(&vcp_usart, &cfg, vcp_usart_handler);
    drv_usart_rx(&vcp_usart, &recv_byte, 1);

    stepper_init();
    robot_zigbee_init();
    ultrasonic_init(ultrasonic_data_handler);

    char string_to_process[STR_TO_PROCESS_SIZE];
    while (1)
    {
        if (pid_proced)
        {
            pid_proced = false;
            for (uint8_t idx = 0; idx < MOTOR_COUNT; idx++)
            {
                robot_set_pwm(&robot_motors[idx]);
            }
            log_msg("Speed=%hd=%hd=%hd=%hd\n", robot_motors[MOTOR_FR_IDX].pid.last_point,
                                               robot_motors[MOTOR_FL_IDX].pid.last_point,
                                               robot_motors[MOTOR_RR_IDX].pid.last_point,
                                               robot_motors[MOTOR_RL_IDX].pid.last_point);
        }

        if (!drv_usart_tx_ongoing_check(&vcp_usart))
        {
            hal_dwt_delay_us(10000);
            log_process();
        }

        if (strx_mngr_retrieve(&strx_mngr, string_to_process))
        {
            if (mini_strstartswith(string_to_process, "sp="))
            {
                char * str_ptr = string_to_process;
                for (uint8_t i = 0; i < MOTOR_COUNT; i++)
                {
                    str_ptr = mini_strchr(str_ptr, '=') + 1;
                    int32_t new_point = mini_atoi(str_ptr);
                    pid_point_set(&robot_motors[i].pid, new_point);
                    log_msg("New point for motor %s: %d\n", robot_motors[i].config->desc, new_point);
                }
            }

            else if (mini_strstartswith(string_to_process, "setpoint_"))
            {
                uint32_t idx = ARRAY_SIZE("setpoint_") - 1;

                uint8_t motor_idx = robot_idx_from_desc_get(&string_to_process[idx]);

                if (motor_idx != UINT8_MAX)
                {
                    idx += MOTOR_DESC_LEN + 1; // desc + '=' sign
                    int32_t new_point = mini_atoi(&string_to_process[idx]);

                    pid_point_set(&robot_motors[motor_idx].pid, new_point);

                    log_msg("New point %s: %d\n", robot_motors[motor_idx].config->desc, new_point);
                }
            }
            else if (mini_strstartswith(string_to_process, "pidkp_"))
            {
                uint32_t idx = ARRAY_SIZE("pidkp_") - 1;

                uint8_t motor_idx = robot_idx_from_desc_get(&string_to_process[idx]);

                if (motor_idx != UINT8_MAX)
                {
                    idx += MOTOR_DESC_LEN + 1; // desc + '=' sign

                    int32_t new_kp = mini_atoi(&string_to_process[idx]);

                    robot_motors[motor_idx].pid.kp_q = new_kp;

                    log_msg("New Kp %s: %d\n", robot_motors[motor_idx].config->desc, new_kp);
                }

            }
            else if (mini_strstartswith(string_to_process, "pidki_"))
            {
                uint32_t idx = ARRAY_SIZE("pidki_") - 1;

                uint8_t motor_idx = robot_idx_from_desc_get(&string_to_process[idx]);

                if (motor_idx != UINT8_MAX)
                {
                    idx += MOTOR_DESC_LEN + 1; // desc + '=' sign

                    int32_t new_ki = mini_atoi(&string_to_process[idx]);

                    robot_motors[motor_idx].pid.ki_q = new_ki;

                    log_msg("New Ki %s: %d\n", robot_motors[motor_idx].config->desc, new_ki);
                }
            }
            else if (mini_strstartswith(string_to_process, "pidkd_"))
            {
                uint32_t idx = ARRAY_SIZE("pidkd_") - 1;

                uint8_t motor_idx = robot_idx_from_desc_get(&string_to_process[idx]);

                if (motor_idx != UINT8_MAX)
                {
                    idx += MOTOR_DESC_LEN + 1; // desc + '=' sign

                    int32_t new_kd = mini_atoi(&string_to_process[idx]);

                    robot_motors[motor_idx].pid.kd_q = new_kd;

                    log_msg("New Kd %s: %d\n", robot_motors[motor_idx].config->desc, new_kd);
                }
            }
            else if (mini_strstartswith(string_to_process, "stepper="))
            {
                const uint32_t idx = ARRAY_SIZE("stepper=") - 1;

                int32_t target_steps = mini_atoi(&string_to_process[idx]);

                stepper_goto_steps(target_steps);

                log_msg("Current steps: %d. Target steps: %d\n", stepper_position_get(), target_steps);
            }

            else if (mini_strstartswith(string_to_process, "dist_measure"))
            {
                ultrasonic_measure();
                log_msg("Distance measurement start at %d steps\n", stepper_position_get());
            }
        }
    }

    return 0;
}

