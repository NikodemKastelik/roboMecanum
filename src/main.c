#include <hal_common.h>
#include <hal_gpio.h>
#include <hal_rcc.h>
#include <hal_usart.h>
#include <hal_tim.h>

#include <drv_usart.h>

#include <mini_printf.h>
#include <mini_string.h>
#include <strx_mngr.h>
#include <logger.h>
#include <pid.h>

#include <pinout.h>

#define STR_TO_PROCESS_SIZE  64
#define STRX_MNGR_BUFSIZE  128

pid_t pid_fr;

char strx_mngr_buffer[STRX_MNGR_BUFSIZE];
strx_mngr_t strx_mngr;

drv_usart_t vcp_usart  = DRV_USART_INSTANCE_GET(2);
uint8_t recv_byte;

volatile bool pid_proced;
volatile int32_t new_pwm_signal;

void dwt_init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void dwt_delay_us(uint32_t us)
{
    uint32_t target_tick = DWT->CYCCNT + us * (16000000 / 1000000);
    while (DWT->CYCCNT <= target_tick);
}

void motor_set_pwm(int32_t pwm_signal)
{
    hal_tim_cc_set(PWM_FRONT_TIMER, PWM_FR_CH_CW, 0);
    hal_tim_cc_set(PWM_FRONT_TIMER, PWM_FR_CH_CCW, 0);
    dwt_delay_us(1000);
    if (pwm_signal < 0)
    {
        pwm_signal = -pwm_signal;
        hal_tim_cc_set(PWM_FRONT_TIMER, PWM_FR_CH_CCW, pwm_signal);
    }
    else
    {
        hal_tim_cc_set(PWM_FRONT_TIMER, PWM_FR_CH_CW, pwm_signal);
    }
}

void log_out_handler(char * buf, uint32_t len)
{
    drv_usart_tx(&vcp_usart, (uint8_t *)buf, len);
}

void wallclock_handler(void)
{
    int16_t enc_latest_read = (int16_t)hal_tim_count_get(ENC_FR_TIMER);
    hal_tim_count_clear(ENC_FR_TIMER);

    int16_t speed_imp_per_sec = enc_latest_read * (WALLCLOCK_TIMER_FREQ / WALLCLOCK_TIMER_TICKS);

    int32_t steering_signal = pid_positional_proc(&pid_fr, speed_imp_per_sec);

    new_pwm_signal = steering_signal;
    pid_proced = true;

    log_msg("Speed: %hd\n", speed_imp_per_sec);
    /*
    log_msg("Speed: %hd\n"
            "Signal: %d\n"
            "Integral: %d\n\n",
            speed_imp_per_sec,
            pwm_signal,
            pid_fr.integral_sum);
    */

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

            //log_msg("Usart got byte: 0x%x : %c\n", chr, chr);

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
    hal_rcc_enable(ENC_FR_TIMER_RCC);
    hal_rcc_enable(WALLCLOCK_TIMER_RCC);

    log_init(log_out_handler);

    const pid_cfg_t pidcfg =
    {
        .kp_q = 1000,
        .ki_q = 300,
        .kd_q = 100,
        .q    = 8,
        .output_min = 0,
        .output_max = PWM_TICKS,
    };
    pid_init(&pid_fr, &pidcfg);

    strx_mngr_init(&strx_mngr, strx_mngr_buffer, STRX_MNGR_BUFSIZE);

    dwt_init();

    hal_gpio_mode_cfg(LED_PIN_PORT, LED_PIN_ORANGE, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(LED_PIN_PORT, LED_PIN_GREEN,  HAL_GPIO_MODE_OUTPUT);

    hal_gpio_mode_cfg(VCP_PIN_PORT, VCP_PIN_TX, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(VCP_PIN_PORT, VCP_PIN_RX, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(VCP_PIN_PORT, VCP_PIN_TX, VCP_PIN_AF);
    hal_gpio_af_cfg(VCP_PIN_PORT, VCP_PIN_RX, VCP_PIN_AF);

    hal_gpio_mode_cfg(PWM_FRONT_PIN_PORT, PWM_FR_PIN_CH_CW, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(PWM_FRONT_PIN_PORT, PWM_FR_PIN_CH_CCW, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(PWM_FRONT_PIN_PORT, PWM_FR_PIN_CH_CW, PWM_FRONT_PIN_AF);
    hal_gpio_af_cfg(PWM_FRONT_PIN_PORT, PWM_FR_PIN_CH_CCW, PWM_FRONT_PIN_AF);

    hal_gpio_mode_cfg(ENC_FR_PIN_CHA_PORT, ENC_FR_PIN_CHA, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(ENC_FR_PIN_CHB_PORT, ENC_FR_PIN_CHB, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(ENC_FR_PIN_CHA_PORT, ENC_FR_PIN_CHA, ENC_FR_PIN_AF);
    hal_gpio_af_cfg(ENC_FR_PIN_CHB_PORT, ENC_FR_PIN_CHB, ENC_FR_PIN_AF);

    hal_tim_timer_cfg(PWM_FRONT_TIMER, HAL_TIM_FREQ_16MHz, PWM_TICKS);
    hal_tim_pwm_cfg(PWM_FRONT_TIMER, PWM_FR_CH_CW, HAL_TIM_PWM_POL_H1L0);
    hal_tim_pwm_cfg(PWM_FRONT_TIMER, PWM_FR_CH_CCW, HAL_TIM_PWM_POL_H1L0);
    hal_tim_start(PWM_FRONT_TIMER);

    hal_tim_counter_cfg(ENC_FR_TIMER, HAL_TIM_PRESCALER_DISABLED, HAL_TIM_CAPACITY_MAXIMUM);
    hal_tim_encoder_cfg(ENC_FR_TIMER, HAL_TIM_ENC_MODE3, HAL_TIM_ENC_POL_H1L0);
    hal_tim_start(ENC_FR_TIMER);

    NVIC_EnableIRQ(WALLCLOCK_TIMER_IRQn);
    hal_tim_timer_cfg(WALLCLOCK_TIMER, WALLCLOCK_TIMER_FREQ, WALLCLOCK_TIMER_TICKS);
    hal_tim_int_enable(WALLCLOCK_TIMER, HAL_TIM_INT_UPDATE);
    hal_tim_start(WALLCLOCK_TIMER);

    NVIC_EnableIRQ(VCP_USART_IRQn);
    drv_usart_cfg_t cfg = DRV_USART_DEFAULT_CONFIG;
    drv_usart_init(&vcp_usart, &cfg, vcp_usart_handler);
    drv_usart_rx(&vcp_usart, &recv_byte, 1);

    __enable_irq();

    char string_to_process[STR_TO_PROCESS_SIZE];
    while (1)
    {

        if (pid_proced)
        {
            pid_proced = false;
            motor_set_pwm(new_pwm_signal);
        }

        if (!drv_usart_tx_ongoing_check(&vcp_usart))
        {
            log_process();
        }

        if (strx_mngr_retrieve(&strx_mngr, string_to_process))
        {
            if (mini_strstartswith(string_to_process, "setpoint="))
            {
                const uint32_t idx = ARRAY_SIZE("setpoint=") - 1;
                int32_t new_point = mini_atoi(&string_to_process[idx]);

                log_msg("New point: %d\n", new_point);

                pid_point_set(&pid_fr, new_point);
            }
            else if (mini_strstartswith(string_to_process, "pidkp="))
            {
                const uint32_t idx = ARRAY_SIZE("pidkp=") - 1;
                int32_t new_kp = mini_atoi(&string_to_process[idx]);

                log_msg("New Kp: %d\n", new_kp);

                pid_fr.kp_q = new_kp;
            }
            else if (mini_strstartswith(string_to_process, "pidki="))
            {
                const uint32_t idx = ARRAY_SIZE("pidki=") - 1;
                int32_t new_ki = mini_atoi(&string_to_process[idx]);

                log_msg("New Ki: %d\n", new_ki);

                pid_fr.ki_q = new_ki;
            }
            else if (mini_strstartswith(string_to_process, "pidkd="))
            {
                const uint32_t idx = ARRAY_SIZE("pidkd=") - 1;
                int32_t new_kd = mini_atoi(&string_to_process[idx]);

                log_msg("New Kd: %d\n", new_kd);

                pid_fr.kd_q = new_kd;
            }
        }
    }

    return 0;
}

