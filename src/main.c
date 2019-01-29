#include <hal_common.h>
#include <hal_gpio.h>
#include <hal_rcc.h>
#include <hal_usart.h>
#include <hal_tim.h>

#include <drv_usart.h>

#include <pid.h>
#include <mini_printf.h>
#include <logger.h>
#include <moving_average.h>

#define LED_ORANGE_PIN  13
#define LED_GREEN_PIN   12
#define LED_PIN_PORT    GPIOD
#define LED_PIN_RCC     HAL_RCC_GPIOD

#define VCP_USART       USART2
#define VCP_USART_RCC   HAL_RCC_USART2
#define VCP_USART_IRQn  USART2_IRQn
#define VCP_PINS_PORT   GPIOA
#define VCP_PINS_RCC    HAL_RCC_GPIOA
#define VCP_AF_NUM      HAL_GPIO_AF_7
#define VCP_TX_PIN      2
#define VCP_RX_PIN      3

#define PWM_TIMER       TIM3
#define PWM_TIMER_RCC   HAL_RCC_TIM3
#define PWM_PINS_PORT   GPIOC
#define PWM_PINS_RCC    HAL_RCC_GPIOC
#define PWM_AF_NUM      HAL_GPIO_AF_2
#define PWM_CH1_PIN     6
#define PWM_CH2_PIN     7
#define PWM_CH3_PIN     8
#define PWM_CH4_PIN     9
#define PWM_TICKS       1000

#define ENC1_TIMER      TIM4
#define ENC1_TIMER_RCC  HAL_RCC_TIM4
#define ENC1_TIMER_IRQn TIM4_IRQn
#define ENC1_PINS_PORT  GPIOB
#define ENC1_PINS_RCC   HAL_RCC_GPIOB
#define ENC1_CH_A_PIN   6
#define ENC1_CH_B_PIN   7
#define ENC1_AF_NUM     HAL_GPIO_AF_2
#define enc1_handler    TIM4_Handler

#define ENC_TICKS_FOR_SPEED_UPDATE  12
#define ENC_TICKS_PER_REV       4480
#define ENC_TICKS_PER_DEGREE    (ENC_TICKS_PER_REV / 360)
#define ENC_TICKS_TO_DEGREE(x)  ((x) / ENC_TICKS_PER_DEGREE)

#define SPEED_SENSE_FREQ        HAL_TIM_FREQ_2MHz

#define HAL_RCC_GPIOE           HAL_RCC_GPIO
#define SPEED_SENSE1_TIMER      TIM9
#define SPEED_SENSE1_TIMER_RCC  HAL_RCC_TIM9
#define SPEED_SENSE1_TIMER_IRQn TIM1_BRK_TIM9_IRQn
#define SPEED_SENSE1_PINS_PORT  GPIOE
#define SPEED_SENSE1_PINS_RCC   HAL_RCC_GPIOE
#define SPEED_SENSE1_AF_NUM     HAL_GPIO_AF_3
#define SPEED_SENSE1_CH1_PIN    5
#define SPEED_SENSE1_CH2_PIN    6
#define speed_sense1_handler    TIM1_BRK_TIM9_Handler

#define WALLCLOCK_TIMER        TIM7
#define WALLCLOCK_TIMER_RCC    HAL_RCC_TIM7
#define WALLCLOCK_TIMER_IRQn   TIM7_IRQn
#define WALLCLOCK_TIMER_FREQ   HAL_TIM_FREQ_100kHz
#define WALLCLOCK_TIMER_TICKS  3125
#define wallclock_handler      TIM7_Handler

/*
#define SENSORLOOP_TIMER       TIM6
#define SENSORLOOP_TIMER_RCC   HAL_RCC_TIM6
#define SENSORLOOP_TIMER_IRQn  TIM6_DAC_IRQn
#define SENSORLOOP_TIMER_FREQ  HAL_TIM_FREQ_10kHz
#define SENSORLOOP_TIMER_TICKS 5
#define sensorloop_handler     TIM6_DAC_Handler
*/

#define BUTTON_PORT  GPIOA
#define BUTTON_PIN   0

#define MOTOR_DIR_PINS_PORT  GPIOC
#define MOTOR_DIR_PINS_RCC   HAL_RCC_GPIOC
#define MOTOR_DIR_CW_PIN     14
#define MOTOR_DIR_CCW_PIN    15

pid_t pid1;

drv_usart_t usart2  = DRV_USART_INSTANCE_GET(2);
uint8_t recv_buf[2];
uint8_t send_buf[1];

//uint16_t enc1_imp_time[8];
//moving_avg_t movavg;

//volatile int16_t current_speed;
volatile int32_t absolute_pos;

void log_out_handler(char * buf, uint32_t len)
{
    drv_usart_tx(&usart2, (uint8_t *)buf, len);
}

/*
void sensorloop_handler(void)
{
    current_speed = 0;
    hal_tim_evt_clear(SENSORLOOP_TIMER, HAL_TIM_EVT_UPDATE);
    //static uint16_t prev_diff;

    uint16_t icp1;
    uint16_t icp2;

    if (hal_tim_evt_check(SPEED_SENSE1_TIMER, HAL_TIM_EVT_CC_CH1)
        && hal_tim_evt_check(SPEED_SENSE1_TIMER, HAL_TIM_EVT_CC_CH2))
    {
        icp1 = hal_tim_cc_get(SPEED_SENSE1_TIMER, HAL_TIM_CH1);
        icp2 = hal_tim_cc_get(SPEED_SENSE1_TIMER, HAL_TIM_CH2);

//        log_msg("Got ICP1: %u ICP2: %u\n\n", icp1, icp2);
    }
    else
    {
        icp1 = 0;
        icp2 = 0;
    }

    uint16_t diff = icp1 - icp2;
    if (diff > (UINT16_MAX / 2))
    {
        diff = icp2 - icp1;
    }
    
    moving_avg_add(&movavg, diff);
    hal_tim_evt_clear(SENSORLOOP_TIMER, HAL_TIM_EVT_UPDATE);
}
*/

/*
void enc1_handler(void)
{
    uint32_t sensortimer_ticks = hal_tim_count_get(SPEED_SENSE1_TIMER);
    hal_tim_count_clear(SPEED_SENSE1_TIMER);
    hal_tim_count_clear(ENC1_TIMER);

    int16_t speed = (int16_t)((SPEED_SENSE_FREQ * ENC_TICKS_FOR_SPEED_UPDATE) / sensortimer_ticks);
    int32_t pos_inc = ENC_TICKS_FOR_SPEED_UPDATE;

    uint32_t evtmask = hal_tim_evt_mask_get(ENC1_TIMER);
    if (evtmask & HAL_TIM_EVT_CC_CH4)
    {
        speed *= -1;
        pos_inc *= -1;
    }

    absolute_pos += pos_inc;
    current_speed = speed;
    hal_tim_evt_clear(ENC1_TIMER, evtmask);

    //log_msg("Enc interrupt. Ticks: %hu\n, Speed: %hd\n\n", sensortimer_ticks, speed);
}
*/

/*
void speed_sense1_handler(void)
{
    //If this timer overflowed it means motor does not rotate
    current_speed= 0;
    hal_tim_evt_clear(SPEED_SENSE1_TIMER, HAL_TIM_EVT_UPDATE);
}
*/

void wallclock_handler(void)
{
    int16_t enc_latest_read = (int16_t)hal_tim_count_get(ENC1_TIMER);
    hal_tim_count_clear(ENC1_TIMER);

    absolute_pos += enc_latest_read;

    int16_t speed_imp_per_sec = enc_latest_read * (WALLCLOCK_TIMER_FREQ / WALLCLOCK_TIMER_TICKS);

    int32_t steering_signal = pid_positional_proc(&pid1, speed_imp_per_sec);

    int32_t pwm_signal = steering_signal;
    if (pwm_signal < 0)
    {
        pwm_signal = -pwm_signal;
        hal_gpio_out_set(MOTOR_DIR_PINS_PORT, MOTOR_DIR_CW_PIN);
        hal_gpio_out_clr(MOTOR_DIR_PINS_PORT, MOTOR_DIR_CCW_PIN);
    }
    else
    {
        hal_gpio_out_clr(MOTOR_DIR_PINS_PORT, MOTOR_DIR_CW_PIN);
        hal_gpio_out_set(MOTOR_DIR_PINS_PORT, MOTOR_DIR_CCW_PIN);
    }

    hal_tim_cc_set(PWM_TIMER, HAL_TIM_CH1, pwm_signal);

    if (hal_gpio_input_check(BUTTON_PORT, BUTTON_PIN))
    {
        hal_gpio_out_toggle(LED_PIN_PORT, LED_ORANGE_PIN);
    }

    log_msg("Speed: %hd\n"
            "Signal: %d\n\n",
            speed_imp_per_sec,
            pwm_signal);

    hal_tim_evt_clear(WALLCLOCK_TIMER, HAL_TIM_EVT_UPDATE);
}


void vcp_usart_handler(drv_usart_evt_t evt, uint8_t const * buf)
{
    switch(evt)
    {
        case DRV_USART_EVT_RXDONE:
            hal_gpio_out_toggle(LED_PIN_PORT, LED_GREEN_PIN);

            uint8_t * next_buf;

            if ((uintptr_t)buf == (uintptr_t)&recv_buf[0])
            {
                next_buf = &recv_buf[1];
            }
            else
            {
                next_buf = &recv_buf[0];
            }
            drv_usart_rx(&usart2, next_buf, 1);

            log_msg("Usart got byte: %c\n", buf[0]);
            break;

        default:
            break;
    }
}

int main(void)
{
    hal_rcc_enable(VCP_PINS_RCC);
    hal_rcc_enable(LED_PIN_RCC);
    hal_rcc_enable(PWM_PINS_RCC);
    hal_rcc_enable(ENC1_PINS_RCC);
//    hal_rcc_enable(SPEED_SENSE1_PINS_RCC);

    hal_rcc_enable(PWM_TIMER_RCC);
    hal_rcc_enable(ENC1_TIMER_RCC);
//    hal_rcc_enable(SPEED_SENSE1_TIMER_RCC);
    hal_rcc_enable(WALLCLOCK_TIMER_RCC);
//    hal_rcc_enable(SENSORLOOP_TIMER_RCC);

    log_init(log_out_handler);

    const pid_cfg_t pidcfg =
    {
        .kp_q = 30,
        .ki_q = 6,
        .kd_q = 0,
        .q    = 8,
        .output_min = PWM_TICKS / 2,
        .output_max = PWM_TICKS,
    };
    pid_init(&pid1, &pidcfg);
    pid_point_set(&pid1, ENC_TICKS_PER_REV / 2);

    //moving_avg_init(&movavg, enc1_imp_time, ARRAY_SIZE(enc1_imp_time));

    hal_gpio_mode_cfg(BUTTON_PORT, BUTTON_PIN, HAL_GPIO_MODE_INPUT);

    hal_gpio_mode_cfg(LED_PIN_PORT, LED_ORANGE_PIN, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(LED_PIN_PORT, LED_GREEN_PIN,  HAL_GPIO_MODE_OUTPUT);

    hal_gpio_mode_cfg(MOTOR_DIR_PINS_PORT, MOTOR_DIR_CW_PIN,  HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(MOTOR_DIR_PINS_PORT, MOTOR_DIR_CCW_PIN, HAL_GPIO_MODE_OUTPUT);

    hal_gpio_mode_cfg(VCP_PINS_PORT, VCP_TX_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(VCP_PINS_PORT, VCP_RX_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(VCP_PINS_PORT,  VCP_TX_PIN,    VCP_AF_NUM);
    hal_gpio_af_cfg(VCP_PINS_PORT,  VCP_RX_PIN,    VCP_AF_NUM);

    hal_gpio_mode_cfg(PWM_PINS_PORT, PWM_CH1_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(PWM_PINS_PORT,  PWM_CH1_PIN,   PWM_AF_NUM);

    hal_gpio_mode_cfg(ENC1_PINS_PORT, ENC1_CH_A_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(ENC1_PINS_PORT, ENC1_CH_B_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(ENC1_PINS_PORT, ENC1_CH_A_PIN, ENC1_AF_NUM);
    hal_gpio_af_cfg(ENC1_PINS_PORT, ENC1_CH_B_PIN, ENC1_AF_NUM);

    /*
    hal_gpio_mode_cfg(SPEED_SENSE1_PINS_PORT, SPEED_SENSE1_CH1_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_mode_cfg(SPEED_SENSE1_PINS_PORT, SPEED_SENSE1_CH2_PIN, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(SPEED_SENSE1_PINS_PORT, SPEED_SENSE1_CH1_PIN, SPEED_SENSE1_AF_NUM);
    hal_gpio_af_cfg(SPEED_SENSE1_PINS_PORT, SPEED_SENSE1_CH2_PIN, SPEED_SENSE1_AF_NUM);
    */

    hal_tim_timer_cfg(PWM_TIMER, HAL_TIM_FREQ_16MHz, PWM_TICKS);
    hal_tim_pwm_cfg(PWM_TIMER, HAL_TIM_CH1, HAL_TIM_PWM_POL_H1L0);
    hal_tim_start(PWM_TIMER);

//    NVIC_EnableIRQ(ENC1_TIMER_IRQn);
    hal_tim_counter_cfg(ENC1_TIMER, HAL_TIM_PRESCALER_DISABLED, HAL_TIM_CAPACITY_MAXIMUM);
    hal_tim_encoder_cfg(ENC1_TIMER, HAL_TIM_ENC_MODE3, HAL_TIM_ENC_POL_H1L0);
/*
    hal_tim_cc_set(ENC1_TIMER, HAL_TIM_CH3, (uint16_t)ENC_TICKS_FOR_SPEED_UPDATE);
    hal_tim_cc_set(ENC1_TIMER, HAL_TIM_CH4, (uint16_t)(-ENC_TICKS_FOR_SPEED_UPDATE));
    hal_tim_int_enable(ENC1_TIMER, HAL_TIM_INT_CC_CH3 | HAL_TIM_INT_CC_CH4);
*/
    hal_tim_start(ENC1_TIMER);

/*
    NVIC_EnableIRQ(SPEED_SENSE1_TIMER_IRQn);
    hal_tim_timer_cfg(SPEED_SENSE1_TIMER, SPEED_SENSE_FREQ, HAL_TIM_CAPACITY_MAXIMUM);
    hal_tim_int_enable(SPEED_SENSE1_TIMER, HAL_TIM_INT_UPDATE);
    //hal_tim_icp_cfg(SPEED_SENSE1_TIMER, HAL_TIM_CH1, HAL_TIM_ICP_POL_H1L0);
    //hal_tim_icp_cfg(SPEED_SENSE1_TIMER, HAL_TIM_CH2, HAL_TIM_ICP_POL_H0L1);
    hal_tim_start(SPEED_SENSE1_TIMER);
*/

    NVIC_EnableIRQ(WALLCLOCK_TIMER_IRQn);
    hal_tim_timer_cfg(WALLCLOCK_TIMER, WALLCLOCK_TIMER_FREQ, WALLCLOCK_TIMER_TICKS);
    hal_tim_int_enable(WALLCLOCK_TIMER, HAL_TIM_INT_UPDATE);
    hal_tim_start(WALLCLOCK_TIMER);

    NVIC_EnableIRQ(VCP_USART_IRQn);
    drv_usart_cfg_t cfg = DRV_USART_DEFAULT_CONFIG;
    drv_usart_init(&usart2, &cfg, vcp_usart_handler);
    drv_usart_rx(&usart2, &recv_buf[0], 1);
    drv_usart_rx(&usart2, &recv_buf[1], 1);

    /*
    NVIC_EnableIRQ(SENSORLOOP_TIMER_IRQn);
    hal_tim_timer_cfg(SENSORLOOP_TIMER, SENSORLOOP_TIMER_FREQ, SENSORLOOP_TIMER_TICKS);
    hal_tim_int_enable(SENSORLOOP_TIMER, HAL_TIM_INT_UPDATE);
    hal_tim_start(SENSORLOOP_TIMER);
    */

    __enable_irq();

    while (1)
    {
        if (!drv_usart_tx_ongoing_check(&usart2))
        {
            log_process();
        }
    }

    return 0;
}

