#include <ultrasonic.h>
#include <pinout.h>

#include <hal_gpio.h>
#include <hal_tim.h>
#include <hal_dwt.h>

ultrasonic_data_handler_t m_handler;
volatile bool m_ongoing;

static void measurement_finalize(uint16_t distance_cm)
{
    hal_tim_stop(ULTRASONIC_TIMER);
    hal_tim_icp_polarity_set(ULTRASONIC_TIMER, HAL_TIM_CH1, HAL_TIM_ICP_POL_RISING);
    m_ongoing = false;
    m_handler(distance_cm);
}

void ultrasonic_init(ultrasonic_data_handler_t handler)
{
    m_handler = handler;

    hal_gpio_mode_cfg(ULTRASONIC_PIN_PORT, ULTRASONIC_PIN_TRIG, HAL_GPIO_MODE_OUTPUT);

    hal_gpio_mode_cfg(ULTRASONIC_PIN_PORT, ULTRASONIC_PIN_ECHO, HAL_GPIO_MODE_AF);
    hal_gpio_af_cfg(ULTRASONIC_PIN_PORT, ULTRASONIC_PIN_ECHO, ULTRASONIC_PIN_ECHO_AF);

    NVIC_EnableIRQ(ULTRASONIC_TIMER_IRQn);
    hal_tim_timer_cfg(ULTRASONIC_TIMER, ULTRASONIC_TIMER_FREQ, ULTRASONIC_TIMER_TICKS);
    hal_tim_int_enable(ULTRASONIC_TIMER, HAL_TIM_INT_UPDATE | HAL_TIM_INT_CC_CH1);
    hal_tim_icp_cfg(ULTRASONIC_TIMER, HAL_TIM_CH1, HAL_TIM_ICP_POL_RISING);
    hal_tim_slave_cfg(ULTRASONIC_TIMER, HAL_TIM_SLAVE_MODE_RESET, HAL_TIM_SLAVE_TRIG_TI1);
}

void ultrasonic_measure(void)
{
    if (m_ongoing)
    {
        return;
    }
    m_ongoing = true;

    hal_tim_start(ULTRASONIC_TIMER);
    hal_gpio_out_set(ULTRASONIC_PIN_PORT, ULTRASONIC_PIN_TRIG);
    hal_dwt_delay_us(10);
    hal_gpio_out_clr(ULTRASONIC_PIN_PORT, ULTRASONIC_PIN_TRIG);
}

void ultrasonic_handler(void)
{
    if (hal_tim_evt_check(ULTRASONIC_TIMER, HAL_TIM_EVT_CC_CH1))
    {
        hal_tim_icp_pol_t polarity = hal_tim_icp_polarity_get(ULTRASONIC_TIMER, HAL_TIM_CH1);
        if (polarity == HAL_TIM_ICP_POL_RISING)
        {
            hal_tim_icp_polarity_set(ULTRASONIC_TIMER, HAL_TIM_CH1, HAL_TIM_ICP_POL_FALLING);
        }
        else
        {
            uint16_t readout_us = hal_tim_cc_get(ULTRASONIC_TIMER, HAL_TIM_CH1);
            uint16_t distance_cm = readout_us / ULTRASONIC_US_PER_CM;
            measurement_finalize(distance_cm);
        }
    }
    else
    {
        measurement_finalize(ULTRASONIC_DISTANCE_TIMEOUTED);
    }

    hal_tim_evt_clear(ULTRASONIC_TIMER, HAL_TIM_EVT_UPDATE | HAL_TIM_EVT_CC_CH1);
}

bool ultrasonic_measurement_ongoing_check(void)
{
    return m_ongoing;
}
