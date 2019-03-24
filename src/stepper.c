#include <stepper.h>
#include <pinout.h>

#include <hal_common.h>
#include <hal_gpio.h>
#include <hal_tim.h>

typedef enum
{
    STEPPER_DIR_CW  =  1,
    STEPPER_DIR_CCW = -1,
} stepper_dir_t;

static volatile uint32_t m_target_steps;
static volatile stepper_dir_t m_dir;

static const uint16_t stepper_halfstep_seq[] =
{
    STEPPER_PIN_IN1_Msk,
    STEPPER_PIN_IN1_Msk | STEPPER_PIN_IN2_Msk,
    STEPPER_PIN_IN2_Msk,
    STEPPER_PIN_IN2_Msk | STEPPER_PIN_IN3_Msk,
    STEPPER_PIN_IN3_Msk,
    STEPPER_PIN_IN3_Msk | STEPPER_PIN_IN4_Msk,
    STEPPER_PIN_IN4_Msk,
    STEPPER_PIN_IN1_Msk | STEPPER_PIN_IN4_Msk,
};

void stepper_init(void)
{
    hal_gpio_mode_cfg(STEPPER_PIN_PORT, STEPPER_PIN_IN1, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(STEPPER_PIN_PORT, STEPPER_PIN_IN2, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(STEPPER_PIN_PORT, STEPPER_PIN_IN3, HAL_GPIO_MODE_OUTPUT);
    hal_gpio_mode_cfg(STEPPER_PIN_PORT, STEPPER_PIN_IN4, HAL_GPIO_MODE_OUTPUT);

    NVIC_EnableIRQ(STEPPER_TIMER_IRQn);
    hal_tim_timer_cfg(STEPPER_TIMER, STEPPER_TIMER_FREQ, STEPPER_TIMER_TICKS);
    hal_tim_int_enable(STEPPER_TIMER, HAL_TIM_INT_UPDATE);
}

void stepper_goto_steps(int32_t steps)
{
    stepper_dir_t dir = STEPPER_DIR_CW;

    if (steps < 0)
    {
        steps = -steps;
        dir = STEPPER_DIR_CCW;
    }

    m_target_steps = steps;
    m_dir = dir;

    hal_tim_count_clear(STEPPER_TIMER);
    hal_tim_start(STEPPER_TIMER);
}

void stepper_handler(void)
{
    static int8_t step;

    const int8_t seq_size = ARRAY_SIZE(stepper_halfstep_seq);

    hal_tim_evt_clear(STEPPER_TIMER, HAL_TIM_EVT_UPDATE);

    STEPPER_PIN_PORT->ODR &= ~STEPPER_PINS_Msk;

    if (!(m_target_steps--))
    {
        hal_tim_stop(STEPPER_TIMER);
        return;
    }

    STEPPER_PIN_PORT->ODR |= stepper_halfstep_seq[step];
    step += (int8_t)m_dir;

    if (step == seq_size)
    {
        step = 0;
    }
    else if (step == -1)
    {
        step = seq_size - 1;
    }

    hal_gpio_out_toggle(LED_PIN_PORT, LED_PIN_BLUE);
}
