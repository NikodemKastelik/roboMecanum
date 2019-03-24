#include <stepper.h>
#include <pinout.h>

#include <hal_common.h>
#include <hal_gpio.h>
#include <hal_tim.h>

static volatile int32_t m_current_steps;
static volatile int32_t m_target_steps;

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
    hal_tim_start(STEPPER_TIMER);
}

void stepper_goto_steps(int32_t steps)
{
    m_target_steps = steps;
}

int32_t stepper_position_get(void)
{
    return m_current_steps;
}

void stepper_handler(void)
{
    static int8_t seq_step;

    const int8_t seq_size = ARRAY_SIZE(stepper_halfstep_seq);

    hal_tim_evt_clear(STEPPER_TIMER, HAL_TIM_EVT_UPDATE);

    STEPPER_PIN_PORT->ODR &= ~STEPPER_PINS_Msk;

    int32_t curr_steps = m_current_steps;
    if (curr_steps == m_target_steps)
    {
        return;
    }

    STEPPER_PIN_PORT->ODR |= stepper_halfstep_seq[seq_step];

    if (m_target_steps > curr_steps)
    {
        seq_step++;
        curr_steps++;
    }
    else
    {
        seq_step--;
        curr_steps--;
    }
    m_current_steps = curr_steps;

    if (seq_step == seq_size)
    {
        seq_step = 0;
    }
    else if (seq_step == -1)
    {
        seq_step = seq_size - 1;
    }

    hal_gpio_out_toggle(LED_PIN_PORT, LED_PIN_BLUE);
}
