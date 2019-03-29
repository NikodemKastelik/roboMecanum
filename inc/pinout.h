#ifndef _PINOUT_H_
#define _PINOUT_H_

#define __GLUE3(x, y, z) x ## y ## z
#define _GLUE3(x, y, z)  __GLUE3(x, y, z)

#define __GLUE(x, y) x ## y
#define _GLUE(x, y)  __GLUE(x, y)

#define HAL_RCC_GPIOE  HAL_RCC_GPIO

#define ZIGBEE_PIN_PORT   GPIOD
#define ZIGBEE_PIN_RESET  0
#define ZIGBEE_PIN_KEY    7

#define LED_PIN_PORT    GPIOD
#define LED_PIN_GREEN   12
#define LED_PIN_ORANGE  13
#define LED_PIN_RED     14
#define LED_PIN_BLUE    15

#define VCP_USART_IDX   2
#define VCP_USART       _GLUE(USART, VCP_USART_IDX)
#define VCP_USART_RCC   _GLUE(HAL_RCC_USART, VCP_USART_IDX)
#define VCP_USART_IRQn  _GLUE3(USART, VCP_USART_IDX, _IRQn)
#define VCP_PIN_PORT    GPIOD
#define VCP_PIN_AF      HAL_GPIO_AF_7
#define VCP_PIN_TX      5
#define VCP_PIN_RX      6

#define WALLCLOCK_TIMER_IDX    7
#define WALLCLOCK_TIMER        _GLUE(TIM, WALLCLOCK_TIMER_IDX)
#define WALLCLOCK_TIMER_RCC    _GLUE(HAL_RCC_TIM, WALLCLOCK_TIMER_IDX)
#define WALLCLOCK_TIMER_IRQn   _GLUE3(TIM, WALLCLOCK_TIMER_IDX, _IRQn)
#define WALLCLOCK_TIMER_FREQ   HAL_TIM_FREQ_100kHz
#define WALLCLOCK_TIMER_TICKS  3125
#define wallclock_handler      _GLUE3(TIM, WALLCLOCK_TIMER_IDX, _Handler)

#define MOTOR_FR_DESC   "FR"
#define MOTOR_FL_DESC   "FL"
#define MOTOR_RR_DESC   "RR"
#define MOTOR_RL_DESC   "RL"
#define MOTOR_DESC_LEN  (ARRAY_SIZE(MOTOR_FR_DESC) - 1)

/*
 * @brief Symbols for motor steering PWM.
 *
 * Each PWM occupies one advanced timer. Each timer consists of four channels.
 * To control one motor two channels are needed. One channel controls clockwise
 * rotation while second channel controls counter clockwise rotation.
 */
#define PWM_FRONT_TIMER_IDX  1
#define PWM_FRONT_TIMER      _GLUE(TIM, PWM_FRONT_TIMER_IDX)
#define PWM_FRONT_TIMER_RCC  _GLUE(HAL_RCC_TIM, PWM_FRONT_TIMER_IDX)
#define PWM_FRONT_PIN_PORT   GPIOE
#define PWM_FRONT_PIN_AF     HAL_GPIO_AF_1
#define PWM_FR_PIN_CH_CW     11
#define PWM_FR_PIN_CH_CCW    9
#define PWM_FL_PIN_CH_CW     13
#define PWM_FL_PIN_CH_CCW    14
#define PWM_FR_CH_CW         HAL_TIM_CH2
#define PWM_FR_CH_CCW        HAL_TIM_CH1
#define PWM_FL_CH_CW         HAL_TIM_CH3
#define PWM_FL_CH_CCW        HAL_TIM_CH4

#define PWM_REAR_TIMER_IDX   8
#define PWM_REAR_TIMER       _GLUE(TIM, PWM_REAR_TIMER_IDX)
#define PWM_REAR_TIMER_RCC   _GLUE(HAL_RCC_TIM, PWM_REAR_TIMER_IDX)
#define PWM_REAR_PIN_PORT    GPIOC
#define PWM_REAR_PIN_AF      HAL_GPIO_AF_3
#define PWM_RR_PIN_CH_CW     7
#define PWM_RR_PIN_CH_CCW    6
#define PWM_RL_PIN_CH_CW     8
#define PWM_RL_PIN_CH_CCW    9
#define PWM_RR_CH_CW         HAL_TIM_CH2
#define PWM_RR_CH_CCW        HAL_TIM_CH1
#define PWM_RL_CH_CW         HAL_TIM_CH3
#define PWM_RL_CH_CCW        HAL_TIM_CH4

#define PWM_TICKS  1000

/*
 * @brief Symbols for motor encoder control.
 *
 * Each motor has its own separate incremental encoder. Each encoder instance
 * occupies one timer instance. To read encoder two channels are needed.
 */
#define ENC_FR_TIMER_IDX     2
#define ENC_FR_TIMER         _GLUE(TIM, ENC_FR_TIMER_IDX)
#define ENC_FR_TIMER_RCC     _GLUE(HAL_RCC_TIM, ENC_FR_TIMER_IDX)
#define ENC_FR_PIN_CHA_PORT  GPIOA
#define ENC_FR_PIN_CHB_PORT  GPIOB
#define ENC_FR_PIN_AF        HAL_GPIO_AF_1
#define ENC_FR_PIN_CHA       15
#define ENC_FR_PIN_CHB       3

#define ENC_FL_TIMER_IDX     3
#define ENC_FL_TIMER         _GLUE(TIM, ENC_FL_TIMER_IDX)
#define ENC_FL_TIMER_RCC     _GLUE(HAL_RCC_TIM, ENC_FL_TIMER_IDX)
#define ENC_FL_PIN_PORT      GPIOB
#define ENC_FL_PIN_AF        HAL_GPIO_AF_2
#define ENC_FL_PIN_CHA       4
#define ENC_FL_PIN_CHB       5

#define ENC_RR_TIMER_IDX     4
#define ENC_RR_TIMER         _GLUE(TIM, ENC_RR_TIMER_IDX)
#define ENC_RR_TIMER_RCC     _GLUE(HAL_RCC_TIM, ENC_RR_TIMER_IDX)
#define ENC_RR_PIN_PORT      GPIOB
#define ENC_RR_PIN_AF        HAL_GPIO_AF_2
#define ENC_RR_PIN_CHA       6
#define ENC_RR_PIN_CHB       7

#define ENC_RL_TIMER_IDX     5
#define ENC_RL_TIMER         _GLUE(TIM, ENC_RL_TIMER_IDX)
#define ENC_RL_TIMER_RCC     _GLUE(HAL_RCC_TIM, ENC_RL_TIMER_IDX)
#define ENC_RL_PIN_PORT      GPIOA
#define ENC_RL_PIN_AF        HAL_GPIO_AF_2
#define ENC_RL_PIN_CHA       0
#define ENC_RL_PIN_CHB       1

#define ENC_TICKS_PER_REV    4480

/**
 * @brief Symbols for the stepper motor control.
 */
#define STEPPER_PIN_PORT  GPIOD
#define STEPPER_PIN_IN1   1
#define STEPPER_PIN_IN2   2
#define STEPPER_PIN_IN3   3
#define STEPPER_PIN_IN4   4

#define STEPPER_PIN_IN1_Msk  (1 << STEPPER_PIN_IN1)
#define STEPPER_PIN_IN2_Msk  (1 << STEPPER_PIN_IN2)
#define STEPPER_PIN_IN3_Msk  (1 << STEPPER_PIN_IN3)
#define STEPPER_PIN_IN4_Msk  (1 << STEPPER_PIN_IN4)
#define STEPPER_PINS_Msk     (STEPPER_PIN_IN1_Msk | \
                              STEPPER_PIN_IN2_Msk | \
                              STEPPER_PIN_IN3_Msk | \
                              STEPPER_PIN_IN4_Msk)

#define STEPPER_TIMER_IDX    6
#define STEPPER_TIMER        _GLUE(TIM, STEPPER_TIMER_IDX)
#define STEPPER_TIMER_RCC    _GLUE(HAL_RCC_TIM, STEPPER_TIMER_IDX)
#define STEPPER_TIMER_IRQn   TIM6_DAC_IRQn
#define STEPPER_TIMER_FREQ   HAL_TIM_FREQ_1MHz
#define STEPPER_TIMER_TICKS  900
#define stepper_handler      TIM6_DAC_Handler

#define STEPPER_STEPS_PER_REV  4096

/**
 * @brief Symbols for the distance sensor control.
 */
#define ULTRASONIC_RANGE_CM      400
#define ULTRASONIC_US_PER_CM     58
#define ULTRASONIC_TIMEOUT_US    (ULTRASONIC_RANGE_CM * ULTRASONIC_US_PER_CM)

#define ULTRASONIC_PIN_PORT      GPIOE
#define ULTRASONIC_PIN_TRIG      6
#define ULTRASONIC_PIN_ECHO      5
#define ULTRASONIC_PIN_ECHO_AF   HAL_GPIO_AF_3

#define ULTRASONIC_TIMER_IDX     9
#define ULTRASONIC_TIMER         _GLUE(TIM, ULTRASONIC_TIMER_IDX)
#define ULTRASONIC_TIMER_RCC     _GLUE(HAL_RCC_TIM, ULTRASONIC_TIMER_IDX)

#define ULTRASONIC_TIMER_IRQn    TIM1_BRK_TIM9_IRQn
#define ULTRASONIC_TIMER_FREQ    HAL_TIM_FREQ_1MHz
#define ULTRASONIC_TIMER_TICKS   ULTRASONIC_TIMEOUT_US

#define ultrasonic_handler       TIM1_BRK_TIM9_Handler

#endif // _PINOUT_H_
