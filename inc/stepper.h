#ifndef _STEPPER_H_
#define _STEPPER_H_

#include <stdint.h>

void stepper_init(void);

void stepper_goto_steps(int32_t steps);

int32_t stepper_position_get(void);

#endif // _STEPPER_H_
