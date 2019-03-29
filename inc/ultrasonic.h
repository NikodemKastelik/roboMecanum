#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <stdint.h>
#include <stdbool.h>

#define ULTRASONIC_DISTANCE_TIMEOUTED  UINT16_MAX

typedef void (* ultrasonic_data_handler_t)(uint16_t distance_cm);

void ultrasonic_init(ultrasonic_data_handler_t handler);

void ultrasonic_measure(void);

bool ultrasonic_measurement_ongoing_check(void);

#endif // _ULTRASONIC_H_
