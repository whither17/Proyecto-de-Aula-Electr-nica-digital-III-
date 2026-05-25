#ifndef DRIVER_TB6612FNG__H
#define DRIVER_TB6612FNG__H
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <stdlib.h>

void motor_driver_init(void);
void motors_set(int left_speed, int right_speed);

static inline void GoForward(int speed) {
    motors_set(speed, speed);
}
static inline void GoBackwards(int speed) {
    motors_set(-speed, -speed);
}
static inline void SpinLeft(int speed) {
    motors_set(-speed, speed);
}

static inline void SpinRight(int speed) {
    motors_set(speed, -speed);
}

static inline void motors_stop(void) {
    motors_set(0, 0);
}

uint32_t motor_get_ticks(void);

#endif