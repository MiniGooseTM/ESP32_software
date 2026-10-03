/* TB6612 motor driver + N20 quadrature encoders */
#pragma once

#include <stdint.h>

#define NUM_MOTORS 2

/* Configure PWM, direction pins and encoders. Motors start coasting. */
void motors_init(void);

/* Drive both motors with a command in [-1, 1] each (clamped). Positive drives
   the wheel forward after MOTOR_x_SIGN is applied. Safe to call from any core. */
void motors_set(float a, float b);

/* Both inputs low: driver outputs off, wheels spin freely */
void motors_coast(void);

/* Both inputs high: windings shorted, wheels resist turning */
void motors_brake(void);

/* Accumulated encoder counts, forward positive (ENCODER_x_SIGN applied) */
void encoders_read(int32_t counts[NUM_MOTORS]);
