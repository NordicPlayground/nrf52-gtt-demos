#ifndef __NRF_DUMMY_PWM_H
#define __NRF_DUMMY_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

#define PWM                 NRF_PWM1

// TIMER3 reload value. The PWM frequency equals '16000000 / TIMER_RELOAD'
#define TIMER_RELOAD        1024

typedef struct
{
    uint32_t pin1;
    uint32_t pin2;
    uint32_t pin3;
    uint32_t pin4;
    uint16_t *pwm_buffer;
    uint32_t pwm_buffer_size;
}pwm_init_t;

void pwm_init(pwm_init_t *pwm_config);

void pwm_run(bool run);

#endif
