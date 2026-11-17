#ifndef APP_MOTOR_PROFILE_H
#define APP_MOTOR_PROFILE_H

#include <stdint.h>

#include "drivers/drv10975.h"

typedef struct
{
    drv10975_registers_t registers;
    uint16_t speed_command;
    uint16_t max_rpm;
    uint8_t pole_pairs;
} motor_profile_t;

extern const motor_profile_t motor1_profile;

#endif
