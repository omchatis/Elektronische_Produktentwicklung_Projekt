#ifndef APP_MOTOR_PROFILE_H
#define APP_MOTOR_PROFILE_H

#include <stddef.h>
#include <stdint.h>

#include "drivers/drv10975.h"

typedef struct
{
    const drv10975_setting_t *settings;
    size_t settings_count;
    uint16_t speed_command;
    uint16_t max_rpm;
    uint8_t pole_pairs;
} motor_profile_t;

extern const motor_profile_t motor1_profile;

#endif
