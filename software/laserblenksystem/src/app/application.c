#include "app/application.h"

#include "app/motor_control.h"
#include "app/motor_profile.h"
#include "board/motor_board.h"
#include "board/timebase.h"

/* This is the only motor instance started by the current firmware. */
static motor_control_t motor1;

volatile motor_debug_t motor1_debug = {
    .state = MOTOR_INITIALIZING,
    .current_raw_min = UINT16_MAX,
};

void application_initialize(void)
{
    bool clock_ready = timebase_initialize();
    if (!clock_ready)
    {
        /* Without SysTick the I2C deadlines cannot be enforced. */
        motor1_debug.state = MOTOR_TIMING_ERROR;
        return;
    }

    motor_board_initialize();
    motor_control_begin(&motor1, &board_motor1, &motor1_profile, &motor1_debug);
}

void application_service(void)
{
    motor_control_service(&motor1);
}
