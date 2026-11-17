#ifndef APP_MOTOR_CONTROL_H
#define APP_MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "app/motor_profile.h"
#include "board/motor_board.h"
#include "drivers/drv10975.h"

typedef enum
{
    MOTOR_INITIALIZING,
    MOTOR_RUNNING,
    MOTOR_STOP_REQUESTED,
    MOTOR_I2C_ERROR,
    MOTOR_REGULATOR_MISMATCH,
    MOTOR_CONFIGURATION_ERROR,
    MOTOR_DRIVER_FAULT,
    MOTOR_OVERSPEED,
    MOTOR_WAITING_FOR_DRIVER,
    MOTOR_TIMING_ERROR
} motor_state_t;

typedef struct
{
    stm32_i2c_diagnostic_t transfer;
    uint8_t reg;
    bool speed_high;
} motor_i2c_failure_t;

enum
{
    MOTOR_KT_TRACE_CAPACITY = 64
};

typedef struct
{
    uint32_t elapsed_ms;
    uint32_t rpm;
    uint32_t closed_loop_fg_edges;
    uint16_t kt_raw;
    uint8_t status;
    uint8_t buffered_command;
} motor_kt_sample_t;

typedef struct
{
    uint16_t count;
    uint16_t next_index; /* Oldest sample when full, next free slot otherwise. */
    uint32_t fault_elapsed_ms; /* Fault detection time in ms since the drive command, or zero before a fault. */
    motor_kt_sample_t samples[MOTOR_KT_TRACE_CAPACITY];
} motor_kt_trace_t;

/* The debugger sets the stop and pause flags. Firmware writes the results.
 * Stop acknowledgement means the speed-zero writes succeeded. The current
 * peak is the highest sampled readback, which can miss short spikes. */
typedef struct
{
    bool stop_requested;
    bool pause_after_wake;
    motor_state_t state;
    bool stop_acknowledged;
    uint8_t status;
    uint8_t fault;
    uint8_t initial_status;
    uint8_t initial_fault;
    uint32_t rpm;
    uint32_t closed_loop_fg_edges;
    uint16_t phase_current_raw;
    uint32_t phase_current_ma;
    uint32_t peak_phase_current_ma;
    uint16_t current_raw_at_peak;
    uint16_t current_raw_min;
    uint16_t current_raw_max;
    uint32_t supply_mv;
    uint8_t speed_command;
    uint8_t buffered_command;
    drv10975_config_snapshot_t config;
    uint8_t last_driver_register;
    motor_i2c_failure_t i2c_failure;
    motor_i2c_failure_t stop_i2c_failure;
    uint32_t wake_attempts;
    uint32_t wake_elapsed_ms;
    motor_kt_trace_t kt_trace;
} motor_debug_t;

typedef enum
{
    MOTOR_STAGE_IDLE,
    MOTOR_STAGE_WAKE,
    MOTOR_STAGE_WAKE_RETRY,
    MOTOR_STAGE_CONFIGURE,
    MOTOR_STAGE_WAIT_TO_START,
    MOTOR_STAGE_MONITOR
} motor_stage_t;

typedef struct
{
    motor_board_t *board;
    const motor_profile_t *profile;
    volatile motor_debug_t *debug;
    drv10975_t driver;
    motor_stage_t stage;
    uint32_t wait_started_ms;
    uint32_t wait_ms;
    uint32_t wake_started_ms;
    uint32_t drive_started_ms;
} motor_control_t;

/* Bind one motor to initialized board hardware, a profile, and debug state.
 * A zero-initialized controller stays idle until this function is called. */
void motor_control_begin(motor_control_t *motor, motor_board_t *board,
                         const motor_profile_t *profile, volatile motor_debug_t *debug);

/* Advance one stage or poll once when its deadline is due. Hardware waits
 * inside I2C are bounded. Call repeatedly from the foreground loop. */
void motor_control_service(motor_control_t *motor);

#endif
