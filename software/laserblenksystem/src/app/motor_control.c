#include "app/motor_control.h"

#include "board/timebase.h"

enum
{
    WAKE_TIMEOUT_MS = 500,
    WAKE_RETRY_DELAY_MS = 2,
    DRIVER_LOCK_RECOVERY_MS = 5500, /* The driver's recovery takes about 5 s. */
    DRIVER_READY_DELAY_MS = 1000,
    DRIVER_RESTART_SETTLE_MS = 20,
    MOTOR_POLL_DELAY_MS = 20
};

void motor_control_begin(motor_control_t *motor, motor_board_t *board,
                         const motor_profile_t *profile, volatile motor_debug_t *debug)
{
    if (profile->pole_pairs == 0U || profile->speed_command == 0U ||
        profile->speed_command > DRV10975_MAX_SPEED_COMMAND || profile->max_rpm == 0U)
    {
        debug->state = MOTOR_CONFIGURATION_ERROR;
        motor->stage = MOTOR_STAGE_IDLE;
        return;
    }

    motor->board = board;
    motor->profile = profile;
    motor->debug = debug;
    motor->driver.bus = motor_board_i2c(board);
    motor->driver.address = DRV10975_I2C_ADDRESS;
    debug->state = MOTOR_INITIALIZING;
    debug->current_raw_min = UINT16_MAX;
    debug->kt_trace.count = 0;
    debug->kt_trace.next_index = 0;
    debug->kt_trace.fault_elapsed_ms = 0;
    motor->stage = MOTOR_STAGE_WAKE;
}

static void schedule_stage(motor_control_t *motor, motor_stage_t stage, uint32_t delay_ms)
{
    motor->wait_started_ms = timebase_millis();
    motor->wait_ms = delay_ms;
    motor->stage = stage;
}

static bool stage_is_due(const motor_control_t *motor)
{
    /* Unsigned subtraction handles wraparound of the millisecond clock. */
    uint32_t elapsed_ms = timebase_millis() - motor->wait_started_ms;
    return elapsed_ms >= motor->wait_ms;
}

static motor_i2c_failure_t capture_i2c_failure(const motor_control_t *motor)
{
    motor_i2c_failure_t failure;
    failure.transfer = motor->driver.last_transfer;
    failure.reg = motor->driver.last_register;
    failure.speed_high = motor_board_speed_pin_high(motor->board);
    return failure;
}

static void finish_motor(motor_control_t *motor, motor_state_t result)
{
    volatile motor_debug_t *debug = motor->debug;
    debug->last_driver_register = motor->driver.last_register;
    if (motor->driver.last_transfer.error != I2C_ERROR_NONE)
    {
        /* Save the original error before the stop command reuses the bus. */
        debug->i2c_failure = capture_i2c_failure(motor);
    }

    motor_board_set_speed_pin(motor->board, false);
    motor_board_disarm_fg(motor->board);
    debug->closed_loop_fg_edges = motor_board_fg_edges(motor->board);

    bool stopped = drv10975_set_speed(&motor->driver, 0);
    debug->stop_acknowledged = stopped;
    if (!stopped)
    {
        debug->stop_i2c_failure = capture_i2c_failure(motor);
    }
    debug->state = result;
    motor->stage = MOTOR_STAGE_IDLE;
}

static void complete_wake(motor_control_t *motor, bool ready)
{
    bool debugger_attached = motor_board_debugger_attached();
    if (motor->debug->pause_after_wake && debugger_attached)
    {
        /* This bench measurement mode is for disconnected motor phases. */
        motor_board_set_speed_pin(motor->board, true);
        motor->debug->pause_after_wake = false;
        motor_board_debug_break();

        /* The pause can outlast the wake attempt. Retry with SPEED high. */
        stm32_i2c_reset(motor->driver.bus);
        ready = drv10975_take_control(&motor->driver);
    }

    if (!ready)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }

    motor_board_set_speed_pin(motor->board, false);
    motor->stage = MOTOR_STAGE_CONFIGURE;
}

static void begin_wake(motor_control_t *motor)
{
    bool ready = drv10975_take_control(&motor->driver);
    if (ready)
    {
        complete_wake(motor, true);
        return;
    }

    /* An asleep DRV10975Z needs SPEED high before it responds over I2C. */
    motor_board_set_speed_pin(motor->board, true);
    motor->wake_started_ms = timebase_millis();
    schedule_stage(motor, MOTOR_STAGE_WAKE_RETRY, 0);
}

static void retry_wake(motor_control_t *motor)
{
    uint32_t elapsed_ms = timebase_millis() - motor->wake_started_ms;
    if (elapsed_ms >= WAKE_TIMEOUT_MS)
    {
        motor->debug->wake_elapsed_ms = elapsed_ms;
        complete_wake(motor, false);
        return;
    }
    if (!stage_is_due(motor))
    {
        return;
    }

    ++motor->debug->wake_attempts;
    bool ready = drv10975_take_control(&motor->driver);
    if (ready)
    {
        motor->debug->wake_elapsed_ms = timebase_millis() - motor->wake_started_ms;
        complete_wake(motor, true);
        return;
    }
    schedule_stage(motor, MOTOR_STAGE_WAKE_RETRY, WAKE_RETRY_DELAY_MS);
}

static void configure_motor(motor_control_t *motor)
{
    bool regulator_3v3;
    bool read_ok = drv10975_read_regulator_3v3(&motor->driver, &regulator_3v3);
    if (!read_ok)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }
    if (!regulator_3v3)
    {
        finish_motor(motor, MOTOR_REGULATOR_MISMATCH);
        return;
    }

    uint8_t initial_status;
    read_ok = drv10975_read_status(&motor->driver, &initial_status);
    if (!read_ok)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }
    motor->debug->initial_status = initial_status;

    uint8_t initial_fault;
    read_ok = drv10975_read_fault(&motor->driver, &initial_fault);
    if (!read_ok)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }
    motor->debug->initial_fault = initial_fault;
    if ((initial_status & DRV10975_STATUS_THERMAL_OR_OVERCURRENT) != 0U)
    {
        motor->debug->status = initial_status;
        motor->debug->fault = initial_fault;
        finish_motor(motor, MOTOR_DRIVER_FAULT);
        return;
    }

    bool configured = drv10975_configure(&motor->driver, motor->profile->settings,
                                         motor->profile->settings_count, &motor->debug->config);
    if (!configured)
    {
        finish_motor(motor, MOTOR_CONFIGURATION_ERROR);
        return;
    }

    uint32_t delay_ms = DRIVER_READY_DELAY_MS;
    if ((initial_status & DRV10975_STATUS_LOCK) != 0U)
    {
        motor->debug->state = MOTOR_WAITING_FOR_DRIVER;
        delay_ms = DRIVER_LOCK_RECOVERY_MS;
    }
    schedule_stage(motor, MOTOR_STAGE_WAIT_TO_START, delay_ms);
}

static void start_motor(motor_control_t *motor)
{
    if (!stage_is_due(motor))
    {
        return;
    }

    motor_board_arm_fg(motor->board);
    bool command_written = drv10975_set_speed(&motor->driver, motor->profile->speed_command);
    if (!command_written)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }

    motor->drive_started_ms = timebase_millis();
    motor->debug->state = MOTOR_RUNNING;
    /* Allow the retained lock flag to clear before the first status check. */
    schedule_stage(motor, MOTOR_STAGE_MONITOR, DRIVER_RESTART_SETTLE_MS);
}

static void save_telemetry(motor_control_t *motor, const drv10975_telemetry_t *sample)
{
    volatile motor_debug_t *debug = motor->debug;
    /* Convert tenths of an electrical Hz to mechanical RPM. */
    uint32_t electrical_rpm = (uint32_t)sample->speed_deci_electrical_hz * 6U;
    debug->rpm = electrical_rpm / motor->profile->pole_pairs;
    debug->supply_mv = sample->supply_mv;
    debug->speed_command = sample->input_command;
    debug->buffered_command = sample->buffered_command;
    debug->closed_loop_fg_edges = motor_board_fg_edges(motor->board);

    debug->phase_current_raw = sample->phase_current_raw;
    debug->phase_current_ma = sample->phase_current_ma;
    if (sample->phase_current_raw < debug->current_raw_min)
    {
        debug->current_raw_min = sample->phase_current_raw;
    }
    if (sample->phase_current_raw > debug->current_raw_max)
    {
        debug->current_raw_max = sample->phase_current_raw;
    }
    if (sample->phase_current_ma > debug->peak_phase_current_ma)
    {
        debug->peak_phase_current_ma = sample->phase_current_ma;
        debug->current_raw_at_peak = sample->phase_current_raw;
    }
}

static void record_kt_sample(const motor_control_t *motor, uint16_t kt_raw)
{
    volatile motor_debug_t *debug = motor->debug;
    volatile motor_kt_trace_t *trace = &debug->kt_trace;
    motor_kt_sample_t entry = {
        .elapsed_ms = timebase_millis() - motor->drive_started_ms,
        .rpm = debug->rpm,
        .closed_loop_fg_edges = debug->closed_loop_fg_edges,
        .kt_raw = kt_raw,
        .status = debug->status,
        .buffered_command = debug->buffered_command,
    };
    uint16_t index = trace->next_index;
    trace->samples[index] = entry;

    ++index;
    if (index == MOTOR_KT_TRACE_CAPACITY)
    {
        index = 0;
    }
    trace->next_index = index;
    if (trace->count < MOTOR_KT_TRACE_CAPACITY)
    {
        ++trace->count;
    }
}

static void monitor_motor(motor_control_t *motor)
{
    if (!stage_is_due(motor))
    {
        return;
    }

    uint8_t status;
    bool read_ok = drv10975_read_status(&motor->driver, &status);
    if (!read_ok)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }
    motor->debug->status = status;
    if ((status & DRV10975_STATUS_ERROR) != 0U)
    {
        /* Preserve pre-fault samples. The fault may already have cut the drive. */
        motor->debug->kt_trace.fault_elapsed_ms = timebase_millis() - motor->drive_started_ms;
        uint8_t fault;
        bool fault_read_ok = drv10975_read_fault(&motor->driver, &fault);
        if (fault_read_ok)
        {
            motor->debug->fault = fault;
        }
        /* Keep the driver fault result even if its detail byte was unreadable. */
        finish_motor(motor, MOTOR_DRIVER_FAULT);
        return;
    }

    drv10975_telemetry_t sample;
    read_ok = drv10975_read_telemetry(&motor->driver, &sample);
    if (!read_ok)
    {
        finish_motor(motor, MOTOR_I2C_ERROR);
        return;
    }

    save_telemetry(motor, &sample);
    record_kt_sample(motor, sample.kt_raw);
    if (motor->debug->rpm >= motor->profile->max_rpm)
    {
        finish_motor(motor, MOTOR_OVERSPEED);
        return;
    }
    schedule_stage(motor, MOTOR_STAGE_MONITOR, MOTOR_POLL_DELAY_MS);
}

void motor_control_service(motor_control_t *motor)
{
    if (motor->stage == MOTOR_STAGE_IDLE)
    {
        return;
    }
    if (motor->debug->stop_requested)
    {
        finish_motor(motor, MOTOR_STOP_REQUESTED);
        return;
    }

    switch (motor->stage)
    {
        case MOTOR_STAGE_WAKE:
            begin_wake(motor);
            break;
        case MOTOR_STAGE_WAKE_RETRY:
            retry_wake(motor);
            break;
        case MOTOR_STAGE_CONFIGURE:
            configure_motor(motor);
            break;
        case MOTOR_STAGE_WAIT_TO_START:
            start_motor(motor);
            break;
        case MOTOR_STAGE_MONITOR:
            monitor_motor(motor);
            break;
        default:
            finish_motor(motor, MOTOR_CONFIGURATION_ERROR);
            break;
    }
}
