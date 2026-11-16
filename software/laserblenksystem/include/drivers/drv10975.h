#ifndef DRIVERS_DRV10975_H
#define DRIVERS_DRV10975_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/stm32_i2c.h"

enum
{
    DRV10975_I2C_ADDRESS = 0x52,
    DRV10975_MAX_SPEED_COMMAND = 511,

    /* Configuration registers used in motor profiles. */
    DRV10975_MOTOR_RESISTANCE = 0x20,
    DRV10975_MOTOR_BEMF = 0x21,
    DRV10975_COMMUTATION_ADVANCE = 0x22,
    DRV10975_STARTUP_OPTIONS = 0x23,
    DRV10975_STARTUP_CURRENT = 0x24,
    DRV10975_STARTUP_ACCELERATION = 0x25,
    DRV10975_ALIGNMENT = 0x26,
    DRV10975_PROTECTION = 0x27,
    DRV10975_CURRENT_LIMITS = 0x28,
    DRV10975_CLOSED_LOOP_RAMP = 0x29,
    DRV10975_REGULATOR = 0x2A,
    DRV10975_SPEED_MODE = 0x2B,
    DRV10975_CONFIG_COUNT = DRV10975_SPEED_MODE - DRV10975_MOTOR_RESISTANCE + 1,

    DRV10975_INITIAL_SPEED_DETECT = 1U << 3, /* SysOpt1 ISDen. */
    DRV10975_STATUS_LOCK = 1U << 4,
    DRV10975_STATUS_THERMAL_OR_OVERCURRENT = (1U << 7) | (1U << 5),
    DRV10975_STATUS_ERROR = 0xF0 /* Overtemperature, sleep or standby, overcurrent, and lock. */
};

typedef struct
{
    stm32_i2c_bus_t *bus;
    uint8_t address; /* 7-bit I2C address. Devices can share it on separate buses. */
    uint8_t last_register;
    /* Keep this device's result even if another target uses the bus next. */
    stm32_i2c_diagnostic_t last_transfer;
} drv10975_t;

typedef struct
{
    uint8_t address;
    uint8_t mask;
    uint8_t value;
} drv10975_setting_t;

typedef struct
{
    /* Index i corresponds to register 0x20 + i. */
    uint8_t before[DRV10975_CONFIG_COUNT];
    uint8_t after[DRV10975_CONFIG_COUNT];
} drv10975_config_snapshot_t;

typedef struct
{
    uint16_t speed_deci_electrical_hz;
    uint16_t phase_current_raw;
    uint16_t kt_raw; /* MotorKt1/2, scaled to V/Hz by dividing by 2884. */
    uint32_t phase_current_ma;
    uint32_t supply_mv;
    uint8_t input_command; /* 0 to 255 readback scale. */
    uint8_t buffered_command;
} drv10975_telemetry_t;

/* Take I2C control of an awake device and send a zero speed command. */
bool drv10975_take_control(drv10975_t *driver);

/* Apply a 0 to 511 command with I2C override. Zero requests a stop. */
bool drv10975_set_speed(drv10975_t *driver, uint16_t command);

/* Read board compatibility, status, and the detailed fault byte. */
bool drv10975_read_regulator_3v3(drv10975_t *driver, bool *is_3v3);
bool drv10975_read_status(drv10975_t *driver, uint8_t *status);
bool drv10975_read_fault(drv10975_t *driver, uint8_t *fault);

/* Apply and verify volatile registers. The caller sets the drive command
 * to zero first. The before and after snapshots can be partial on failure. */
bool drv10975_configure(drv10975_t *driver, const drv10975_setting_t *settings,
                        size_t count, volatile drv10975_config_snapshot_t *snapshot);

/* Read a complete telemetry sample, keeping the speed in electrical units. */
bool drv10975_read_telemetry(drv10975_t *driver, drv10975_telemetry_t *telemetry);

#endif
