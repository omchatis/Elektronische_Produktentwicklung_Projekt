#ifndef DRIVERS_DRV10975_H
#define DRIVERS_DRV10975_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/stm32_i2c.h"

enum
{
    DRV10975_I2C_ADDRESS = 0x52,
    DRV10975_MAX_SPEED_COMMAND = 511,

    DRV10975_CONFIG_COUNT = 12, /* MotorParam1 (0x20) through SysOpt9 (0x2B). */
    DRV10975_KEEP = UINT8_MAX, /* Keep the value currently in the driver. */

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

/* Field names and groups follow the datasheet register map. Set each field
 * explicitly to an unshifted binary code or DRV10975_KEEP. Zero is a code. */
typedef struct
{
    struct
    {
        uint8_t DoubleFreq;
        uint8_t Rm;
    } MotorParam1;

    struct
    {
        uint8_t AdjMode;
        uint8_t Kt;
    } MotorParam2;

    struct
    {
        uint8_t CtrlAdvMd;
        uint8_t TCtrlAdv;
    } MotorParam3;

    struct
    {
        uint8_t ISDThr;
        uint8_t IPDAdvcAgl;
        uint8_t ISDen;
        uint8_t RvsDrEn;
        uint8_t RvsDrThr;
    } SysOpt1;

    struct
    {
        uint8_t OpenLCurr;
        uint8_t OpLCurrRt;
        uint8_t BrkDoneThr;
    } SysOpt2;

    struct
    {
        uint8_t CtrlCoef;
        uint8_t StAccel2;
        uint8_t StAccel;
    } SysOpt3;

    struct
    {
        uint8_t Op2ClsThr;
        uint8_t AlignTime;
    } SysOpt4;

    struct
    {
        uint8_t LockEn3;
        uint8_t LockEn2;
        uint8_t LockEn1;
        uint8_t LockEn0;
        uint8_t AVSIndEn;
        uint8_t AVSMEn;
        uint8_t AVSMMd;
        uint8_t IPDRlsMd;
    } SysOpt5;

    struct
    {
        uint8_t SWiLimitThr;
        uint8_t HWiLimitThr;
    } SysOpt6;

    struct
    {
        uint8_t LockEn5;
        uint8_t ClsLpAccel;
        uint8_t Deadtime;
    } SysOpt7;

    struct
    {
        uint8_t IPDCurrThr;
        uint8_t LockEn4;
        uint8_t VregSel;
        uint8_t IPDClk;
    } SysOpt8;

    struct
    {
        uint8_t FGOLsel;
        uint8_t FGcycle;
        uint8_t KtLckThr;
        uint8_t SpdCtrlMd;
        uint8_t CLoopDis;
    } SysOpt9;
} drv10975_registers_t;

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

/* Apply explicit fields and verify the resulting volatile registers. The caller
 * sets the drive command to zero first. The snapshots can be partial on failure. */
bool drv10975_configure(drv10975_t *driver, const drv10975_registers_t *registers,
                        volatile drv10975_config_snapshot_t *snapshot);

/* Read a complete telemetry sample, keeping the speed in electrical units. */
bool drv10975_read_telemetry(drv10975_t *driver, drv10975_telemetry_t *telemetry);

#endif
