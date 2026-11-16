#include "drivers/drv10975.h"

#include <stddef.h>

/* DRV10975 register addresses and names from TI SLVSCP2F, Section 8.5.2. */
enum
{
    /* Commands and device control. */
    REG_SpeedCtrl1 = 0x00,
    REG_SpeedCtrl2 = 0x01,
    REG_DevCtrl = 0x02, /* Unused EEPROM programming key register. */
    REG_EECtrl = 0x03,

    /* Read-only diagnostics. Addresses 0x04 to 0x0F and 0x1D are unlisted. */
    REG_Status = 0x10,
    REG_MotorSpeed1 = 0x11,
    REG_MotorSpeed2 = 0x12,
    REG_MotorPeriod1 = 0x13, /* Unused. The firmware reads MotorSpeed1/2. */
    REG_MotorPeriod2 = 0x14, /* Unused. */
    REG_MotorKt1 = 0x15,
    REG_MotorKt2 = 0x16,
    REG_MotorCurrent1 = 0x17,
    REG_MotorCurrent2 = 0x18,
    REG_IPDPosition = 0x19, /* Unused initial position detection result. */
    REG_SupplyVoltage = 0x1A,
    REG_SpeedCmd = 0x1B,
    REG_spdCmdBuffer = 0x1C,
    REG_FaultCode = 0x1E,

    /* Motor profile settings. */
    REG_MotorParam1 = 0x20,
    REG_MotorParam2 = 0x21,
    REG_MotorParam3 = 0x22,
    REG_SysOpt1 = 0x23,
    REG_SysOpt2 = 0x24,
    REG_SysOpt3 = 0x25,
    REG_SysOpt4 = 0x26,
    REG_SysOpt5 = 0x27,
    REG_SysOpt6 = 0x28,
    REG_SysOpt7 = 0x29,
    REG_SysOpt8 = 0x2A,
    REG_SysOpt9 = 0x2B
};

/* Masks and readback conversions from Table 9. */
enum
{
    SPEED_OVERRIDE = 1U << 7,
    CONTROL_SLEEP_DISABLE = 1U << 7,
    CONTROL_REGISTER_WRITE_ENABLE = 1U << 6,
    REGULATOR_3V3 = 1U << 2,
    CURRENT_READBACK_MASK = 0x07FF,
    CURRENT_READBACK_OFFSET = 1023,
    CURRENT_SCALE_MA = 3000,
    CURRENT_SCALE_COUNTS = 512,
    SUPPLY_SCALE_MV = 22800,
    SUPPLY_SCALE_COUNTS = 256
};

_Static_assert(DRV10975_CONFIG_COUNT == REG_SysOpt9 - REG_MotorParam1 + 1,
               "Configuration snapshot must cover MotorParam1 through SysOpt9.");

typedef struct
{
    uint8_t mask;
    uint8_t value;
} register_update_t;

typedef struct
{
    uint8_t address;
    uint8_t lsb;
    uint8_t max_code;
    size_t offset;
} field_layout_t;

/* Datasheet bit positions. Reserved bits have no entry and stay unchanged. */
static const field_layout_t field_layout[] = {
    {REG_MotorParam1, 7, 0b1, offsetof(drv10975_registers_t, MotorParam1.DoubleFreq)},
    {REG_MotorParam1, 0, 0b1111111, offsetof(drv10975_registers_t, MotorParam1.Rm)},

    {REG_MotorParam2, 7, 0b1, offsetof(drv10975_registers_t, MotorParam2.AdjMode)},
    {REG_MotorParam2, 0, 0b1111111, offsetof(drv10975_registers_t, MotorParam2.Kt)},

    {REG_MotorParam3, 7, 0b1, offsetof(drv10975_registers_t, MotorParam3.CtrlAdvMd)},
    {REG_MotorParam3, 0, 0b1111111, offsetof(drv10975_registers_t, MotorParam3.TCtrlAdv)},

    {REG_SysOpt1, 6, 0b11, offsetof(drv10975_registers_t, SysOpt1.ISDThr)},
    {REG_SysOpt1, 4, 0b11, offsetof(drv10975_registers_t, SysOpt1.IPDAdvcAgl)},
    {REG_SysOpt1, 3, 0b1, offsetof(drv10975_registers_t, SysOpt1.ISDen)},
    {REG_SysOpt1, 2, 0b1, offsetof(drv10975_registers_t, SysOpt1.RvsDrEn)},
    {REG_SysOpt1, 0, 0b11, offsetof(drv10975_registers_t, SysOpt1.RvsDrThr)},

    {REG_SysOpt2, 6, 0b11, offsetof(drv10975_registers_t, SysOpt2.OpenLCurr)},
    {REG_SysOpt2, 3, 0b111, offsetof(drv10975_registers_t, SysOpt2.OpLCurrRt)},
    {REG_SysOpt2, 0, 0b111, offsetof(drv10975_registers_t, SysOpt2.BrkDoneThr)},

    {REG_SysOpt3, 6, 0b11, offsetof(drv10975_registers_t, SysOpt3.CtrlCoef)},
    {REG_SysOpt3, 3, 0b111, offsetof(drv10975_registers_t, SysOpt3.StAccel2)},
    {REG_SysOpt3, 0, 0b111, offsetof(drv10975_registers_t, SysOpt3.StAccel)},

    {REG_SysOpt4, 3, 0b11111, offsetof(drv10975_registers_t, SysOpt4.Op2ClsThr)},
    {REG_SysOpt4, 0, 0b111, offsetof(drv10975_registers_t, SysOpt4.AlignTime)},

    {REG_SysOpt5, 7, 0b1, offsetof(drv10975_registers_t, SysOpt5.LockEn3)},
    {REG_SysOpt5, 6, 0b1, offsetof(drv10975_registers_t, SysOpt5.LockEn2)},
    {REG_SysOpt5, 5, 0b1, offsetof(drv10975_registers_t, SysOpt5.LockEn1)},
    {REG_SysOpt5, 4, 0b1, offsetof(drv10975_registers_t, SysOpt5.LockEn0)},
    {REG_SysOpt5, 3, 0b1, offsetof(drv10975_registers_t, SysOpt5.AVSIndEn)},
    {REG_SysOpt5, 2, 0b1, offsetof(drv10975_registers_t, SysOpt5.AVSMEn)},
    {REG_SysOpt5, 1, 0b1, offsetof(drv10975_registers_t, SysOpt5.AVSMMd)},
    {REG_SysOpt5, 0, 0b1, offsetof(drv10975_registers_t, SysOpt5.IPDRlsMd)},

    {REG_SysOpt6, 4, 0b1111, offsetof(drv10975_registers_t, SysOpt6.SWiLimitThr)},
    {REG_SysOpt6, 1, 0b111, offsetof(drv10975_registers_t, SysOpt6.HWiLimitThr)},

    {REG_SysOpt7, 7, 0b1, offsetof(drv10975_registers_t, SysOpt7.LockEn5)},
    {REG_SysOpt7, 4, 0b111, offsetof(drv10975_registers_t, SysOpt7.ClsLpAccel)},
    {REG_SysOpt7, 0, 0b1111, offsetof(drv10975_registers_t, SysOpt7.Deadtime)},

    {REG_SysOpt8, 4, 0b1111, offsetof(drv10975_registers_t, SysOpt8.IPDCurrThr)},
    {REG_SysOpt8, 3, 0b1, offsetof(drv10975_registers_t, SysOpt8.LockEn4)},
    {REG_SysOpt8, 2, 0b1, offsetof(drv10975_registers_t, SysOpt8.VregSel)},
    {REG_SysOpt8, 0, 0b11, offsetof(drv10975_registers_t, SysOpt8.IPDClk)},

    {REG_SysOpt9, 6, 0b11, offsetof(drv10975_registers_t, SysOpt9.FGOLsel)},
    {REG_SysOpt9, 4, 0b11, offsetof(drv10975_registers_t, SysOpt9.FGcycle)},
    {REG_SysOpt9, 2, 0b11, offsetof(drv10975_registers_t, SysOpt9.KtLckThr)},
    {REG_SysOpt9, 1, 0b1, offsetof(drv10975_registers_t, SysOpt9.SpdCtrlMd)},
    {REG_SysOpt9, 0, 0b1, offsetof(drv10975_registers_t, SysOpt9.CLoopDis)},
};

static bool write_register(drv10975_t *driver, uint8_t reg, uint8_t value)
{
    uint8_t bytes[] = {reg, value};
    driver->last_register = reg;
    bool written = stm32_i2c_write(driver->bus, driver->address, bytes, sizeof(bytes));
    driver->last_transfer = driver->bus->diagnostic;
    return written;
}

static bool read_register(drv10975_t *driver, uint8_t reg, uint8_t *value)
{
    /* TI SLVUB30A specifies a STOP after selecting the register. */
    driver->last_register = reg;
    bool selected = stm32_i2c_write(driver->bus, driver->address, &reg, 1);
    if (!selected)
    {
        driver->last_transfer = driver->bus->diagnostic;
        return false;
    }

    bool read_ok = stm32_i2c_read(driver->bus, driver->address, value, 1);
    driver->last_transfer = driver->bus->diagnostic;
    return read_ok;
}

static bool update_register(drv10975_t *driver, uint8_t reg, const register_update_t *setting)
{
    uint8_t current;
    bool read_ok = read_register(driver, reg, &current);
    if (!read_ok)
    {
        return false;
    }

    /* A one in the mask selects a bit to replace. */
    uint8_t preserved_bits = current & (uint8_t)~setting->mask;
    uint8_t configured_bits = setting->value & setting->mask;
    uint8_t expected = preserved_bits | configured_bits;
    bool written = write_register(driver, reg, expected);
    if (!written)
    {
        return false;
    }

    uint8_t actual;
    bool readback_ok = read_register(driver, reg, &actual);
    if (!readback_ok)
    {
        return false;
    }
    return actual == expected;
}

static bool read_configuration(drv10975_t *driver, volatile uint8_t *destination)
{
    for (uint8_t i = 0; i < DRV10975_CONFIG_COUNT; ++i)
    {
        uint8_t reg = (uint8_t)(REG_MotorParam1 + i);
        uint8_t value;
        bool read_ok = read_register(driver, reg, &value);
        if (!read_ok)
        {
            return false;
        }
        destination[i] = value;
    }
    return true;
}

static bool encode_updates(drv10975_t *driver, const drv10975_registers_t *registers,
                           register_update_t *updates)
{
    /* Each profile field is a byte. Its offset connects it to the bit layout. */
    const unsigned char *profile_bytes = (const unsigned char *)registers;
    const size_t field_count = sizeof(field_layout) / sizeof(field_layout[0]);
    for (size_t i = 0; i < field_count; ++i)
    {
        const field_layout_t *field = &field_layout[i];
        uint8_t code = profile_bytes[field->offset];
        if (code == DRV10975_KEEP)
        {
            continue;
        }
        if (code > field->max_code)
        {
            driver->last_register = field->address;
            return false;
        }

        size_t index = field->address - REG_MotorParam1;
        uint8_t field_mask = (uint8_t)(field->max_code << field->lsb);
        uint8_t field_value = (uint8_t)(code << field->lsb);
        updates[index].mask |= field_mask;
        updates[index].value |= field_value;
    }
    return true;
}

bool drv10975_configure(drv10975_t *driver, const drv10975_registers_t *registers,
                        volatile drv10975_config_snapshot_t *snapshot)
{
    if (registers == NULL || snapshot == NULL)
    {
        return false;
    }

    /* Validate all field codes before writing any configuration register. */
    register_update_t updates[DRV10975_CONFIG_COUNT] = {0};
    bool valid = encode_updates(driver, registers, updates);
    if (!valid)
    {
        return false;
    }

    bool before_ok = read_configuration(driver, snapshot->before);
    if (!before_ok)
    {
        return false;
    }

    for (uint8_t i = 0; i < DRV10975_CONFIG_COUNT; ++i)
    {
        if (updates[i].mask == 0U)
        {
            continue;
        }

        uint8_t reg = (uint8_t)(REG_MotorParam1 + i);
        bool updated = update_register(driver, reg, &updates[i]);
        if (!updated)
        {
            return false;
        }
    }

    bool after_ok = read_configuration(driver, snapshot->after);
    return after_ok;
}

bool drv10975_set_speed(drv10975_t *driver, uint16_t command)
{
    if (command > DRV10975_MAX_SPEED_COMMAND)
    {
        return false;
    }

    /* Writing the low byte latches bit 8 and applies the complete command. */
    uint8_t high_byte = SPEED_OVERRIDE | (uint8_t)(command >> 8);
    uint8_t low_byte = (uint8_t)command;
    bool high_written = write_register(driver, REG_SpeedCtrl2, high_byte);
    if (!high_written)
    {
        return false;
    }

    bool low_written = write_register(driver, REG_SpeedCtrl1, low_byte);
    return low_written;
}

bool drv10975_take_control(drv10975_t *driver)
{
    uint8_t control = CONTROL_SLEEP_DISABLE | CONTROL_REGISTER_WRITE_ENABLE;
    bool control_written = write_register(driver, REG_EECtrl, control);
    if (!control_written)
    {
        return false;
    }

    bool stopped = drv10975_set_speed(driver, 0);
    return stopped;
}

bool drv10975_read_regulator_3v3(drv10975_t *driver, bool *is_3v3)
{
    uint8_t regulator;
    bool read_ok = read_register(driver, REG_SysOpt8, &regulator);
    if (!read_ok)
    {
        return false;
    }

    *is_3v3 = (regulator & REGULATOR_3V3) != 0U;
    return true;
}

bool drv10975_read_status(drv10975_t *driver, uint8_t *status)
{
    bool read_ok = read_register(driver, REG_Status, status);
    return read_ok;
}

bool drv10975_read_fault(drv10975_t *driver, uint8_t *fault)
{
    bool read_ok = read_register(driver, REG_FaultCode, fault);
    return read_ok;
}

static bool read_word(drv10975_t *driver, uint8_t high_register, uint8_t low_register,
                      uint16_t *word)
{
    uint8_t high;
    uint8_t low;
    /* TI specifies reading the high register before the low register. */
    bool high_read = read_register(driver, high_register, &high);
    if (!high_read)
    {
        return false;
    }

    bool low_read = read_register(driver, low_register, &low);
    if (!low_read)
    {
        return false;
    }

    *word = ((uint16_t)high << 8) | low;
    return true;
}

static uint32_t current_to_milliamps(uint16_t raw)
{
    /* Keep TI's piecewise conversion. Near the offset boundary, the raw
     * sample is needed to interpret a jump in the converted value. */
    uint16_t counts = raw;
    if (counts >= CURRENT_READBACK_OFFSET)
    {
        counts -= CURRENT_READBACK_OFFSET;
    }
    return (uint32_t)counts * CURRENT_SCALE_MA / CURRENT_SCALE_COUNTS;
}

bool drv10975_read_telemetry(drv10975_t *driver, drv10975_telemetry_t *telemetry)
{
    bool read_ok = read_word(driver, REG_MotorSpeed1, REG_MotorSpeed2,
                             &telemetry->speed_deci_electrical_hz);
    if (!read_ok)
    {
        return false;
    }

    uint16_t current_word;
    read_ok = read_word(driver, REG_MotorCurrent1, REG_MotorCurrent2, &current_word);
    if (!read_ok)
    {
        return false;
    }

    uint8_t supply;
    read_ok = read_register(driver, REG_SupplyVoltage, &supply);
    if (!read_ok)
    {
        return false;
    }
    read_ok = read_register(driver, REG_SpeedCmd, &telemetry->input_command);
    if (!read_ok)
    {
        return false;
    }
    read_ok = read_register(driver, REG_spdCmdBuffer, &telemetry->buffered_command);
    if (!read_ok)
    {
        return false;
    }

    read_ok = read_word(driver, REG_MotorKt1, REG_MotorKt2, &telemetry->kt_raw);
    if (!read_ok)
    {
        return false;
    }

    telemetry->phase_current_raw = current_word & CURRENT_READBACK_MASK;
    telemetry->phase_current_ma = current_to_milliamps(telemetry->phase_current_raw);
    telemetry->supply_mv = (uint32_t)supply * SUPPLY_SCALE_MV / SUPPLY_SCALE_COUNTS;
    return true;
}
