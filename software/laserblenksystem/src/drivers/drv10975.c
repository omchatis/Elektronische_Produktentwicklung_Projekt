#include "drivers/drv10975.h"

/* Register addresses and conversions follow TI SLVSCP2F, Table 9. */
enum
{
    REG_SPEED_LOW = 0x00,
    REG_SPEED_HIGH = 0x01,
    REG_CONTROL = 0x03,
    REG_STATUS = 0x10,
    REG_SPEED_READBACK = 0x11,
    REG_KT_READBACK = 0x15,
    REG_CURRENT_READBACK = 0x17,
    REG_SUPPLY_VOLTAGE = 0x1A,
    REG_INPUT_COMMAND = 0x1B,
    REG_BUFFERED_COMMAND = 0x1C,
    REG_FAULT = 0x1E,

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

static bool update_register(drv10975_t *driver, const drv10975_setting_t *setting)
{
    uint8_t current;
    bool read_ok = read_register(driver, setting->address, &current);
    if (!read_ok)
    {
        return false;
    }

    /* A one in the mask selects a bit to replace. */
    uint8_t preserved_bits = current & (uint8_t)~setting->mask;
    uint8_t configured_bits = setting->value & setting->mask;
    uint8_t expected = preserved_bits | configured_bits;
    bool written = write_register(driver, setting->address, expected);
    if (!written)
    {
        return false;
    }

    uint8_t actual;
    bool readback_ok = read_register(driver, setting->address, &actual);
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
        uint8_t reg = (uint8_t)(DRV10975_MOTOR_RESISTANCE + i);
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

bool drv10975_configure(drv10975_t *driver, const drv10975_setting_t *settings,
                        size_t count, volatile drv10975_config_snapshot_t *snapshot)
{
    if (settings == NULL || snapshot == NULL || count == 0U || count > DRV10975_CONFIG_COUNT)
    {
        return false;
    }
    for (size_t i = 0; i < count; ++i)
    {
        uint8_t reg = settings[i].address;
        if (reg < DRV10975_MOTOR_RESISTANCE || reg > DRV10975_SPEED_MODE)
        {
            driver->last_register = reg;
            return false;
        }
    }

    bool before_ok = read_configuration(driver, snapshot->before);
    if (!before_ok)
    {
        return false;
    }
    for (size_t i = 0; i < count; ++i)
    {
        bool updated = update_register(driver, &settings[i]);
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
    bool high_written = write_register(driver, REG_SPEED_HIGH, high_byte);
    if (!high_written)
    {
        return false;
    }

    bool low_written = write_register(driver, REG_SPEED_LOW, low_byte);
    return low_written;
}

bool drv10975_take_control(drv10975_t *driver)
{
    uint8_t control = CONTROL_SLEEP_DISABLE | CONTROL_REGISTER_WRITE_ENABLE;
    bool control_written = write_register(driver, REG_CONTROL, control);
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
    bool read_ok = read_register(driver, DRV10975_REGULATOR, &regulator);
    if (!read_ok)
    {
        return false;
    }

    *is_3v3 = (regulator & REGULATOR_3V3) != 0U;
    return true;
}

bool drv10975_read_status(drv10975_t *driver, uint8_t *status)
{
    bool read_ok = read_register(driver, REG_STATUS, status);
    return read_ok;
}

bool drv10975_read_fault(drv10975_t *driver, uint8_t *fault)
{
    bool read_ok = read_register(driver, REG_FAULT, fault);
    return read_ok;
}

static bool read_word(drv10975_t *driver, uint8_t first_register, uint16_t *word)
{
    uint8_t high;
    uint8_t low;
    /* TI specifies reading the high register before the low register. */
    bool high_read = read_register(driver, first_register, &high);
    if (!high_read)
    {
        return false;
    }

    uint8_t low_register = first_register + 1U;
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
    bool read_ok = read_word(driver, REG_SPEED_READBACK, &telemetry->speed_deci_electrical_hz);
    if (!read_ok)
    {
        return false;
    }

    uint16_t current_word;
    read_ok = read_word(driver, REG_CURRENT_READBACK, &current_word);
    if (!read_ok)
    {
        return false;
    }

    uint8_t supply;
    read_ok = read_register(driver, REG_SUPPLY_VOLTAGE, &supply);
    if (!read_ok)
    {
        return false;
    }
    read_ok = read_register(driver, REG_INPUT_COMMAND, &telemetry->input_command);
    if (!read_ok)
    {
        return false;
    }
    read_ok = read_register(driver, REG_BUFFERED_COMMAND, &telemetry->buffered_command);
    if (!read_ok)
    {
        return false;
    }

    read_ok = read_word(driver, REG_KT_READBACK, &telemetry->kt_raw);
    if (!read_ok)
    {
        return false;
    }

    telemetry->phase_current_raw = current_word & CURRENT_READBACK_MASK;
    telemetry->phase_current_ma = current_to_milliamps(telemetry->phase_current_raw);
    telemetry->supply_mv = (uint32_t)supply * SUPPLY_SCALE_MV / SUPPLY_SCALE_COUNTS;
    return true;
}
