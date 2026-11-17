#include "drivers/stm32_i2c.h"

#include <stddef.h>

#include "board/timebase.h"

void stm32_i2c_reset(stm32_i2c_bus_t *bus)
{
    /* TIMINGR can be programmed only with the peripheral disabled. */
    bus->registers->CR1 = 0;
    bus->registers->TIMINGR = bus->timing;
}

static bool scl_is_high(const stm32_i2c_bus_t *bus)
{
    return (bus->scl_port->IDR & bus->scl_mask) != 0U;
}

static bool sda_is_high(const stm32_i2c_bus_t *bus)
{
    return (bus->sda_port->IDR & bus->sda_mask) != 0U;
}

static void record_error(stm32_i2c_bus_t *bus, stm32_i2c_error_t error,
                         uint32_t waiting_for)
{
    /* Capture the peripheral and pins before cleanup changes them. */
    bus->diagnostic.error = error;
    bus->diagnostic.isr = bus->registers->ISR;
    bus->diagnostic.waiting_for = waiting_for;
    bus->diagnostic.scl_high = scl_is_high(bus);
    bus->diagnostic.sda_high = sda_is_high(bus);
}

static bool wait_for_flag(stm32_i2c_bus_t *bus, uint32_t flag, uint32_t started_ms)
{
    const uint32_t errors = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;
    do
    {
        uint32_t status = bus->registers->ISR;
        if ((status & errors) != 0U)
        {
            record_error(bus, I2C_ERROR_PERIPHERAL, flag);
            return false;
        }
        if ((status & flag) != 0U)
        {
            return true;
        }
    } while (timebase_millis() - started_ms < bus->timeout_ms);

    record_error(bus, I2C_ERROR_WAIT_TIMEOUT, flag);
    return false;
}

static bool prepare_bus(stm32_i2c_bus_t *bus, uint32_t started_ms)
{
    I2C_TypeDef *i2c = bus->registers;
    if ((i2c->CR1 & I2C_CR1_PE) == 0U)
    {
        /* A sleeping driver can remove power from the bus pull-ups. */
        bool scl_high = scl_is_high(bus);
        bool sda_high = sda_is_high(bus);
        if (!scl_high || !sda_high)
        {
            record_error(bus, I2C_ERROR_LINES_LOW, 0);
            return false;
        }
        i2c->CR1 = I2C_CR1_PE;
    }

    while ((i2c->ISR & I2C_ISR_BUSY) != 0U)
    {
        if (timebase_millis() - started_ms >= bus->timeout_ms)
        {
            record_error(bus, I2C_ERROR_BUS_BUSY, I2C_ISR_BUSY);
            return false;
        }
    }
    return true;
}

static void request_stop_if_owned(I2C_TypeDef *i2c)
{
    uint32_t status = i2c->ISR;
    bool bus_busy = (status & I2C_ISR_BUSY) != 0U;
    /* Arbitration loss relinquishes the bus. AUTOEND handles a NACK. */
    bool stop_handled_elsewhere = (status & (I2C_ISR_ARLO | I2C_ISR_NACKF)) != 0U;
    if (bus_busy && !stop_handled_elsewhere)
    {
        i2c->CR2 |= I2C_CR2_STOP;
    }
}

/* Exactly one buffer is supplied. RX data implies a read transaction. */
static bool transfer(stm32_i2c_bus_t *bus, uint8_t address,
                     const uint8_t *tx_data, uint8_t *rx_data, uint8_t count)
{
    bool read = rx_data != NULL;
    bus->diagnostic = (stm32_i2c_diagnostic_t){
        .address = address,
        .read = read,
    };
    if (address > 0x7FU || count == 0U || (tx_data == NULL && rx_data == NULL))
    {
        bus->diagnostic.error = I2C_ERROR_INVALID_TRANSFER;
        return false;
    }

    /* One deadline covers bus idle, every byte, and STOP. */
    uint32_t started_ms = timebase_millis();
    bool ready = prepare_bus(bus, started_ms);
    if (!ready)
    {
        return false;
    }

    I2C_TypeDef *i2c = bus->registers;
    /* Ones in ICR clear stale flags. AUTOEND generates STOP after NBYTES. */
    i2c->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF |
               I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    uint32_t address_field = (uint32_t)address << 1;
    uint32_t count_field = (uint32_t)count << I2C_CR2_NBYTES_Pos;
    uint32_t direction_field = read ? I2C_CR2_RD_WRN : 0U;
    i2c->CR2 = address_field | count_field | direction_field | I2C_CR2_AUTOEND | I2C_CR2_START;

    uint32_t byte_ready_flag = read ? I2C_ISR_RXNE : I2C_ISR_TXIS;
    for (uint8_t i = 0; i < count; ++i)
    {
        bus->diagnostic.byte_index = i;
        bool byte_ready = wait_for_flag(bus, byte_ready_flag, started_ms);
        if (!byte_ready)
        {
            request_stop_if_owned(i2c);
            return false;
        }

        if (read)
        {
            rx_data[i] = (uint8_t)i2c->RXDR;
        }
        else
        {
            i2c->TXDR = tx_data[i];
        }
    }

    bus->diagnostic.byte_index = count;
    bool stop_detected = wait_for_flag(bus, I2C_ISR_STOPF, started_ms);
    return stop_detected;
}

bool stm32_i2c_write(stm32_i2c_bus_t *bus, uint8_t address,
                    const uint8_t *data, uint8_t count)
{
    bool transferred = transfer(bus, address, data, NULL, count);
    return transferred;
}

bool stm32_i2c_read(stm32_i2c_bus_t *bus, uint8_t address,
                   uint8_t *data, uint8_t count)
{
    bool transferred = transfer(bus, address, NULL, data, count);
    return transferred;
}
