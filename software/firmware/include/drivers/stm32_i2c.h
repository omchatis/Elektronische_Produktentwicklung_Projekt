#ifndef DRIVERS_STM32_I2C_H
#define DRIVERS_STM32_I2C_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx.h"

typedef enum
{
    I2C_ERROR_NONE,
    I2C_ERROR_PERIPHERAL,
    I2C_ERROR_WAIT_TIMEOUT,
    I2C_ERROR_BUS_BUSY,
    I2C_ERROR_LINES_LOW,
    I2C_ERROR_INVALID_TRANSFER
} stm32_i2c_error_t;

typedef struct
{
    stm32_i2c_error_t error;
    uint8_t address;
    bool read;
    uint8_t byte_index; /* Equals the byte count while waiting for STOP. */
    uint32_t waiting_for;
    uint32_t isr;
    bool scl_high; /* Pin levels are captured on a hardware error. */
    bool sda_high;
} stm32_i2c_diagnostic_t;

typedef struct
{
    I2C_TypeDef *registers;
    GPIO_TypeDef *scl_port;
    GPIO_TypeDef *sda_port;
    uint32_t scl_mask;
    uint32_t sda_mask;
    uint32_t timing;
    uint32_t timeout_ms;
    stm32_i2c_diagnostic_t diagnostic;
} stm32_i2c_bus_t;

/* Board setup supplies the clocks and GPIO alternate functions first.
 * Reset leaves I2C disabled until the first transfer sees both lines high. */
void stm32_i2c_reset(stm32_i2c_bus_t *bus);

/* Single-direction transactions with a STOP, using a 7-bit target address.
 * Calls are bounded by the bus timeout and must run outside interrupts. */
bool stm32_i2c_write(stm32_i2c_bus_t *bus, uint8_t address,
                    const uint8_t *data, uint8_t count);
bool stm32_i2c_read(stm32_i2c_bus_t *bus, uint8_t address,
                   uint8_t *data, uint8_t count);

#endif
