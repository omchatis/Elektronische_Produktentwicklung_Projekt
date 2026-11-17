#ifndef BOARD_MOTOR_BOARD_H
#define BOARD_MOTOR_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/stm32_i2c.h"

typedef struct motor_board motor_board_t;
extern motor_board_t board_motor1;

/* Initialize the board wiring for the enabled motor after starting SysTick. */
void motor_board_initialize(void);
stm32_i2c_bus_t *motor_board_i2c(const motor_board_t *motor);
void motor_board_set_speed_pin(const motor_board_t *motor, bool high);
bool motor_board_speed_pin_high(const motor_board_t *motor);

/* FG is the driver's frequency output. Arm capture before sending the run command. */
void motor_board_arm_fg(motor_board_t *motor);
void motor_board_disarm_fg(const motor_board_t *motor);
uint32_t motor_board_fg_edges(const motor_board_t *motor);

bool motor_board_debugger_attached(void);
void motor_board_debug_break(void);

#endif
