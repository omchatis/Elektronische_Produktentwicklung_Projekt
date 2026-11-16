#include "board/motor_board.h"

enum
{
    MOTOR1_I2C_AF = 4,
    MOTOR1_I2C_TIMEOUT_MS = 5,
    /* HSI16 kernel clock, about 90 kHz SCL with the board pull-ups. */
    MOTOR1_I2C_TIMING = 0x30511216U,
    EXTI_CONFIG_PINS_8_TO_11 = 2
};

struct motor_board
{
    stm32_i2c_bus_t *i2c;
    GPIO_TypeDef *speed_port;
    uint32_t speed_pin_mask;
    uint32_t fg_line_mask;
    volatile uint32_t fg_edges;
};

static stm32_i2c_bus_t motor1_bus = {
    .registers = I2C2,
    .scl_port = GPIOA,
    .sda_port = GPIOA,
    .scl_mask = GPIO_IDR_ID9,
    .sda_mask = GPIO_IDR_ID8,
    .timing = MOTOR1_I2C_TIMING,
    .timeout_ms = MOTOR1_I2C_TIMEOUT_MS,
};

motor_board_t board_motor1 = {
    .i2c = &motor1_bus,
    .speed_port = GPIOA,
    .speed_pin_mask = GPIO_BSRR_BS0,
    .fg_line_mask = EXTI_IMR1_IM11,
};

static void initialize_motor1_outputs(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;

    /* Preload the latches before enabling PA0 (SPEED) and PC6 (DIR). */
    GPIOA->BSRR = GPIO_BSRR_BR0;
    GPIOC->BSRR = GPIO_BSRR_BR6;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0) | GPIO_MODER_MODE0_0;
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODE6) | GPIO_MODER_MODE6_0;
}

static void initialize_motor1_i2c(void)
{
    /* PA8 SDA and PA9 SCL use AF4 and open drain. The board supplies pull-ups. */
    GPIOA->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    uint32_t alternate_functions = GPIOA->AFR[1];
    alternate_functions &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
    alternate_functions |= MOTOR1_I2C_AF << GPIO_AFRH_AFSEL8_Pos;
    alternate_functions |= MOTOR1_I2C_AF << GPIO_AFRH_AFSEL9_Pos;
    GPIOA->AFR[1] = alternate_functions;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9)) |
                   GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;

    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_I2C2SEL) | RCC_CCIPR_I2C2SEL_1;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    (void)RCC->APB1ENR1;
    stm32_i2c_reset(&motor1_bus);
}

static void initialize_motor1_fg(void)
{
    /* PC11 has an external pull-up. Route its falling edges to EXTI11. */
    GPIOC->MODER &= ~GPIO_MODER_MODE11;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD11;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    uint32_t port_selection = SYSCFG->EXTICR[EXTI_CONFIG_PINS_8_TO_11];
    port_selection &= ~SYSCFG_EXTICR3_EXTI11;
    port_selection |= SYSCFG_EXTICR3_EXTI11_PC;
    SYSCFG->EXTICR[EXTI_CONFIG_PINS_8_TO_11] = port_selection;

    EXTI->IMR1 &= ~EXTI_IMR1_IM11;
    EXTI->RTSR1 &= ~EXTI_RTSR1_RT11;
    EXTI->FTSR1 |= EXTI_FTSR1_FT11;
    EXTI->PR1 = EXTI_PR1_PIF11;
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void motor_board_initialize(void)
{
    initialize_motor1_outputs();
    initialize_motor1_i2c();
    initialize_motor1_fg();
}

stm32_i2c_bus_t *motor_board_i2c(const motor_board_t *motor)
{
    return motor->i2c;
}

void motor_board_set_speed_pin(const motor_board_t *motor, bool high)
{
    uint32_t write_mask = motor->speed_pin_mask;
    if (!high)
    {
        /* The upper half of BSRR resets the corresponding output bit. */
        write_mask <<= 16;
    }
    motor->speed_port->BSRR = write_mask;
}

bool motor_board_speed_pin_high(const motor_board_t *motor)
{
    return (motor->speed_port->IDR & motor->speed_pin_mask) != 0U;
}

void motor_board_arm_fg(motor_board_t *motor)
{
    /* Clear any edge accumulated while the line was masked. PR1 is W1C. */
    EXTI->PR1 = motor->fg_line_mask;
    EXTI->IMR1 |= motor->fg_line_mask;
}

void motor_board_disarm_fg(const motor_board_t *motor)
{
    EXTI->IMR1 &= ~motor->fg_line_mask;
}

uint32_t motor_board_fg_edges(const motor_board_t *motor)
{
    return motor->fg_edges;
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t pending = EXTI->PR1 & EXTI->IMR1;
    if ((pending & EXTI_PR1_PIF11) != 0U)
    {
        EXTI->PR1 = EXTI_PR1_PIF11;
        ++board_motor1.fg_edges;
    }
}

bool motor_board_debugger_attached(void)
{
    return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U;
}

void motor_board_debug_break(void)
{
    __BKPT(0);
}
