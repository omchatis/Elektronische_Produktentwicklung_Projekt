#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx.h"

/*
 * Motor 1 continuous drive for the STM32G431 and DRV10975Z.
 *
 * The STM32 wakes and configures the driver over I2C, requests motion once,
 * monitors the driver's registers, and requests a stop on an error,
 * overspeed, or a debugger stop request.
 * The DRV10975Z generates the three-phase PWM and runs the startup sequence.
 * See TI DRV10975 datasheet SLVSCP2F, sections 8.4 and 8.5.3, for driver
 * operation and register definitions.
 *
 * For startup tuning, drv_configure_motor() contains the PWM frequency,
 * startup current, alignment time, and electrical acceleration settings.
 * Inspect the motor_* variables during the run or after it stops.
 */
enum {
    TEST_COMMAND = 24, /* About 4.7% drive command for the low-speed run. */
    TEST_MAX_RPM = 1000, /* Stop at this reported speed (100 electrical Hz). */
    TEST_OPEN_LOOP_ONLY = 0, /* 0 allows handoff. Set to 1 to test only startup. */
    WAKE_TIMEOUT_MS = 500, /* Bench wake window; SPEED high can request motion. */
    I2C_TIMEOUT_MS = 5, /* Per-transfer deadline, including the wait for bus idle. */
    DRIVER_LOCK_RECOVERY_MS = 5500, /* Allow the driver's 5 s lock recovery. */

    /* DRV10975Z 7-bit address and command registers. */
    DRV_ADDRESS = 0x52,
    DRV_SPEED_LOW = 0x00,
    DRV_SPEED_HIGH = 0x01,
    DRV_CONTROL = 0x03,

    /* Driver status and telemetry registers. */
    DRV_STATUS = 0x10,
    DRV_RPM_HIGH = 0x11,
    DRV_RPM_LOW = 0x12,
    DRV_CURRENT_HIGH = 0x17,
    DRV_CURRENT_LOW = 0x18,
    DRV_SUPPLY_VOLTAGE = 0x1A,
    DRV_COMMAND_READBACK = 0x1B,
    DRV_BUFFERED_COMMAND = 0x1C,
    DRV_FAULT = 0x1E,

    /* Motor parameters and operating options (volatile register image). */
    DRV_MOTOR_RESISTANCE = 0x20,
    DRV_MOTOR_BEMF = 0x21,
    DRV_COMMUTATION_ADVANCE = 0x22,
    DRV_STARTUP_OPTIONS = 0x23,
    DRV_STARTUP_CURRENT = 0x24,
    DRV_STARTUP_ACCELERATION = 0x25,
    DRV_ALIGNMENT = 0x26,
    DRV_PROTECTION = 0x27,
    DRV_CURRENT_LIMITS = 0x28,
    DRV_CLOSED_LOOP_RAMP = 0x29,
    DRV_REGULATOR = 0x2A,
    DRV_SPEED_MODE = 0x2B,
    DRV_CONFIG_COUNT = DRV_SPEED_MODE - DRV_MOTOR_RESISTANCE + 1,

    DRV_OVERRIDE = 1U << 7, /* SpeedCtrl2: select the I2C speed command. */
    DRV_SLEEP_DISABLE = 1U << 7, /* EECtrl: keep the awake driver accessible. */
    DRV_REGISTER_WRITE_ENABLE = 1U << 6, /* EECtrl: enable configuration writes. */
    DRV_INITIAL_SPEED_DETECT = 1U << 3, /* SysOpt1 ISDen. */
    DRV_REGULATOR_3V3 = 1U << 2, /* SysOpt8 VregSel: 3.3 V setting. */
    DRV_STATUS_ERROR = 0xF0, /* Overtemp, sleep, overcurrent, or lock. */
    DRV_STATUS_LOCK = 1U << 4,
    DRV_STATUS_THERMAL_OR_OVERCURRENT = (1U << 7) | (1U << 5),

    /* Disconnected pair measurement 1.182 ohm minus 0.7 ohm shorted-lead
     * resistance gives 0.482 ohm between phases, or 0.241 ohm per phase.
     * Nearest DRV10975 encoding is 0x28, representing 0.2352 ohm. */
    MOTOR_RESISTANCE_FIELD = 0x28,
    /* Disconnected phase-to-phase BEMF, Vpp = 1.3 V, electrical T = 8 ms:
     * Kt = (Vpp / 2) * T = 0.0052 V/Hz.
     * Nearest encoding is 0x07, representing about 0.00485 V/Hz. */
    MOTOR_BEMF_FIELD = 0x07,
};

typedef enum {
    MOTOR_INITIALIZING,
    MOTOR_RUNNING,
    MOTOR_STOP_REQUESTED, /* The debugger requested a stop. Check the ACK. */
    MOTOR_I2C_ERROR,
    MOTOR_REGULATOR_MISMATCH,
    MOTOR_CONFIGURATION_ERROR,
    MOTOR_DRIVER_FAULT,
    MOTOR_OVERSPEED,
    MOTOR_WAITING_FOR_DRIVER,
} MotorTestState;

typedef enum {
    I2C_ERROR_NONE,
    I2C_ERROR_PERIPHERAL,
    I2C_ERROR_WAIT_TIMEOUT,
    I2C_ERROR_BUS_BUSY,
    I2C_ERROR_LINES_LOW,
} I2cError;

typedef struct {
    I2cError error;
    uint8_t reg; /* DRV register selected by the caller. */
    bool read; /* Direction of this transfer, including the register-address write. */
    uint32_t byte_index; /* Zero-based byte index, or count when waiting for STOP. */
    uint32_t waiting_for; /* STM32 I2C flag being checked when the error occurred. */
    uint32_t isr; /* STM32 I2C status register at the error. */
    /* GPIO levels captured at the error. These fields can be stale on success. */
    bool scl_high;
    bool sda_high;
    bool speed_high;
} I2cDiagnostic;

/* Inspect these in the ST-Link debugger. Volatile keeps the values available
 * as the firmware updates them. The final values remain visible in the idle loop. */
volatile MotorTestState motor_state = MOTOR_INITIALIZING;
/* Status bits 7:4 report temperature, sleep, overcurrent, and lock.
 * FaultCode gives the cause of a lock. Bit 4 indicates an open-loop stall. */
volatile uint8_t motor_status; /* Status byte from register 0x10. */
volatile uint8_t motor_fault; /* FaultCode captured on the driver fault path. */
volatile uint16_t motor_rpm; /* Driver readback scaled for six pole pairs. */
/* FG is configured to pulse only in closed loop. Repeated edges show that
 * the driver made the handoff, even if a later fault ended the test. */
volatile uint32_t motor_closed_loop_fg_edges;

/* Telemetry is sampled before the stop command. Open-loop speed can reflect
 * the commutation frequency, so use physical motion to confirm shaft rotation. */
volatile uint16_t motor_phase_current_raw;
volatile uint32_t motor_phase_current_mA;
volatile uint32_t motor_peak_phase_current_mA; /* Largest converted sample. */
volatile uint16_t motor_current_raw_at_peak;
volatile uint16_t motor_current_raw_min = UINT16_MAX; /* Sentinel until first sample. */
volatile uint16_t motor_current_raw_max;
volatile uint32_t motor_supply_mV;
volatile uint8_t motor_speed_command; /* Driver input command readback, 0–255. */
volatile uint8_t motor_buffered_command; /* Driver command buffer, 0–255. */
volatile bool motor_stop_acknowledged; /* Both zero-command register writes completed. */
volatile bool motor_stop_requested; /* Set true and let the firmware run to send speed 0. */
volatile uint8_t motor_initial_status;
volatile uint8_t motor_initial_fault;
/* Configuration snapshots. Array index i corresponds to register 0x20 + i. */
volatile uint8_t motor_config_before[DRV_CONFIG_COUNT];
volatile uint8_t motor_config_after[DRV_CONFIG_COUNT];
volatile I2cDiagnostic i2c_diagnostic;
/* Preserved before the final stop attempt changes the transport diagnostics. */
volatile I2cDiagnostic motor_i2c_failure;
volatile uint32_t motor_wake_attempts; /* Retries with SPEED held high. */
volatile uint32_t motor_wake_elapsed_ms; /* Time spent in the retry loop. */
/* Set true in the debugger at main() only with the motor phases disconnected.
 * Pauses with SPEED high so the driver supplies can be measured. */
volatile bool motor_pause_after_wake;

/* SysTick increments this counter every millisecond while the MCU runs. */
static volatile uint32_t tick_ms;

void SysTick_Handler(void)
{
    ++tick_ms;
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF11) {
        EXTI->PR1 = EXTI_PR1_PIF11;
        ++motor_closed_loop_fg_edges;
    }
}

static void delay_ms(uint32_t duration)
{
    /* Unsigned subtraction gives elapsed time even across counter wraparound.
     * Wait for an interrupt between checks instead of spinning the CPU. */
    uint32_t start = tick_ms;
    while (tick_ms - start < duration) {
        __WFI();
    }
}

static void init_motor_pins(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;

    /* Preload the outputs low before switching the pins to output mode.
     * PA0 drives SPEED for wake-up. PC6 holds the test direction at DIR=0. */
    GPIOA->BSRR = GPIO_BSRR_BR0;
    GPIOC->BSRR = GPIO_BSRR_BR6;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0) | GPIO_MODER_MODE0_0;
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODE6) | GPIO_MODER_MODE6_0;

    /* PA8 is SDA and PA9 is SCL. AF4 routes both to I2C2. Open-drain outputs
     * drive low or release the line so the board's external pull-ups can raise it. */
    GPIOA->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFU) | 0x44U;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9)) |
                   GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;
}

static void init_i2c(void)
{
    /* I2C2 kernel clock is HSI16. Standard-mode timing, roughly 90 kHz. */
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_I2C2SEL) | RCC_CCIPR_I2C2SEL_1;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    (void)RCC->APB1ENR1;
    /* The pull-ups use the driver's V3P3, which is off during sleep.
     * Keep I2C disabled until the released bus lines have risen. */
    I2C2->CR1 = 0;
    I2C2->TIMINGR = 0x30511216U;
}

static void arm_motor_fg_capture(void)
{
    /* FG_M1 is connected to PC11 and has an external pull-up. Arm falling
     * edges after wake and configuration, while the drive command is zero. */
    GPIOC->MODER &= ~GPIO_MODER_MODE11;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD11;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~SYSCFG_EXTICR3_EXTI11) |
                      SYSCFG_EXTICR3_EXTI11_PC;
    EXTI->RTSR1 &= ~EXTI_RTSR1_RT11;
    EXTI->FTSR1 |= EXTI_FTSR1_FT11;
    EXTI->PR1 = EXTI_PR1_PIF11;
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    EXTI->IMR1 |= EXTI_IMR1_IM11;
}

static void i2c_record_error(I2cError error, uint32_t waiting_for)
{
    /* Capture peripheral flags and pin levels before cleanup can change them. */
    uint32_t pins = GPIOA->IDR;
    i2c_diagnostic.isr = I2C2->ISR;
    i2c_diagnostic.waiting_for = waiting_for;
    i2c_diagnostic.scl_high = (pins & GPIO_IDR_ID9) != 0;
    i2c_diagnostic.sda_high = (pins & GPIO_IDR_ID8) != 0;
    i2c_diagnostic.speed_high = (pins & GPIO_IDR_ID0) != 0;
    i2c_diagnostic.error = error;
}

static bool i2c_wait(uint32_t flag, uint32_t start)
{
    /* Check for NACK, bus error, arbitration loss, or overrun before accepting
     * a ready flag. The caller passes the start time of the whole transfer. */
    const uint32_t errors = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;
    do {
        uint32_t status = I2C2->ISR;
        if (status & errors) {
            i2c_record_error(I2C_ERROR_PERIPHERAL, flag);
            return false;
        }
        if (status & flag) {
            return true;
        }
    } while (tick_ms - start < I2C_TIMEOUT_MS);
    i2c_record_error(I2C_ERROR_WAIT_TIMEOUT, flag);
    return false;
}

/* One short I2C transaction in a single direction. AUTOEND generates STOP
 * after NBYTES bytes have been transferred. */
static bool i2c_transfer(uint8_t *data, uint32_t count, bool read)
{
    uint32_t start = tick_ms;
    i2c_diagnostic.error = I2C_ERROR_NONE;
    i2c_diagnostic.read = read;
    i2c_diagnostic.byte_index = 0;
    if (!(I2C2->CR1 & I2C_CR1_PE)) {
        /* Before enabling I2C2, require both released lines to read high. */
        const uint32_t bus_pins = GPIO_IDR_ID8 | GPIO_IDR_ID9;
        if ((GPIOA->IDR & bus_pins) != bus_pins) {
            i2c_record_error(I2C_ERROR_LINES_LOW, 0);
            return false;
        }
        I2C2->CR1 = I2C_CR1_PE;
    }
    while (I2C2->ISR & I2C_ISR_BUSY) {
        if (tick_ms - start >= I2C_TIMEOUT_MS) {
            i2c_record_error(I2C_ERROR_BUS_BUSY, I2C_ISR_BUSY);
            return false;
        }
    }

    /* Clear stale flags and shift the 7-bit address into the STM32 SADD field. */
    I2C2->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF |
                I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    I2C2->CR2 = (DRV_ADDRESS << 1) | (count << I2C_CR2_NBYTES_Pos) |
                (read ? I2C_CR2_RD_WRN : 0U) | I2C_CR2_AUTOEND | I2C_CR2_START;

    for (uint32_t i = 0; i < count; ++i) {
        i2c_diagnostic.byte_index = i;
        /* RXNE means a byte is available. TXIS requests the next transmit byte. */
        if (!i2c_wait(read ? I2C_ISR_RXNE : I2C_ISR_TXIS, start)) {
            uint32_t status = I2C2->ISR;
            /* Try to release a busy bus if the MCU still owns the transaction.
             * Arbitration loss relinquishes ownership. AUTOEND handles NACK. */
            if ((status & I2C_ISR_BUSY) && !(status & (I2C_ISR_ARLO | I2C_ISR_NACKF))) {
                I2C2->CR2 |= I2C_CR2_STOP;
            }
            return false;
        }
        if (read) {
            data[i] = (uint8_t)I2C2->RXDR;
        } else {
            I2C2->TXDR = data[i];
        }
    }
    i2c_diagnostic.byte_index = count;
    return i2c_wait(I2C_ISR_STOPF, start);
}

static bool drv_write(uint8_t reg, uint8_t value)
{
    /* The first byte selects the register and the second byte is its new value. */
    uint8_t data[] = {reg, value};
    i2c_diagnostic.reg = reg;
    return i2c_transfer(data, sizeof(data), false);
}

static bool drv_read(uint8_t reg, uint8_t *value)
{
    /* TI SLVUB30A shows STOP after the register address, then START + read. */
    i2c_diagnostic.reg = reg;
    return i2c_transfer(&reg, 1, false) && i2c_transfer(value, 1, true);
}

static bool drv_update_register(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;
    uint8_t actual;
    if (!drv_read(reg, &current)) {
        return false;
    }

    /* A 1 in the mask selects a bit to replace. Readback verifies the write
     * and checks that the bits outside the mask kept their original values. */
    uint8_t expected = (current & (uint8_t)~mask) | (value & mask);
    return drv_write(reg, expected) &&
           drv_read(reg, &actual) &&
           actual == expected;
}

static bool drv_configure_motor(void)
{
    /* Apply this profile while the speed command is zero. Writes go to
     * volatile driver registers and are reapplied each time the firmware starts. */
    static const struct {
        uint8_t reg;
        uint8_t mask;
        uint8_t value;
    } settings[] = {
        /* 0x20: bit 7 (DoubleFreq) selects 25 kHz at 0 or 50 kHz at 1.
         * Bits 6:0 encode Rm. PWM frequency controls the switching pulses. */
        {DRV_MOTOR_RESISTANCE, 0xFF, MOTOR_RESISTANCE_FIELD},
        /* 0x21: Kt in bits 6:0. Preserve the existing adjustment mode in bit 7. */
        {DRV_MOTOR_BEMF, 0x7F, MOTOR_BEMF_FIELD},
        /* 0x22: CtrlAdvMd = 0 selects fixed time. TCtrlAdv = 0x49 sets
         * 2.5 us * (9 << 4) = 360 us of commutation advance. */
        {DRV_COMMUTATION_ADVANCE, 0xFF, 0x49},
        /* 0x23: disable initial speed detection for a start from rest. */
        {DRV_STARTUP_OPTIONS, DRV_INITIAL_SPEED_DETECT, 0x00},
        /* 0x24: bits 7:6 set 0.8 A, bits 5:3 set 0.7 VCC/s voltage ramp,
         * and bits 2:0 disable braking. The driver calculates startup voltage
         * from the current setting, Rm, and a speed-based BEMF term. Actual
         * winding current can differ from the setting. */
        {DRV_STARTUP_CURRENT, 0xFF, 0x98},
        /* 0x25: bits 7:6 set control coefficient 1. Bits 2:0 set
         * A1 = 0.9 Hz/s and bits 5:3 set A2 = 1.6 Hz/s^2.
         * After alignment, the driver advances the field at an electrical
         * frequency of A1*t + 0.5*A2*t^2, with t in seconds. The rotor must
         * follow that field for a successful open-loop start. */
        {DRV_STARTUP_ACCELERATION, 0xFF, 0xEE},
        /* 0x26: bits 2:0 set 5.3 s of alignment. Bits 7:3 set the handoff
         * threshold to 38.4 electrical Hz (384 RPM for six pole pairs).
         * The programmed ramp reaches this threshold about 6.4 s after alignment. */
        {DRV_ALIGNMENT, 0xFF, 0x90},
        /* 0x27: enable current, speed, Kt, and no-motor fault detection.
         * Disable inductive anti-voltage-surge (AVS) for the DRV10975Z. */
        {DRV_PROTECTION, 0xF8, 0xF0},
        /* 0x28: 1.2 A closed-loop acceleration limit and 1.6 A lock threshold. */
        {DRV_CURRENT_LIMITS, 0xFE, 0x66},
        /* 0x29: enable closed-loop stall detection, set 0.37 VCC/s closed-loop
         * ramp and 360 ns dead time between complementary switch transitions. */
        {DRV_CLOSED_LOOP_RAMP, 0xFF, 0xC8},
        /* 0x2A: IPD threshold zero selects align-and-go. Enable open-loop
         * stall detection and preserve the regulator voltage selection. */
        {DRV_REGULATOR, 0xF8, 0x08},
        /* 0x2B: bit 0 controls open-loop-only operation. Set FGOLsel[7:6]
         * to 01 so FG pulses mark closed-loop operation. Preserve bits 5:1. */
        {DRV_SPEED_MODE, 0xC1, 0x40 | TEST_OPEN_LOOP_ONLY},
    };
    uint8_t value;
    /* Save the whole register range before and after the verified writes. */
    for (uint32_t i = 0; i < DRV_CONFIG_COUNT; ++i) {
        if (!drv_read(DRV_MOTOR_RESISTANCE + i, &value)) {
            return false;
        }
        motor_config_before[i] = value;
    }
    for (uint32_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        if (!drv_update_register(settings[i].reg, settings[i].mask, settings[i].value)) {
            return false;
        }
    }
    for (uint32_t i = 0; i < DRV_CONFIG_COUNT; ++i) {
        if (!drv_read(DRV_MOTOR_RESISTANCE + i, &value)) {
            return false;
        }
        motor_config_after[i] = value;
    }
    return true;
}

static bool drv_read_motor_data(void)
{
    uint8_t high;
    uint8_t low;
    uint8_t supply;
    uint8_t command;
    uint8_t buffered_command;
    /* Reading each MSB first latches the corresponding LSB in the driver. */
    if (!drv_read(DRV_RPM_HIGH, &high) || !drv_read(DRV_RPM_LOW, &low)) {
        return false;
    }
    /* The register counts tenths of an electrical Hz. For six pole pairs,
     * RPM = (count / 10) * (60 / 6), so the numerical value equals RPM. */
    motor_rpm = ((uint16_t)high << 8) | low;
    if (!drv_read(DRV_CURRENT_HIGH, &high) || !drv_read(DRV_CURRENT_LOW, &low)) {
        return false;
    }
    uint16_t current = ((uint16_t)(high & 0x07U) << 8) | low;
    motor_phase_current_raw = current;
    if (current < motor_current_raw_min) {
        motor_current_raw_min = current;
    }
    if (current > motor_current_raw_max) {
        motor_current_raw_max = current;
    }
    /* Apply the conversion as written in DRV10975 Table 9, in milliamps.
     * The branch at 1023 causes a jump in the converted value. Check the
     * saved raw code alongside any surprising peak. */
    if (current >= 1023U) {
        current -= 1023U;
    }
    motor_phase_current_mA = (uint32_t)current * 3000U / 512U;
    if (motor_phase_current_mA > motor_peak_phase_current_mA) {
        motor_peak_phase_current_mA = motor_phase_current_mA;
        motor_current_raw_at_peak = motor_phase_current_raw;
    }
    if (!drv_read(DRV_SUPPLY_VOLTAGE, &supply) ||
        !drv_read(DRV_COMMAND_READBACK, &command) ||
        !drv_read(DRV_BUFFERED_COMMAND, &buffered_command)) {
        return false;
    }
    /* The supply register uses a scale of 22.8 V / 256 per count. */
    motor_supply_mV = (uint32_t)supply * 22800U / 256U;
    motor_speed_command = command;
    motor_buffered_command = buffered_command;
    return true;
}

static bool drv_set_speed(uint16_t command)
{
    /* Write the override flag and command bit 8 first. Writing bits 7:0
     * then applies the full command. Zero requests a stop. */
    return drv_write(DRV_SPEED_HIGH, DRV_OVERRIDE | (command >> 8)) &&
           drv_write(DRV_SPEED_LOW, (uint8_t)command);
}

static bool drv_take_control(void)
{
    /* Enable serial writes and keep the driver awake, then take control
     * with a zero speed command before changing its motor settings. */
    return drv_write(DRV_CONTROL, DRV_SLEEP_DISABLE | DRV_REGISTER_WRITE_ENABLE) &&
           drv_set_speed(0);
}

static bool drv_wake(void)
{
    /* An already-awake driver can accept the I2C command with SPEED low. */
    bool ready = drv_take_control();
    if (!ready) {
        /* Sleep disables I2C and its local pull-up supply. Raise SPEED to wake
         * the driver, then retry until I2C accepts the zero speed override. */
        GPIOA->BSRR = GPIO_BSRR_BS0;
        uint32_t start = tick_ms;
        do {
            ++motor_wake_attempts;
            ready = drv_take_control();
            if (!ready) {
                delay_ms(2);
            }
        } while (!ready && tick_ms - start < WAKE_TIMEOUT_MS);
        motor_wake_elapsed_ms = tick_ms - start;
    }

    /* Optional one-shot measurement breakpoint when a debugger is attached. */
    if (motor_pause_after_wake && (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)) {
        GPIOA->BSRR = GPIO_BSRR_BS0;
        motor_pause_after_wake = false;
        __BKPT(0);
        /* Measurements may outlast the wake retry window. Reset the MCU's
         * I2C state and retry with SPEED still high before returning. */
        init_i2c();
        ready = drv_take_control();
    }
    GPIOA->BSRR = GPIO_BSRR_BR0;
    return ready;
}

static MotorTestState run_motor(void)
{
    uint8_t value;
    /* Check the board's expected VregSel value before requesting motion. */
    if (!drv_wake() || !drv_read(DRV_REGULATOR, &value)) {
        return MOTOR_I2C_ERROR;
    }
    if (!(value & DRV_REGULATOR_3V3)) {
        return MOTOR_REGULATOR_MISMATCH;
    }

    /* Keep startup status and fault separate from the readings during motion.
     * Abort on an initial thermal or overcurrent indication. */
    if (!drv_read(DRV_STATUS, &value)) {
        return MOTOR_I2C_ERROR;
    }
    motor_initial_status = value;
    if (!drv_read(DRV_FAULT, &value)) {
        return MOTOR_I2C_ERROR;
    }
    motor_initial_fault = value;
    if (motor_initial_status & DRV_STATUS_THERMAL_OR_OVERCURRENT) {
        motor_status = motor_initial_status;
        motor_fault = motor_initial_fault;
        return MOTOR_DRIVER_FAULT;
    }

    if (!drv_configure_motor()) {
        return MOTOR_CONFIGURATION_ERROR;
    }

    if (motor_initial_status & DRV_STATUS_LOCK) {
        /* The driver retains lock status until restart and has a 5 s retry delay. */
        motor_state = MOTOR_WAITING_FOR_DRIVER;
        delay_ms(DRIVER_LOCK_RECOVERY_MS);
    } else {
        delay_ms(1000);
    }
    if (motor_stop_requested) {
        return MOTOR_STOP_REQUESTED;
    }
    arm_motor_fg_capture();
    if (!drv_set_speed(TEST_COMMAND)) {
        return MOTOR_I2C_ERROR;
    }
    motor_state = MOTOR_RUNNING;

    /* Allow the driver to process restart before checking retained lock status.
     * While the MCU is halted, the DRV10975Z keeps its last speed command. */
    delay_ms(20);
    while (!motor_stop_requested) {
        if (!drv_read(DRV_STATUS, &value)) {
            return MOTOR_I2C_ERROR;
        }
        motor_status = value;
        if (value & DRV_STATUS_ERROR) {
            if (drv_read(DRV_FAULT, &value)) {
                motor_fault = value;
            }
            return MOTOR_DRIVER_FAULT;
        }
        if (!drv_read_motor_data()) {
            return MOTOR_I2C_ERROR;
        }
        if (motor_rpm >= TEST_MAX_RPM) {
            return MOTOR_OVERSPEED;
        }
        delay_ms(20); /* Pause between polls. I2C reads add to this interval. */
    }
    return MOTOR_STOP_REQUESTED;
}

int main(void)
{
    /* CMSIS startup leaves the STM32G431 running from its reset clock, HSI16. */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);
    init_motor_pins();
    init_i2c();

    /* Use one exit path for stop requests and errors. A stop attempt updates the I2C
     * diagnostics, so save the original failure before sending the command. */
    MotorTestState result = run_motor();
    EXTI->IMR1 &= ~EXTI_IMR1_IM11; /* Freeze the FG count before stopping. */
    if (i2c_diagnostic.error != I2C_ERROR_NONE) {
        motor_i2c_failure = i2c_diagnostic;
    }
    motor_stop_acknowledged = drv_set_speed(0);
    motor_state = result;

    /* Run once per reset. The retained debugger values describe this run. */
    for (;;) {
        __WFI();
    }
}
