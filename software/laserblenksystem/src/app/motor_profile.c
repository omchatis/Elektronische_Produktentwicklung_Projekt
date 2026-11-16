#include "app/motor_profile.h"

enum
{
    MOTOR1_COMMAND = 80, /* I2C drive command on the 0 to 511 scale. */
    MOTOR1_MAX_RPM = 10000, /* Firmware stop threshold: 1000 electrical Hz with six pole pairs. */
    MOTOR1_POLE_PAIRS = 6,
    MOTOR1_OPEN_LOOP_ONLY = 0, /* Set to 1 to prevent closed-loop handoff. */

    /* The corrected phase-to-phase measurement was 0.482 ohm. Half of that
     * gives 0.241 ohm phase resistance. Code 0x28 represents 0.2352 ohm. */
    MOTOR1_RESISTANCE_FIELD = 0x28,
    /* Phase-to-phase BEMF measured 1.3 V peak-to-peak at an 8 ms period.
     * (1.3 V / 2) * 8 ms is 5.2 mV/Hz. Code 0x07 represents 4.85 mV/Hz. */
    MOTOR1_BEMF_FIELD = 0x07,
    MOTOR1_HALF_CYCLE_ADJUSTMENT = 0x80
};

/* Mask bits select which parts of a register the driver changes. */
static const drv10975_setting_t motor1_settings[] = {
    /* 0x20: bit 7 (DoubleFreq) selects 25 kHz at 0 or 50 kHz at 1.
     * Bits 6:0 encode Rm. PWM frequency controls the switching pulses. */
    {DRV10975_MOTOR_RESISTANCE, 0xFF, MOTOR1_RESISTANCE_FIELD},
    /* 0x21: Bits 6:0 hold the programmed Kt. Bit 7 enables two driver
     * adjustments per electrical cycle to help track BEMF at handoff. */
    {DRV10975_MOTOR_BEMF, 0xFF, MOTOR1_HALF_CYCLE_ADJUSTMENT | MOTOR1_BEMF_FIELD},
    /* 0x22: CtrlAdvMd = 0 selects fixed time. TCtrlAdv = 0x49 sets
     * 2.5 us * (9 << 4) = 360 us of commutation advance. */
    {DRV10975_COMMUTATION_ADVANCE, 0xFF, 0x49},
    /* 0x23: disable initial speed detection for a start from rest. */
    {DRV10975_STARTUP_OPTIONS, DRV10975_INITIAL_SPEED_DETECT, 0x00},
    /* 0x24: bits 7:6 set 0.8 A, bits 5:3 set 0.7 VCC/s voltage ramp,
     * and bits 2:0 disable braking. The driver calculates startup voltage
     * from the current setting, Rm, and a speed-based BEMF term. Actual
     * winding current can differ from the setting. */
    {DRV10975_STARTUP_CURRENT, 0xFF, 0x98},
    /* 0x25: bits 7:6 set control coefficient 1. Bits 2:0 set
     * A1 = 0.9 Hz/s and bits 5:3 set A2 = 0.66 Hz/s^2.
     * After alignment, the driver advances the field at an electrical
     * frequency of A1*t + 0.5*A2*t^2, with t in seconds. The rotor must
     * follow that field for a successful open-loop start. */
    {DRV10975_STARTUP_ACCELERATION, 0xFF, 0xF6},
    /* 0x26: bits 2:0 set 5.3 s of alignment. Bits 7:3 set the handoff
     * threshold to 51.2 electrical Hz (512 RPM for six pole pairs).
     * The open-loop ramp equation predicts about 11.2 s after alignment. */
    {DRV10975_ALIGNMENT, 0xFF, 0x98},
    /* 0x27: enable current, speed, Kt, and no-motor fault detection.
     * Disable inductive anti-voltage-surge (AVS) for the DRV10975Z. */
    {DRV10975_PROTECTION, 0xF8, 0xF0},
    /* 0x28: 1.2 A closed-loop acceleration limit and 1.6 A lock threshold. */
    {DRV10975_CURRENT_LIMITS, 0xFE, 0x66},
    /* 0x29: enable closed-loop stall detection, set 0.19 VCC/s closed-loop
     * ramp and 360 ns dead time between complementary switch transitions. */
    {DRV10975_CLOSED_LOOP_RAMP, 0xFF, 0xD8},
    /* 0x2A: IPD threshold zero selects align-and-go. Enable open-loop
     * stall detection and preserve the regulator voltage selection. */
    {DRV10975_REGULATOR, 0xF8, 0x08},
    /* 0x2B: bit 0 controls open-loop-only operation. Set FGOLsel[7:6]
     * to 01 so FG pulses mark closed-loop operation. Preserve bits 5:1. */
    {DRV10975_SPEED_MODE, 0xC1, 0x40 | MOTOR1_OPEN_LOOP_ONLY},
};

const motor_profile_t motor1_profile = {
    .settings = motor1_settings,
    .settings_count = sizeof(motor1_settings) / sizeof(motor1_settings[0]),
    .speed_command = MOTOR1_COMMAND,
    .max_rpm = MOTOR1_MAX_RPM,
    .pole_pairs = MOTOR1_POLE_PAIRS,
};
