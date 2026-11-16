#include "app/motor_profile.h"

const motor_profile_t motor1_profile = {
    .speed_command = 80, /* I2C drive command on the 0 to 511 scale. */
    .max_rpm = 10000, /* Firmware stop threshold in mechanical RPM. */
    .pole_pairs = 6,

    /* Set each field to a datasheet code or DRV10975_KEEP.
     * KEEP uses whatever the driver already has loaded for that field. */
    .registers = {
        .MotorParam1 = { /* 0x20 */
            .DoubleFreq = 0b0, /* PWM output frequency, 25 kHz. */
            .Rm = 0b0101000, /* Phase resistance, 0.2352 ohm. Measured 0.482 ohm phase to phase. */
        },
        .MotorParam2 = { /* 0x21 */
            .AdjMode = 0b1, /* Half-cycle adjustment for BEMF tracking at handoff. */
            .Kt = 0b0000111, /* BEMF constant, 4.85 mV/Hz. Measured about 5.2 mV/Hz. */
        },
        .MotorParam3 = { /* 0x22 */
            .CtrlAdvMd = 0b0, /* Fixed commutation advance time. */
            .TCtrlAdv = 0b1001001, /* Commutation advance, 360 us. */
        },
        .SysOpt1 = { /* 0x23 */
            .ISDThr = DRV10975_KEEP, /* Initial speed detection threshold. */
            .IPDAdvcAgl = DRV10975_KEEP, /* Initial position detection advance angle. */
            .ISDen = 0b0, /* Disable initial speed detection for a start from rest. */
            .RvsDrEn = DRV10975_KEEP, /* Reverse drive enable. */
            .RvsDrThr = DRV10975_KEEP, /* Reverse drive speed threshold. */
        },
        .SysOpt2 = { /* 0x24 */
            .OpenLCurr = 0b10, /* Open-loop current setting, 0.8 A. */
            .OpLCurrRt = 0b011, /* Open-loop drive voltage ramp, 0.7 VCC/s. */
            .BrkDoneThr = 0b000, /* Startup braking disabled. */
        },
        .SysOpt3 = { /* 0x25 */
            .CtrlCoef = 0b11, /* Closed-loop control coefficient, 1. */
            .StAccel2 = 0b110, /* Second-order open-loop acceleration, 0.66 Hz/s^2. */
            .StAccel = 0b110, /* First-order open-loop acceleration, 0.9 Hz/s. */
        },
        .SysOpt4 = { /* 0x26 */
            .Op2ClsThr = 0b10011, /* Handoff threshold, 51.2 electrical Hz (512 RPM with six pole pairs). */
            .AlignTime = 0b000, /* Rotor alignment time, 5.3 s. */
        },
        .SysOpt5 = { /* 0x27 */
            .LockEn3 = 0b1, /* Enable no-motor detection (LockEn[3]). */
            .LockEn2 = 0b1, /* Enable abnormal Kt detection (LockEn[2]). */
            .LockEn1 = 0b1, /* Enable abnormal speed detection (LockEn[1]). */
            .LockEn0 = 0b1, /* Enable lock current detection (LockEn[0]). */
            .AVSIndEn = 0b0, /* Disable unsupported inductive anti-voltage-surge protection. */
            .AVSMEn = DRV10975_KEEP, /* Mechanical anti-voltage-surge enable. */
            .AVSMMd = DRV10975_KEEP, /* Mechanical anti-voltage-surge voltage limit selection. */
            .IPDRlsMd = DRV10975_KEEP, /* Initial position detection release mode. */
        },
        .SysOpt6 = { /* 0x28 */
            .SWiLimitThr = 0b0110, /* Closed-loop acceleration current limit, 1.2 A. */
            .HWiLimitThr = 0b011, /* Lock detection current limit, 1.6 A. */
        },
        .SysOpt7 = { /* 0x29 */
            .LockEn5 = 0b1, /* Enable closed-loop stall detection (LockEn[5]). */
            .ClsLpAccel = 0b101, /* Closed-loop acceleration, 0.19 VCC/s. */
            .Deadtime = 0b1000, /* Output switch dead time, 360 ns. */
        },
        .SysOpt8 = { /* 0x2A */
            .IPDCurrThr = 0b0000, /* Disable initial position detection to use align-and-go. */
            .LockEn4 = 0b1, /* Enable open-loop stall detection (LockEn[4]). */
            .VregSel = DRV10975_KEEP, /* Regulator voltage selection. This board requires 3.3 V. */
            .IPDClk = DRV10975_KEEP, /* Initial position detection clock. */
        },
        .SysOpt9 = { /* 0x2B */
            .FGOLsel = 0b01, /* Output FG pulses only in closed loop. */
            .FGcycle = DRV10975_KEEP, /* FG pulses per electrical cycle. */
            .KtLckThr = DRV10975_KEEP, /* Abnormal Kt limits. */
            .SpdCtrlMd = DRV10975_KEEP, /* Analog or PWM mode for the SPEED pin. */
            .CLoopDis = 0b0, /* Allow the open-to-closed-loop handoff. */
        },
    },
};
