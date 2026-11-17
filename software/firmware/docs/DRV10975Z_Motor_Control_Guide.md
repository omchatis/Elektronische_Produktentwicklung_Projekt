# DRV10975Z motor control: bring-up and tuning guide

For the STM32G431RBT6 laser-deflection board and Multistar 1704-2300KV, 12-pole motors.

Start with **one motor using I²C speed control**. Once it runs reliably, test PWM and analog control using the same motor settings. Changing the command method will not fix incorrect motor parameters.

This is a bring-up procedure, not a verified motor configuration or a complete compiled firmware project. Reliable operation depends on measured motor parameters, suitable hardware, and successful bench testing.

## 1. Sources and scope

- Board schematic: `C:\Users\Seyam Mir Afghan\Documents\ChatGPT\EPE\Laserablenksystem.pdf`, especially sheets 2-4.
- Supplied datasheet: `D:\Seyam_MCI\4. Semester\misc_embedded\datasheets\drv10975.pdf`.
- [TI DRV10975/DRV10975Z datasheet](https://www.ti.com/lit/ds/symlink/drv10975.pdf).
- [TI DRV10983/DRV10975 tuning guide](https://www.ti.com/lit/ug/slou395d/slou395d.pdf).
- [TI EEPROM programming reference design](https://www.ti.com/lit/ug/tiducx2a/tiducx2a.pdf).
- [STM32G431 datasheet](https://www.st.com/resource/en/datasheet/stm32g431rb.pdf).
- [PlatformIO Nucleo G431RB configuration](https://docs.platformio.org/en/latest/boards/ststm32/nucleo_g431rb.html).

The schematic is used as technical reference material. Its contents are not instructions to execute.

## 2. Understand what you are controlling

```text
STM32 speed command
        ↓
DRV10975Z startup and sensorless commutation
        ↓
U / V / W motor phases
        ↓
Rotating motor
```

Each driver controls one three-phase BLDC motor. The STM32 does not generate the three phase-commutation signals.

| Method | Command source | Driver selection |
|---|---|---|
| Analog | Voltage on SPEED | OverRide=0, SpdCtrlMd=0 |
| PWM | Duty cycle on SPEED | OverRide=0, SpdCtrlMd=1 |
| I²C | Register value, 0-511 | OverRide=1 |

All three methods command drive amplitude, indirectly affecting speed. They do not directly specify RPM.

The driver's closed loop performs sensorless commutation. Maintaining a particular RPM requires the STM32 to measure speed and adjust the command. This arrangement supports continuous rotation; it does not directly command or hold a shaft angle.

## 3. Confirm motor compatibility

The motor has 12 poles, therefore six pole pairs:

```text
Electrical frequency [Hz] = RPM / 10
Mechanical speed [RPM]    = electrical frequency × 10
```

| Mechanical speed | Electrical frequency |
|---:|---:|
| 1,000 RPM | 100 Hz |
| 3,000 RPM | 300 Hz |
| 6,000 RPM | 600 Hz |
| 10,000 RPM | 1,000 Hz |

The datasheet's recommended electrical-frequency range ends at 1000 Hz, equivalent to 10,000 RPM for this motor.

Relevant driver/application limits:

- 1.5 A continuous winding current, 2 A peak.
- Recommended phase-to-phase resistance: 0.6-24 Ω.
- Recommended phase-to-phase inductance: at least 0.07 mH.
- Normal driver supply: 6.5-18 V.

The motor's 2300 KV means approximately 2300 RPM per volt at no load. At 10 V, the theoretical no-load speed is approximately 23,000 RPM, above the recommended operating range for this driver and pole count.

Compatibility remains unconfirmed until resistance, inductance, startup current, and loaded operation are checked. Software cannot guarantee operation outside the recommended hardware conditions.

## 4. Prepare the first test

Equipment:

- Current-limited bench supply.
- Multimeter.
- Oscilloscope or logic analyzer.
- ST-Link.
- Ideally an LCR meter and current probe.

Procedure:

1. Disconnect the laser.
2. Remove propellers and the mirror/load.
3. Secure the motor body.
4. Test only Motor 1.
5. Keep a way to remove motor power immediately.
6. Never connect or disconnect phase wires while powered.

Bench-supply current limiting is useful, but DC supply current is not the same as winding current. Configure the driver's current limits as well.

## 5. Verify the PCB

| Signal | Motor 1 | Motor 2 |
|---|---|---|
| SPEED | PA0 | PA1 |
| DIR | PC6 | PC7 |
| FG | PC11 | PC12 |
| SCL | PA9 | PC8 |
| SDA | PA8 | PC9 |

Both drivers use fixed 7-bit I²C address **0x52**. Keep their buses separate unless a means of isolating the devices is added.

The schematic includes approximately 4.75 kΩ pullups for SDA, SCL, and FG to each driver's local 3.3 V supply.

Check:

- U, V, and W motor connections.
- Both driver VCC pins connected to the 10 V rail.
- GND, PGND, SWGND, and exposed pad properly grounded.
- Charge-pump capacitors and their connections.
- V1P8 and V3P3 capacitors.
- Regulator resistor connections.
- Solder bridges and exposed-pad soldering.

Two parallel 78.7 Ω regulator resistors give approximately 39.35 Ω, close to TI's 39 Ω recommendation.

**Board-specific regulator check:** D6/D7 are approximately 3.9 V Zeners, associated with the 3.3 V VREG configuration. Keep `VregSel=1` unless the regulator/Zener design is changed and verified. Verify the diode's pulse-power and leakage specifications against TI's requirements; nominal voltage alone is insufficient.

Do not power the STM32 from the driver's V3P3 output. Use the board's MCU supply.

## 6. Measure motor parameters

Disconnect the motor from the driver before these measurements.

### Resistance

Measure U-V, V-W, and W-U. Readings should be similar. Compensate for lead resistance, preferably using a four-wire measurement.

```text
Rphase = Rphase-to-phase / 2
```

Example only:

```text
Measured phase-to-phase resistance = 0.80 Ω
Equivalent phase resistance       = 0.40 Ω
```

Do not enter advertised internal resistance without knowing its measurement convention.

### Inductance

Measure phase-to-phase inductance. Record measurement frequency and rotor position, and check multiple positions. Below 0.07 mH is outside the datasheet's recommended application range.

### BEMF constant

Spin the electrically disconnected motor externally and measure phase-to-phase generated voltage:

```text
Ep = half the peak-to-peak voltage
Te = electrical waveform period in seconds
Kt = Ep × Te   [V/Hz]
```

Use an appropriate differential or isolated measurement arrangement. Never connect an earth-grounded oscilloscope ground clip to a driven motor phase.

A rough initial estimate from nominal KV is:

```text
Kt ≈ 60 / (KV × pole pairs)
   ≈ 60 / (2300 × 6)
   ≈ 0.00435 V/Hz
```

This estimate is not a validated setting. Follow TI's tuning guide for measurement and validation.

## 7. Create the PlatformIO project

```ini
[env:laser_g431]
platform = ststm32
board = nucleo_g431rb
framework = cmsis

upload_protocol = stlink
debug_tool = stlink
```

The Nucleo board definition supplies the MCU/build configuration. Your custom PCB still needs its own pin and clock initialization.

Start with a simple internal-clock configuration. Verify:

1. Firmware builds.
2. ST-Link connects.
3. A breakpoint in `main()` works.
4. A known GPIO changes state.
5. Millisecond timing is correct.

CMSIS provides register definitions and core support; GPIO, timers, and I²C still require initialization.

## 8. Implement and verify I²C

Begin at approximately 100 kHz. Use hardware I²C on verified alternate functions, or software I²C on the existing GPIO pins. Check the exact STM32G431 pin mappings; do not copy STM32F401 alternate-function numbers.

For software I²C, use open-drain SDA and SCL:

```text
LOW  → drive low
HIGH → release the line; pullup raises it
```

Do not drive either line push-pull high. Use bounded waits for released lines to rise, ACK checks, and timeouts.

Implement these transport functions for the selected motor bus:

```c
bool drv_read(uint8_t reg, uint8_t *value);
bool drv_write(uint8_t reg, uint8_t value);
```

Return failure on NACK or timeout.

Write transaction:

```text
START → 0xA4 → register → data → STOP
```

Read transaction:

```text
START → 0xA4 → register → REPEATED START → 0xA5
      → read one byte → NACK → STOP
```

The address is 0x52 in 7-bit form; on-wire address bytes are 0xA4 for write and 0xA5 for read. Check whether your API expects shifted or unshifted addressing.

Verify waveforms and ACKs before motor testing.

## 9. Handle DRV10975Z sleep and wake-up

On the Z version:

- Sleep disables I²C.
- Configuration not stored in EEPROM is lost.
- I²C cannot wake a sleeping driver.
- SPEED must wake it.
- `sleepDis=1` prevents subsequent sleep, but does not wake an already sleeping device.

First communication test, with the motor disconnected:

1. Power the board normally.
2. Drive SPEED to 3.3 V.
3. Allow initialization; approximately 400 ms is a reasonable initial bench delay, then use bounded ACK retries.
4. Read register 0x03.
5. Write 0xC0 to 0x03: sleepDis=1, SIdata=1, EEPROM command bits zero.
6. Acquire zero-speed I²C override:

   ```text
   0x01 ← 0x80
   0x00 ← 0x00
   ```

7. Set SPEED low.
8. Confirm continued ACKs and read back 0x03.

**With the motor connected, holding SPEED high can request motion before firmware takes control.** Characterize the sequence unloaded. For normal startup, acquire zero-command override promptly, with bounded retries and motor-power shutdown on failure. Do not assume the bench delay is a motion-free interval.

If unintended startup motion is unacceptable, add a hardware means of preventing it.

Repeat initialization after power-up and after any sleep event that loses volatile configuration.

## 10. Load motor configuration into RAM

Use firmware-loaded settings initially, not EEPROM programming.

| Address | Purpose |
|---|---|
| 0x20 | Resistance and internal motor PWM frequency |
| 0x21 | BEMF constant and adjustment mode |
| 0x22 | Commutation advance |
| 0x23 | Initial-speed and reverse-drive settings |
| 0x24 | Open-loop current and current ramp |
| 0x25 | Startup acceleration |
| 0x26 | Alignment time and handoff frequency |
| 0x27 | Protection and AVS |
| 0x28 | Current limits |
| 0x29 | Closed-loop acceleration and dead time |
| 0x2A | IPD, protection, regulator voltage |
| 0x2B | FG, input mode, closed-loop enable |

Resistance encoding:

```text
Rphase ≈ 0.00735 × mantissa × 2^exponent
Field  = (exponent << 4) | mantissa
```

BEMF encoding:

```text
Kt    ≈ mantissa × 2^exponent / 1442
Field = (exponent << 4) | mantissa
```

Use the nearest valid datasheet table value. Preserve unrelated bits, including bit 7 of 0x20/0x21.

Initial tuning choices:

- Internal motor PWM: 25 kHz, DoubleFreq=0.
- Regulator selection compatible with the Zener circuit.
- Current and lock protections enabled.
- Conservative startup acceleration.
- Align-and-go initially if appropriate.
- Reverse-drive behavior deferred until ordinary startup works.

Possible supervised bench starting points, subject to measured current:

| Setting | Initial trial |
|---|---|
| Open-loop current | 0.2 A; increase to 0.4 A only if needed |
| Acceleration current limit | 0.6 A |
| Lock-detection current threshold | 0.8 A |
| Initial command | Modest, increased gradually |

For the two current-limit fields in 0x28:

```text
SWiLimitThr = 3 → 0.6 A acceleration limit
HWiLimitThr = 1 → 0.8 A lock threshold
```

Preserve unrelated bits. These limits do not guarantee that every transient remains below the threshold.

Read back every configuration register. Do not proceed if values differ.

## 11. Tune startup using I²C

This helper assumes the transport functions above are implemented and validated. It is not a complete firmware application.

```c
bool drv_set_i2c_speed(unsigned command)
{
    if (command > 511U)
        command = 511U;

    /* High bit first; enable I2C override. */
    if (!drv_write(0x01,
                   0x80U | ((command >> 8) & 1U)))
        return false;

    /* Low byte applies the command. */
    return drv_write(0x00, command & 0xFFU);
}
```

Procedure:

1. Set zero command.
2. Configure measured motor and startup parameters.
3. For a short supervised tuning test, set CLoopDis=1 in 0x2B.
4. Apply a modest command.
5. Observe alignment and open-loop rotation.
6. Stop and adjust current/acceleration if the motor twitches or loses synchronization.
7. Once open-loop startup works, stop the motor.
8. Set CLoopDis=0.
9. Restart and observe transfer to sensorless closed-loop commutation.
10. Adjust BEMF constant, handoff threshold, and startup acceleration as required.

Open-loop-only operation is a tuning step, not the final operating mode. Change one setting at a time and log results.

There is no universal validated startup register image for this motor without measurements and observed behavior.

## 12. Normal I²C speed control

Keep sleepDis=1, OverRide=1, and SPEED at a known level, normally low.

| Command | Approximate command percentage |
|---:|---:|
| 0 | 0% |
| 51 | 10% |
| 102 | 20% |
| 153 | 30% |
| 256 | 50% |
| 511 | 100% |

```c
if (!drv_set_i2c_speed(102)) {
    /* Enter the application's communication-failure shutdown path. */
}
```

Increase gradually while monitoring speed and current. Do not begin with 511: this motor can exceed the driver's recommended electrical-speed range.

To stop, request zero. A zero command does not guarantee immediate mechanical stopping. Confirm zero measured speed before reversing DIR.

## 13. Direct PWM control

After successful I²C testing:

1. Stop the motor.
2. Prepare the SPEED PWM output at zero duty.
3. Set SpdCtrlMd=1 in 0x2B.
4. Clear OverRide, bit 7 of 0x01.
5. Keep sleepDis=1 during testing.
6. Increase duty gradually.

Use 20 kHz initially. Permitted SPEED input frequency is 1-100 kHz.

```text
PA0 → TIM2_CH1 → Motor 1 SPEED
PA1 → TIM2_CH2 → Motor 2 SPEED
```

Verify alternate-function selection and the actual timer clock.

```text
PWM frequency = timer_clock / ((PSC + 1) × (ARR + 1))
```

If TIM2 receives 16 MHz:

```text
PSC=0, ARR=799 → 20 kHz

  0% → CCR=0
 10% → CCR=80
 20% → CCR=160
 50% → CCR=400
100% → CCR=800
```

CMSIS configuration fragment:

```c
/* Assumes TIM2 reset, clock enabled, 16 MHz timer clock,
 * and correct GPIO alternate-function configuration. */
TIM2->PSC = 0;
TIM2->ARR = 799;
TIM2->CCR1 = 0;
TIM2->CCR2 = 0;

TIM2->CCMR1 =
    (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
    (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
TIM2->CR1 = TIM_CR1_ARPE;
TIM2->EGR = TIM_EGR_UG;
TIM2->CR1 |= TIM_CR1_CEN;
```

This fragment has not been compiled or tested on the board.

Check approximately 0-3.3 V amplitude, frequency, duty, and startup pulses with an oscilloscope.

The external command PWM frequency is independent of internal 25/50 kHz motor phase switching. This is duty-cycle control, not a hobby ESC's 1-2 ms servo pulse.

## 14. Analog control

Stop before selecting analog control:

```text
SpdCtrlMd=0
OverRide=0
sleepDis=1
```

Approximate input behavior:

```text
Below 0.10 V               → zero command
0.10 V to 0.9 × V3P3       → proportional command
Above 0.9 × V3P3           → maximum command
```

At V3P3=3.3 V, full scale is approximately 2.97 V:

```text
Command fraction = (VSPEED - 0.10) / (2.97 - 0.10)
Clamp between 0 and 1.
```

| Command | Approximate SPEED voltage |
|---:|---:|
| Stop | 0 V |
| 10% | 0.387 V |
| 20% | 0.674 V |
| 30% | 0.961 V |
| 50% | 1.535 V |
| 100% | 2.97 V |

### Option A: external adjustable voltage

1. Set the STM32 SPEED GPIO to high impedance.
2. Connect an adjustable 0-3.3 V source and common ground.
3. Start at zero and increase slowly.

Do not connect an external voltage source while the STM32 actively drives the same net.

### Option B: DAC

Wire a supported DAC output to SPEED. PA0/PA1 cannot simply be assumed to be DAC outputs. Check the MCU pinout and modify wiring if required.

### Option C: filtered PWM

Add an RC filter:

```text
STM32 PWM ── 1 kΩ ──┬── DRV SPEED
                     |
                    1 µF
                     |
                    GND
```

At 20 kHz PWM, this gives approximately a 1 ms time constant. Keep the driver in analog mode and measure the actual voltage; loading and leakage affect it.

```text
Direct PWM mode → driver measures duty cycle
Analog mode     → driver measures voltage
```

## 15. Measure actual speed

Read 0x11 first, then 0x12:

```text
raw = (high << 8) | low
Electrical frequency = raw / 10 Hz
```

For this six-pole-pair motor:

```text
RPM = electrical frequency × 10 = raw
```

A raw value of 3000 therefore corresponds to approximately 3000 RPM.

Alternatively, with FGcycle=00, FG outputs one pulse per electrical cycle:

```text
RPM = FG frequency × 10
300 Hz FG → 3000 RPM
```

Count one edge per pulse, or compensate if counting both edges. PC11/PC12 can be used as interrupt inputs for FG pulse counting.

## 16. Add fault handling

| Register | Meaning |
|---|---|
| 0x10 | Status |
| 0x1E | Fault code |
| 0x1B | Input command readback |
| 0x1C | Buffered/applied command readback |
| 0x11-0x12 | Speed |

Status:

```text
0x10 bit 7 → overtemperature
0x10 bit 6 → sleep/standby indication
0x10 bit 5 → overcurrent indication; cleared on read
0x10 bit 4 → motor lock
```

Fault code:

```text
0x1E bit 0 → lock-detection current limit
0x1E bit 1 → abnormal speed
0x1E bit 2 → abnormal BEMF constant
0x1E bit 3 → no motor
0x1E bit 4 → stuck in open loop
0x1E bit 5 → stuck in closed loop
```

On serious fault:

1. Request zero command.
2. Log status, fault code, command, and speed.
3. Stop increasing the command.
4. Remove motor power if communication or stopping fails.
5. Diagnose before retrying.

The driver may automatically retry locked-motor starts after approximately five seconds. Do not assume a faulted motor will remain stationary indefinitely.

## 17. Validate repeatability and add the load

Test:

1. Starts from different shaft positions.
2. Intended supply range.
3. Both intended directions.
4. Cold and warm operation.
5. Minimum and maximum intended speeds.
6. Stop/restart.
7. MCU reset with motor power still present.
8. Power interruption and recovery.
9. Lost I²C communication.

Then attach and balance the mirror/load and repeat tests.

Only then enable Motor 2. The schematic marks the shared 10 V supply as 3 A maximum. Check voltage droop and heating with both motors running.

## 18. Add an RPM controller if needed

```text
Requested RPM
      ↓
STM32 PI controller ← measured RPM
      ↓
Limited command
      ↓
I²C / PWM / analog
      ↓
DRV10975Z and motor
```

An initial update interval of 20-50 ms can be used for experiments, then tuned for the system.

Include command limits, acceleration limits, integral anti-windup, a separate startup state, overspeed shutdown, and fault shutdown.

The same controller can output I²C commands, PWM duty, or analog voltage.

## 19. Store validated settings in firmware

Store the validated 0x20-0x2B image in STM32 flash. Reload it after driver power-up or sleep that loses configuration.

Do not repeatedly program EEPROM.

EEPROM programming requires at least 22 V at driver VCC, unlike normal 6.5-18 V operation. This requires a controlled programming arrangement.

**Do not connect the board's 24 V input directly to the 10 V driver rail.** Driver absolute maximum VCC is 23 V, and other components may have lower limits. Follow TI's programming reference design and review the entire programming power path first.

## 20. Troubleshooting

| Symptom | Check first |
|---|---|
| No I²C ACK | Sleep, missing local 3.3 V, address convention, SDA/SCL mapping |
| Writes ACK but do nothing | SIdata=0, wrong register/bit, readback mismatch |
| Settings disappear | Sleep or power cycle |
| PWM has no effect | OverRide=1, analog mode, wrong alternate function |
| Analog has no effect | OverRide=1, PWM mode, actual input voltage |
| Motor twitches | Phase wiring, startup current, alignment, acceleration |
| Starts then stops | Handoff, BEMF parameter, lock/current fault |
| Current rises sharply | Lost synchronization, excessive load, unsuitable motor |
| Speed differs from command percentage | Percentage is not an RPM setpoint |
| Both motors fail together | Shared rail droop or current limit |

## 21. Completion checklist

- [ ] Resistance and inductance measured and evaluated.
- [ ] BEMF constant measured or estimated, then validated.
- [ ] Regulator/Zener design verified.
- [ ] MCU programming and timing verified.
- [ ] I²C wake, ACK, write, and readback repeatable.
- [ ] sleepDis and volatile-configuration reload handled.
- [ ] Current limits and protections configured.
- [ ] Open-loop startup characterized.
- [ ] Closed-loop handoff reliable.
- [ ] I²C speed control validated.
- [ ] PWM speed control validated.
- [ ] Analog speed control validated.
- [ ] Speed measurement and overspeed handling validated.
- [ ] Stop, direction change, reset, and fault behavior validated.
- [ ] Loaded operation and two-motor supply behavior validated.
- [ ] Final configuration saved in STM32 firmware.
