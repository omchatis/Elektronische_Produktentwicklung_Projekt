# Laser deflection system firmware

PlatformIO builds this C firmware for the STM32G431 using CMSIS. The
`laser_g431` environment currently starts Motor 1.

```sh
pio run -e laser_g431
```

## Where to read and change the code

| File | Responsibility |
| --- | --- |
| `src/main.c` | Initialize the application, service it, and wait for interrupts. |
| `src/app/application.c` | Create the active motor instance and connect its board, profile, and debugger state. |
| `src/app/motor_control.c` | Sequence wake, configuration, startup, monitoring, and stop for one motor. |
| `src/app/motor_profile.c` | Set Motor 1's command, speed ceiling, pole pairs, and DRV10975 register values. |
| `src/drivers/drv10975.c` | Send chip commands, verify configuration, and decode telemetry. |
| `src/drivers/stm32_i2c.c` | Transfer bytes to a 7-bit address with a timeout for the selected bus. |
| `src/board/motor_board.c` | Configure I²C2, SPEED, DIR, and FG pins. Count FG edges in the EXTI interrupt. |
| `src/board/timebase.c` | Provide the millisecond clock used for deadlines. |

After reset, the motor service wakes the driver and sets its command to zero.
It checks status, writes the profile, then sends the drive command. The
DRV10975 handles alignment, open-loop acceleration, and closed-loop handoff.
The STM32 continues sampling status and telemetry until a stop request,
reported overspeed, or failure. The wake retry delay, lock recovery wait,
and poll interval use timestamps so other device services can run between
steps. Each I²C transaction is synchronous and bounded.

## Debugger watches

Expand `motor1_debug` in the debugger. The previous standalone watches now
have these paths.

| Watch | Meaning |
| --- | --- |
| `motor1_debug.state`, `.fault` | Run state and the driver's fault code. |
| `motor1_debug.rpm` | Driver speed readback converted using the profile's pole-pair count. In open loop, check actual shaft motion separately. |
| `motor1_debug.closed_loop_fg_edges` | Falling FG edges recorded by the board and copied at each poll or stop. Repeated edges indicate that the driver entered closed loop. |
| `motor1_debug.phase_current_ma`, `.peak_phase_current_ma`, `.current_raw_at_peak` | The last complete sample, highest sampled conversion, and the raw code associated with the peak. |
| `motor1_debug.config.before[3]`, `.config.after[3]` | Register `0x23` before and after configuration. Array index zero is register `0x20`. |
| `motor1_debug.kt_trace` | Last 64 complete polls with elapsed time, raw `MotorKt1/2`, RPM, FG count, status, and buffered command. |
| `motor1_debug.last_driver_register`, `.i2c_failure`, `.stop_i2c_failure` | The register at exit, the original failed transfer, and any failure when sending the stop command. |
| `motor1_debug.stop_requested`, `.stop_acknowledged` | Set the request to `true` and resume execution. The acknowledgement shows whether both speed-zero writes completed. |
| `motor1_debug.pause_after_wake` | Optional measurement breakpoint. Set at `main()` with the motor phases disconnected. |

Halting the STM32 leaves the DRV10975 on its last command. Let the firmware
execute to service a debugger stop request.

After a fault, run `p motor1_debug.kt_trace`. If the trace is full,
`next_index` points to the oldest sample, and the order wraps at the end
of the array. Otherwise, the first `count` entries are valid in order.
`fault_elapsed_ms` records when the running firmware saw the driver fault.
Each `kt_raw` is the combined register value. Divide by 2884 for V/Hz.
With the current `Kt = 0x07` and `KtLckThr = 3`, raw values below 7 or
above 28 fall outside the configured Lock2 window.

## Adding hardware later

For Motor 2, describe its I²C bus and control pins in the board module,
including its own FG interrupt line and counter. Give it a profile,
`motor_control_t` instance, and debug record in the application. Call
`motor_control_service()` once per foreground pass for each enabled motor.
Each driver can use address `0x52` on its separate bus. Budget the combined
I²C service time before enabling both motors.

Another I²C device can use the same transport with its own address on a
compatible bus. Its device driver should define its own transaction
sequence and registers. The board provides the bus, and the application
decides when to service the device.
