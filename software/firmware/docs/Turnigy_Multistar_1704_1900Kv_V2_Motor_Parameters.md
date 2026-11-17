# Turnigy Multistar 1704-1900Kv — Motor Parameters

**Motor:** Turnigy Multistar 1704-1900Kv 12 Pole Multi-Rotor Outrunner V2  
**Type:** Three-phase permanent-magnet BLDC / outrunner (can be controlled as a PMSM using FOC)  
**Compiled:** 2026-11-10

> **Source categories:** **[TI identified]** = values identified on a sample of this exact motor by Texas Instruments MotorWare/InstaSPIN-FOC; **[Catalog]** = published V2 product specifications; **[Calculated]** = estimates using stated assumptions, **not directly measured**. Real motors and measurement methods can give different values.

## 1. Quick reference — electrical / FOC parameters

| Parameter | Symbol or TI variable | Value | Unit | Source / interpretation |
|---|---|---:|---|---|
| Motor type | `USER_MOTOR_TYPE` | `MOTOR_Type_Pm` | — | [TI identified] |
| Number of rotor magnetic poles | `P` | **12** | poles | [TI identified / Catalog] |
| Pole pairs | `p` | **6** | pairs | [TI identified] |
| Speed constant | `Kv` | **1900** | rpm/V | [Catalog] |
| Stator phase resistance (TI model) | `Rs` | **0.225925133** | Ω | [TI identified] |
| d-axis inductance | `Ld` | **15.7321483** | µH | [TI identified] |
| q-axis inductance | `Lq` | **15.7321483** | µH | [TI identified] |
| Identified rated flux / back-EMF parameter | `USER_MOTOR_RATED_FLUX` | **0.00297940802** | V/Hz electrical | [TI identified]; TI's specific V/Hz convention |
| Approx. line-to-line inductance | `L_LL` | **31.4643** | µH | [Calculated] ≈ 2 × TI phase inductance; not an LCR reading |
| Approx. line-to-line DC resistance | `R_LL` | **0.451850266** | Ω | [Calculated] ≈ 2 × TI phase resistance for a balanced equivalent wye model |
| Catalog “internal resistance” | — | **0.405** | Ω | [Catalog]; phase/line-line convention not specified; do **not** substitute for `Rs` without checking |
| Flux linkage under TI's V/Hz-to-Wb conversion | `lambda_m` | **0.00047419** | Wb | [Calculated] = rated flux / (2π); sinusoidal convention |
| Torque constant, peak q-axis current | `Kt,iq_peak` | **0.00427** | Nm/A_peak | [Calculated] = 1.5 × 6 × `lambda_m` for `Ld = Lq` |
| Idealized torque constant from advertised Kv | `Kt,Kv` | **0.00503** | Nm/A | [Calculated] = 60 / (2π × 1900); **different conventions**, not a direct measurement |

**Naming warning:** In most motor-control literature, `Ke` is the **back-EMF constant** and `Kt` is the **torque constant**. Voltage and current conventions (phase-neutral vs line-line; peak vs RMS vs DC input current; mechanical vs electrical angular velocity) must be specified before comparing constants.

## 2. Published V2 product specifications

These specifications are taken from a listing explicitly labeled **V2**, rather than the older non-V2 HobbyKing listing (which gives different mechanical details).

| Parameter | Listed value | Notes |
|---|---|---|
| Product | Turnigy Multistar 1704-1900Kv, 12-pole, V2 | Multi-rotor outrunner |
| Speed constant | 1900 rpm/V | Nominal catalog Kv |
| Recommended battery | 2–3S LiPo | Nominally ~7.4–11.1 V; a fully charged 3S pack is 12.6 V |
| “Max voltage” in product summary | 11 V | Catalog field; appears to refer to ~3S nominal voltage, **not** necessarily an absolute electrical limit |
| Maximum power | 49 W | Catalog rating; conditions not documented |
| Maximum current | 4.5 A | Stated for **10 seconds**, not a continuous-current guarantee |
| No-load current | 0.5 A | Test voltage/speed not specified in the listing |
| “Internal resistance” | 0.405 Ω | Measurement convention not specified |
| Magnetic poles | 12 | Six pole pairs |
| Motor-only mass | 15.5 g | Excludes adapters and screws |
| Outer dimensions | 21 mm diameter × 15.5 mm length | V2 listing |
| Shaft specification | “3 × 2 mm” | Catalog's shaft-size text; verify physical dimensions before manufacturing a mount |
| Mounting-hole spacing | 14 mm | Catalog |
| Mounting screw thread | M2 | Catalog |
| Motor connectors | 2 mm bullet | Catalog |
| Prop adapters | Removable conventional adapter and flat two-hole adapter | V2 catalog description |

## 3. Original TI InstaSPIN-FOC identified values

Texas Instruments tested the **exact Turnigy Multistar 1704-1900Kv 12 Pole Multi-Rotor Outrunner V2** in **TIDA-00916**. The numbers below are copied from **Section 4.1.1, “Motor Identification,” printed page 16**, obtained with MotorWare `proj_lab02c`:

```c
#define USER_MOTOR_TYPE              MOTOR_Type_Pm
#define USER_MOTOR_NUM_POLE_PAIRS    (6)
#define USER_MOTOR_Rs                (0.225925133)   // ohm; stator phase model
#define USER_MOTOR_Ls_d              (1.57321483e-05) // H = 15.7321483 uH
#define USER_MOTOR_Ls_q              (1.57321483e-05) // H = 15.7321483 uH
#define USER_MOTOR_RATED_FLUX        (0.00297940802) // V/Hz, TI convention
```

**TI test context (not motor ratings):** The tests were carried out near **22–23 °C**, with approximately **45 kHz PWM** and a **22.5 kHz current-control loop**. TI also demonstrated operation around **12,000 rpm** using an **11.2 V DC bus**; this is a demonstrated test point, **not** a guaranteed motor maximum or recommended ESC switching frequency.

## 4. Calculations and unit conventions

### 4.1 Inductance for an LCR meter

The identified inductances are:

- `Ld = Lq = 1.57321483 × 10^-5 H = 15.7321483 µH` (TI **FOC phase-model** parameters).
- Approximate measured terminal-pair inductance for a symmetric three-phase wye-equivalent motor: `L(U–V) ≈ L(V–W) ≈ L(W–U) ≈ 2 × 15.7321483 = 31.4643 µH`.

The **31.46 µH** result is an **estimate**, not a published direct LCR reading. It assumes the controller's equivalent phase inductance convention and a balanced machine; the result may change with rotor angle, magnetic coupling, bias current, test frequency, and the meter's series/parallel mode.

**LCR procedure:** Disconnect ESC and power; hold rotor stationary; perform open/short fixture compensation; select `Ls–Rs` (series) and start at **1 kHz**; measure between **U–V**, **V–W**, and **W–U**, leaving the third lead floating. Record the frequency and minimum/maximum readings across rotor positions.

### 4.2 Resistance

For a balanced equivalent wye model:

`R_phase(TI) = 0.225925133 Ω`  
`R_line-line ≈ 2 × R_phase = 0.451850266 Ω`

The catalog separately quotes `0.405 Ω` as **internal resistance** but does not identify its test configuration. This may differ because of convention, temperature, sample variation, and method. Very small DC resistances should be measured with **four-wire/Kelvin** methods, ideally with a current-limited DC method.

### 4.3 Flux, back EMF, and torque constant

TI's reported motor-identification quantity:

`RatedFlux = 0.00297940802 V/Hz` (**electrical** Hz; TI InstaSPIN convention).

For the standard sinusoidal FOC flux-linkage convention:

`lambda_m ≈ RatedFlux / (2π) = 0.00047419 Wb`  
`Kt (for peak-valued Iq, Ld ≈ Lq) ≈ (3/2) × p × lambda_m ≈ 0.00427 Nm/A_peak`

An **independent, idealized** estimate from the **advertised Kv** is:

`Kt ≈ 60 / (2π × Kv) = 60 / (2π × 1900) ≈ 0.00503 Nm/A`

**Do not treat these two torque figures as directly interchangeable:** they arise from different source measurements/conventions and depend on whether current means sinusoidal peak phase `Iq`, RMS phase current, or DC bus current. For an FOC model that uses TI's phase and peak-`Iq` definitions, the TI-flux-derived result is the internally consistent starting point.

### 4.4 Electrical vs mechanical speed

With `p = 6` pole pairs:

`f_electrical [Hz] = rpm_mechanical × 6 / 60 = rpm_mechanical / 10`  
`rpm_mechanical = 10 × f_electrical [Hz]`

Examples: **10,000 rpm → 1,000 Hz electrical**; **12,000 rpm → 1,200 Hz electrical**.

## 5. Recommended initial motor-model inputs

For a **sinusoidal, symmetric PMSM / surface-PM-like FOC model** using **phase-equivalent resistance and inductance** and **peak-valued d–q currents**:

```text
motor_type               = PM / BLDC / PMSM
pole_pairs               = 6
Rs_phase_ohm             = 0.225925133
Ld_H                     = 1.57321483e-5
Lq_H                     = 1.57321483e-5
rated_flux_V_per_Hz_elec = 0.00297940802  # TI InstaSPIN units
flux_linkage_Wb          = 0.00047419     # derived under sinusoidal convention
Kt_Nm_per_Apeak_Iq       = 0.00427        # derived, not directly tested
Kv_rpm_per_V             = 1900          # catalog
```

**Before using in a different simulation package:** Check whether it expects **line-to-line vs phase** values and **phase peak vs RMS** conventions. Do not blindly put the TI `V/Hz` parameter into a field expecting `V/(rad/s)`.

## 6. Sources

1. **Texas Instruments**, *High-Speed Sensorless-FOC Reference Design for Drone ESCs*, TIDUCF1 (December 2016), **Table 5 (p. 15)** and **§4.1.1, p. 16** for the exact motor and its identified parameters. [Official PDF](https://www.ti.com/lit/ug/tiducf1/tiducf1.pdf) · [TIDA-00916 design page](https://www.ti.com/tool/TIDA-00916)
2. **Turnigy / HobbyKing V2 catalog text mirrored in RCSearch**, *Turnigy Multistar 1704-1900Kv 12 Pole Multirotor Outrunner V2*, listing SKU 9192000198-0; published V2 mechanical/electrical specs. [V2 listing](https://rcsearch.ru/hobbyking/i91879/). This is a catalog mirror, not a measurement certificate.
3. **Texas Instruments**, *Motor Parameter Tuning — AM13E230X SDK User Guide*, for interpretation of `Rs`, `Ls_d`, `Ls_q`, and `RatedFlux_VpHz` in TI's motor-model terminology. [TI documentation](https://software-dl.ti.com/mcu-sdk/esd/AM13E2X-SDK/26_01_00_03/exports/docs/docs/am13e230x/user_guide/chapters/solutions/am13e230x/motor_control/control_motor_tuning.html)

---

*Engineering note: These are practical starting parameters, not a complete validated motor model. Datasheet values, TI motor identification, and derived numbers describe different test conditions. Motor inertia, friction, magnet temperature coefficients, thermal resistance, back-EMF waveform, and current-dependent saturation curves were not reliably established by these sources.*
