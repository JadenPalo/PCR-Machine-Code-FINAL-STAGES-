# Arduino PCR Temperature Controller

An Arduino-based Polymerase Chain Reaction (PCR) temperature control system designed to continuously monitor, regulate, and transition thermal states using a DS18B20 digital temperature sensor, an Exponential Moving Average (EMA) noise filter, a Proportional-Integral-Derivative (PID) control algorithm, and a BTS7960 H-bridge motor driver to drive a Peltier module.

---

## Overview

The primary function of this system is to maintain precise thermal control over an aluminum sample block required for nucleic acid amplification. The controller continuously reads temperature data, filters out sensor noise, calculates thermal error against a target setpoint, and dynamically adjusts power and polarity delivered to the Peltier element for both heating and cooling cycles.

### Operational Sequence

1. **Data Acquisition:** Sample raw temperature data from the DS18B20 sensor.
2. **Sensor & Range Verification:** Validate sensor connectivity and check operating range constraints.
3. **Filtering:** Apply an Exponential Moving Average (EMA) filter to the raw reading.
4. **PID Control Calculation:** Evaluate error relative to target temperature using the PID algorithm.
5. **Driver Output Generation:** Compute required Pulse Width Modulation (PWM) duty cycle and determine direction (heating vs. cooling).
6. **Thermal Regulation:** Actuate the BTS7960 driver to supply power to the Peltier module.
7. **Execution Loop:** Repeat continuously while control mode is enabled.

---

## Hardware Configuration & Pin Assignments

| Component | Pin / Interface | Description |
| :--- | :--- | :--- |
| **DS18B20 Sensor** | Digital Pin 2 | 1-Wire Digital Temperature Sensor |
| **BTS7960 RPWM** | Digital Pin 5 (PWM) | Forward / Heating PWM Control Signal |
| **BTS7960 LPWM** | Digital Pin 6 (PWM) | Reverse / Cooling PWM Control Signal |
| **BTS7960 Enable** | Digital Pins 7 & 8 | Driver Enable / Disable Control Lines |

> **Note:** A software lockout flag (`PELTIER_HARDWARE_VERIFIED`) is initialized to `0` on system boot to prevent activation until hardware connectivity is verified.

---

## Control Theory & Signal Processing

### 1. Temperature Noise Filtering (EMA)

To mitigate transient measurement noise and momentary disconnection dropouts (such as unexpected $-127^\circ\text{C}$ readings), an Exponential Moving Average (EMA) filter is applied to the raw readings:

$$EMA = (P \cdot \alpha) + \left(\text{Previous } EMA \cdot (1 - \alpha)\right)$$

Where:
* $P$ = Current raw sensor temperature reading
* $\alpha$ = Smoothing factor defined by $\alpha = \frac{2}{1 + N}$
* $N$ = Effective number of time periods

The smoothing factor is configured to $\alpha = 0.2$, balancing raw sensor response ($20\%$) with historical filtered data ($80\%$) to smooth spurious fluctuations.

### 2. PID Temperature Control

Thermal error is evaluated using the filtered temperature value:

$$\text{ERROR} = T_{\text{target}} - T_{\text{filtered}}$$

The continuous control variable $u(t)$ is calculated via the standard PID control equation:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(t) \, dt + K_d \frac{de(t)}{dt}$$

* **Proportional Gain ($K_p$):** Generates output proportional to immediate thermal error.
* **Integral Gain ($K_i$):** Accumulates steady-state error over time to eliminate long-term offset.
* **Derivative Gain ($K_d$):** Responds to the rate of error change to prevent overshoot.

### 3. Peltier Module Actuation Logic

The PID output $u(t)$ is converted into a 8-bit PWM value ($0$ to $255$):

* **Positive Output ($u(t) > 0$):** Set system to **Heating** mode. Apply PWM to `RPWM` (Pin 5) while holding `LPWM` (Pin 6) at `0`.
* **Negative Output ($u(t) < 0$):** Set system to **Cooling** mode. Apply PWM to `LPWM` (Pin 6) while holding `RPWM` (Pin 5) at `0`.
* **Zero Output ($u(t) = 0$):** Deactivate outputs (`RPWM = 0`, `LPWM = 0`).

---

## Timing Architecture

The main execution loop operates using non-blocking, independent interval timers to ensure real-time responsiveness:

* **Sensor Sampling Interval:** Updates temperature measurement every $150\text{ ms}$.
* **Control Calculation Interval:** Executes PID update every $150\text{ ms}$.

Because control loops depend on accurate interval timing for integral integration ($\int e(t) dt$) and derivative differentiation ($\frac{de}{dt}$), the exact elapsed time ($\Delta t$) between updates is calculated continuously to avoid timing jitter.

---

## Serial Command Interface

The system includes a interactive command-line interface over Serial Monitor (`processSerial()`):

| Command | Action / Behavior |
| :--- | :--- |
| `HELP` | Outputs the list of available commands. |
| `SET 37` | Sets the target temperature to $37^\circ\text{C}$ (or specified numerical value). |
| `CONTROL ON` | Activates PID temperature control (requires hardware verification & valid sensor state). |
| `CONTROL OFF` | Disables PID control and cuts power to the Peltier driver. |
| `STATUS` | Prints raw temp, filtered temp, target temp, and controller operational state. |

---

## Safety Features & Diagnostics

The controller incorporates multi-layer safety mechanisms to prevent hardware damage:

1. **Sensor Validation Check:** Verifies DS18B20 connection before allowing control loop activation.
2. **Sensor Operational Range Guard:** Rejects readings outside $-55^\circ\text{C} \ge T \ge 125^\circ\text{C}$.
3. **Over-Temperature Protection:** Triggers immediate system shutdown if raw reading reaches $100^\circ\text{C}$.
4. **Hardware Verification Lockout:** Block activation until `PELTIER_HARDWARE_VERIFIED` is explicitly enabled.
5. **Anti-Windup Protection:** Constrains the integral sum to prevent excessive error accumulation.
6. **Derivative Filter:** Filters high-frequency noise spikes in derivative calculations to prevent control instability.

In the event of an error condition, `safetyShutdown()` is invoked to instantly stop the Peltier driver, clear PID accumulator terms, disable control mode, and issue an alert over the serial terminal.
