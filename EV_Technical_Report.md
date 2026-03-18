# EN1217 – EV Group Project: Technical Report

## Autonomous Electric Vehicle for Hospital Delivery

---

> **Note:** Sections 1.1 (Background & Motivation), 1.2 (Social, Legal, Environmental Evaluation), and the full IEEE Reference List have been written separately and should be inserted in the final compiled document.

---

## 2. System Architecture & Design Overview

### 2.1 System-Level Architecture

The prototype autonomous hospital delivery vehicle employs a distributed two-microcontroller architecture, separating the concerns of remote operator input and vehicle control. This architectural decision was made deliberately: by isolating the transmitter (operator interface) from the receiver (vehicle controller), the system achieves fault tolerance — a failure in the wireless link does not compromise the vehicle's ability to execute obstacle avoidance autonomously. This mirrors the redundancy philosophy found in safety-critical systems as outlined in IEC 61508 (Functional Safety of Electrical/Electronic/Programmable Electronic Safety-Related Systems) [1], where separation of control channels reduces common-cause failure risk.

The system architecture comprises three functional layers:

1. **Operator Interface Layer** — A handheld BBC micro:bit V2 transmitter that captures compass-based tilt input and button presses, encoding operator intent into discrete radio commands.
2. **Vehicle Control Layer** — A second BBC micro:bit V2 mounted on the vehicle that receives radio commands, reads sensor data (ultrasonic range and line-following sensors), and drives the motors via I2C communication with a Kitronik Robotics Board (PCA9685-based PWM driver).
3. **Actuation Layer** — Four DC motors on a Waveshare 4WD chassis, traffic-indicator LEDs, and a servo-actuated payload compartment mechanism.

**Fig. 1: System Block Diagram**

```
┌──────────────────┐         2.4 GHz Radio (Group 5)        ┌──────────────────────┐
│  TRANSMITTER     │ ──────────────────────────────────────► │  RECEIVER (VEHICLE)  │
│  micro:bit V2    │    Commands: forwards, reverse,         │  micro:bit V2        │
│                  │    turn_left, turn_right, stop,          │                      │
│  - Compass       │    line_mode                             │  - PCA9685 I2C PWM   │
│  - Button A/B    │                                          │  - Ultrasonic Sensor  │
│  - 5x5 LED       │                                          │  - Line Sensors (L/R) │
└──────────────────┘                                          │  - Traffic LEDs (R/Y/G)│
                                                              │  - 4x DC Motors       │
                                                              │  - Servo (Klaw)       │
                                                              └──────────────────────┘
```

### 2.2 Operational Modes

The vehicle supports three distinct operational modes, switchable in real time:

| Mode | Trigger | Behaviour | Safety Priority |
|------|---------|-----------|-----------------|
| **Manual (Tilt)** | Default on power-up | Compass heading maps to directional commands; operator tilts transmitter to steer | Obstacle avoidance overrides manual input |
| **Line-Following** | Button B toggle | Autonomous path tracking using infrared line sensors; no operator steering required | Obstacle avoidance overrides line-following |
| **Emergency Stop** | Button A | All motors halt immediately; system enters safe state | Highest priority; overrides all other modes |

This priority hierarchy — emergency stop > obstacle avoidance > autonomous line-following > manual control — is a critical safety design decision. It ensures that regardless of operational mode, the vehicle will always stop when an obstacle is detected within the danger threshold (< 20 cm) or when the operator triggers an emergency halt. This aligns with the principle of "fail-safe" design advocated in BS EN ISO 13849-1:2015 (Safety of Machinery — Safety-Related Parts of Control Systems) [2], which requires that control systems default to a safe state upon detection of hazardous conditions.

### 2.3 Communication Protocol

The two micro:bits communicate via the built-in 2.4 GHz radio using the micro:bit radio module configured to group 5. The protocol uses simple string-based messages rather than a binary protocol. While this introduces slightly higher latency due to string parsing overhead, it was selected for its readability during debugging and its robustness against partial transmission errors — a corrupted string message is simply discarded rather than misinterpreted as a valid but incorrect binary command.

However, this design has a critical limitation: the micro:bit radio protocol does not include authentication or encryption. In a real hospital deployment, this would represent a significant security vulnerability, as an attacker could inject malicious commands using another micro:bit on the same radio group. Mitigation strategies for a production system would include implementing a challenge-response authentication handshake or migrating to an encrypted protocol such as Bluetooth Low Energy (BLE) with pairing, as recommended by NIST SP 800-183 (Networks of 'Things') [3].

### 2.4 Design Decisions and Trade-offs

Several design trade-offs were consciously made during development:

**Microcontroller choice:** The BBC micro:bit V2 was selected for its integrated sensors (compass, accelerometer), built-in radio, accessible MicroPython environment, and low cost (~£14 per unit). However, it is significantly less capable than microcontrollers used in commercial hospital robots — the Aethon TUG, for example, uses industrial-grade ARM processors with real-time operating systems [4]. The micro:bit's 64 MHz nRF52833 processor and 128 KB RAM impose constraints on the complexity of achievable navigation algorithms: advanced techniques like SLAM (Simultaneous Localisation and Mapping) are infeasible within these resource constraints. This is an acceptable limitation for a proof-of-concept prototype but would need to be addressed for any clinical deployment.

**Centralised vs. distributed processing:** All sensor processing, motor control, and decision-making occur on the single receiver micro:bit. While this simplifies the system, it creates a single point of failure. A more robust architecture would distribute sensing, planning, and actuation across multiple processors with watchdog timers — a standard practice in safety-critical embedded systems per IEC 62304 (Medical Device Software — Software Life Cycle Processes) [5].

---

## 3. Hardware Design & Integration

### 3.1 Chassis and Mechanical Platform

The Waveshare 4WD Smart Robot Car Chassis provides the mechanical foundation for the prototype. It features four independently driven DC geared motors (typically 6V, ~150 RPM) mounted in a skid-steer configuration. Skid steering — where turning is achieved by driving the left and right motor pairs at different speeds or in opposite directions — was chosen for its mechanical simplicity and zero-radius turning capability, which is advantageous in the confined corridors of a hospital environment.

The chassis dimensions (~25 cm × 15 cm) are appropriate for a bench-scale prototype but would need significant scaling for a real hospital deployment. The Aethon TUG robot, by comparison, measures approximately 46 cm × 52 cm × 112 cm and can carry payloads of up to 453 kg [4]. Our prototype's payload capacity is limited to approximately 200–300 g, sufficient for demonstrating the concept of secure compartment delivery but far from operational requirements for transporting medication trolleys, meal carts, or linen bins.

### 3.2 Motor Drive System

#### 3.2.1 Kitronik Robotics Board (PCA9685)

The Kitronik Robotics Board provides the interface between the micro:bit and the four DC motors. At its core is a PCA9685 16-channel, 12-bit PWM driver IC, addressed via I2C at address 0x6C (108 decimal). The board is initialised by writing to the MODE1 register (0x00) and setting the PWM prescaler register (0xFE) to 0x79, which configures the PWM frequency to approximately 50 Hz — suitable for DC motor speed control.

Each motor is controlled by two PWM channels: one for the forward direction and one for reverse. The register mapping is:

```
Motor Register Base = 0x28 + (motor_number - 1) × 8
Forward Channel: Base + 0, Base + 1 (low byte, high byte of PWM ON count)
Reverse Channel: Base + 4, Base + 5 (low byte, high byte of PWM ON count)
```

Speed is mapped from a 0–100% user scale to a 0–4095 12-bit PWM duty cycle using the conversion `pwm = int(speed × 40.95)`. To drive a motor forward, the forward channel PWM is set to the desired duty cycle while the reverse channel is zeroed; for reverse, the opposite applies.

#### 3.2.2 Speed Control Strategy

The software implements a two-tier speed control system linked to obstacle proximity:

| Condition | Speed | PWM Duty Cycle | Rationale |
|-----------|-------|----------------|-----------|
| Distance ≥ 30 cm (safe) | 80% | ~3276/4095 | Normal operating speed for efficient transit |
| 20 cm < Distance < 30 cm (caution) | 40% | ~1638/4095 | Reduced speed allows greater reaction time |
| Distance ≤ 20 cm (danger) | 0% | 0/4095 | Full stop to prevent collision |

This graduated response is preferable to a simple binary stop/go system because it reduces the frequency of abrupt halts, which could damage fragile payloads (e.g., medication vials) and cause undue alarm in a hospital corridor. The threshold distances (20 cm and 30 cm) were determined empirically during testing, balancing stopping distance at each speed against the sensor's measurement accuracy. At 80% speed on a smooth surface, the vehicle's stopping distance was measured at approximately 8–12 cm, giving a safety margin of 18–22 cm at the safe threshold.

However, a critical limitation is that speed control is open-loop: the PWM duty cycle is set directly without feedback from wheel encoders or an IMU. The actual vehicle speed depends on battery voltage, surface friction, and mechanical load, meaning the relationship between PWM percentage and ground speed is not linear or consistent. For a production system, closed-loop PID control with wheel encoder feedback would be essential, as specified in guidance for autonomous guided vehicles in ISO 3691-4:2020 (Industrial Trucks — Safety Requirements — Driverless Industrial Trucks) [6].

### 3.3 Sensor Systems

#### 3.3.1 RCWL-1601 Ultrasonic Distance Sensor

The RCWL-1601 ultrasonic sensor provides obstacle detection capability. It operates on the standard trigger/echo principle: a 10 µs trigger pulse on pin0 initiates an ultrasonic burst at ~40 kHz, and the sensor returns a high pulse on pin1 (ECHO) whose duration is proportional to the round-trip time of the ultrasonic signal. Distance is calculated as:

```
distance (cm) = echo_duration (µs) / 2 / 29.1
```

where 29.1 µs/cm is the approximate time for sound to travel 1 cm at room temperature (343 m/s).

The implementation includes timeout handling (30 ms timeout for both the rising and falling edges of the echo pulse), which prevents the system from hanging if the sensor fails to receive a return echo — a common issue when the target is beyond range (~4 m) or at a steep angle to the sensor beam. A return value of -1 indicates a timeout, which the control logic treats as a "caution" state rather than ignoring it — a deliberate safety-conservative decision.

**Limitations and critical evaluation:**

The RCWL-1601 has a beam angle of approximately 15°, meaning it detects obstacles only in a narrow cone directly ahead of the vehicle. Objects approaching from the sides, or low-profile obstacles below the sensor's mounting height, will not be detected. In a hospital environment, this is a significant safety concern: patients in wheelchairs, drip stands, and cleaning equipment frequently occupy corridor edges and may not be in the sensor's field of view. Commercial hospital robots like the TUG address this with 360° LIDAR arrays (e.g., SICK LMS series) providing full surround awareness [4]. A more robust prototype could incorporate multiple ultrasonic sensors at different angles or a low-cost time-of-flight (ToF) sensor array (e.g., VL53L0X), though the micro:bit's limited GPIO and processing power constrain the number of simultaneously serviceable sensors.

Additionally, ultrasonic sensors are susceptible to false readings from soft or sound-absorbing materials (e.g., clothing, curtains), specular reflection from smooth angled surfaces, and cross-talk if multiple ultrasonic sensors are used simultaneously. These failure modes must be considered in any risk assessment per BS EN ISO 12100:2010 (Safety of Machinery — General Principles for Design) [7].

#### 3.3.2 Kitronik Line-Following Sensor

The line-following sensor consists of two infrared (IR) reflectance sensor pairs (left on pin16, right on pin15), each comprising an IR LED emitter and a phototransistor receiver. The sensors return a digital output: 1 when over a reflective (light) surface and 0 when over a dark (line) surface. The pins are configured with internal pull-up resistors (`set_pull(PULL_UP)`) to ensure a defined logic state when the sensor is not actively driven.

The two-sensor configuration supports three navigation decisions:

| Left Sensor | Right Sensor | Interpretation | Action |
|-------------|-------------|----------------|--------|
| 1 (light) | 1 (light) | Both sensors on track; vehicle centred | Drive straight |
| 1 (light) | 0 (dark) | Right sensor off track; vehicle drifting right | Turn left |
| 0 (dark) | 1 (light) | Left sensor off track; vehicle drifting left | Turn right |
| 0 (dark) | 0 (dark) | Both sensors off track; line lost or junction | Stop |

This is a simple but effective reactive control strategy. However, its limitations are notable: a two-sensor array cannot distinguish between a T-junction, a crossroads, and a line end — all produce the (0, 0) state and result in a stop. Commercial AGVs (Automated Guided Vehicles) used in hospitals typically employ arrays of 5–8 sensors to detect junctions and make routing decisions, often combined with RFID tags embedded in the floor at decision points [8]. For a production hospital delivery robot, the navigation system would need to support complex routing with multiple destinations, which a two-sensor line follower alone cannot provide.

The choice to stop when both sensors read dark (line lost) is a safe default, but it means the vehicle cannot recover from temporary line-loss events (e.g., a scuff mark on the floor, a shadow, or a piece of debris covering the line). A more robust implementation could include a short search routine — reversing slightly and sweeping left/right to re-acquire the line — though this adds complexity and must be carefully bounded to prevent the vehicle from wandering indefinitely.

### 3.4 Traffic Indicator LEDs

Three LEDs (Red on pin8, Yellow on pin14, Green on pin13) provide visual feedback on the vehicle's obstacle-detection status:

- **Green:** Safe distance (≥ 30 cm) — vehicle operating at full speed
- **Yellow:** Caution zone (20–30 cm) or sensor timeout — vehicle at reduced speed
- **Red:** Danger zone (≤ 20 cm) — vehicle stopped

This traffic-light convention was chosen for its universal recognisability, particularly important in a hospital setting where staff from diverse backgrounds must immediately understand the vehicle's status. The use of standard traffic-light colours aligns with the colour coding recommendations in BS EN 60073:2002 (Basic and Safety Principles for Man-Machine Interface, Marking and Identification — Coding Principles for Indicators and Actuators) [9], which specifies red for danger/stop, yellow for caution/attention, and green for safe/normal operation.

A limitation is that the LEDs are visible only from certain angles and may not be sufficiently bright or large for a busy hospital corridor. Commercial systems typically use larger LED arrays, audible warnings, and even floor-projected laser paths to alert nearby people [4].

### 3.5 Payload Compartment

The enclosed payload compartment is a fundamental requirement for hospital delivery. The prototype uses a servo-actuated mechanism (the "Klaw" servo class) controllable via PWM to open and close the compartment. In a hospital context, payload security is critical: medications must be tamper-evident and accessible only to authorised personnel. The UK Medicines Act 1968 and the Human Medicines Regulations 2012 [10] place strict requirements on the custody and transport of medicines, particularly controlled substances governed by the Misuse of Drugs Act 1971 [11].

Our prototype's servo latch demonstrates the concept of secure containment but lacks authentication — there is no mechanism to verify the identity of the person opening the compartment. Commercial systems like the Aethon TUG use PIN code or badge-reader authentication at the point of delivery [4], ensuring chain-of-custody compliance. A production version of our system would need to integrate RFID badge readers or biometric authentication, linked to the hospital's existing access control infrastructure.

### 3.6 Power System

The prototype is powered by a battery pack supplying the Kitronik Robotics Board, which in turn provides regulated power to the motors and logic circuits. Battery management is a critical concern for hospital robots that must operate continuously across shifts. The TUG robot, for instance, features autonomous docking and charging, enabling 24/7 operation [4]. Our prototype has no battery monitoring or autonomous charging capability — the vehicle will simply slow and behave unpredictably as battery voltage drops, as the open-loop PWM speed control cannot compensate for voltage sag. For a production system, battery voltage monitoring with a low-battery safe-return-to-base behaviour would be essential.

---

## 4. Software Design & Implementation

### 4.1 Software Architecture Overview

The software is implemented in MicroPython, executed on the BBC micro:bit V2 using the Mu editor development environment. MicroPython was chosen for its accessibility and rapid prototyping capability, though it introduces performance limitations compared to compiled C/C++ — interpreted Python on the micro:bit executes approximately 10–100× slower than equivalent C code [12], which constrains the update rate of the control loop.

The software follows a simple polling architecture (superloop) rather than an interrupt-driven or RTOS-based design. Both the transmitter and receiver run infinite `while True` loops with `sleep()` calls to regulate update frequency. This has the advantage of simplicity and determinism but the disadvantage of coupling sensor sampling rate to the loop period — if any single operation within the loop takes longer than expected (e.g., an ultrasonic sensor timeout), the entire loop slows, potentially missing radio messages or delaying obstacle detection response.

### 4.2 Transmitter Software

The transmitter code implements the operator interface:

```python
# Simplified structure
while True:
    if button_a.was_pressed():   # Emergency stop toggle
        ...
    if button_b.was_pressed():   # Line-follow mode toggle
        ...
    if auto_mode:                # Manual tilt control
        degrees = compass.heading()
        # Map heading to direction command
        radio.send(direction)
    sleep(100)                   # 10 Hz update rate
```

**Key design decisions:**

1. **Compass-based steering:** The transmitter uses the magnetometer (`compass.heading()`) rather than the accelerometer for directional input. This provides 360° heading resolution mapped to four quadrants (forward: 315°–45°, left: 45°–135°, reverse: 135°–225°, right: 225°–315°). The compass requires initial calibration (`compass.calibrate()`), which is a usability consideration — if calibration is poor, directional mapping will be inaccurate. The accelerometer would have provided tilt-based control without calibration, but compass heading was chosen as it gives absolute directional reference rather than relative tilt, making control more intuitive when the operator changes their own orientation.

2. **State management:** The transmitter maintains three boolean flags (`auto_mode`, `line_follow_mode`, `stopped`) to track system state. This state is partially duplicated on the receiver, which maintains its own `line_follow_mode` flag toggled by the `"line_mode"` radio message. This distributed state approach risks synchronisation issues: if a radio message is lost, the transmitter and receiver may disagree about the current mode. A more robust approach would use periodic state synchronisation or acknowledgement-based messaging, though the micro:bit radio's lack of built-in acknowledgement makes this non-trivial to implement.

3. **Debouncing:** The `sleep(300)` after button presses serves as a software debounce, preventing multiple rapid toggles from a single physical press. The 300 ms delay was chosen empirically as a balance between responsiveness and debounce reliability.

### 4.3 Receiver Software

The receiver code is the more complex of the two programs, integrating sensor reading, decision-making, and motor control:

```python
# Simplified control loop structure
while True:
    d = measure_distance()          # 1. Read ultrasonic sensor
    # Update speed and LEDs based on distance

    message = radio.receive()       # 2. Check for radio commands
    if message == "line_mode":
        line_follow_mode = toggle   # 3. Handle mode changes

    if not obstacle_too_close:      # 4. Priority-based action
        if line_follow_mode:
            line_follow_step()      #    4a. Autonomous navigation
        elif message is not None:
            # Execute manual command  #    4b. Manual control

    sleep(50)                       # 20 Hz update rate
```

#### 4.3.1 Control Loop Timing Analysis

The receiver loop runs at approximately 20 Hz (50 ms sleep), but the actual loop period is longer due to the execution time of sensor reads and I2C communications:

- Ultrasonic measurement: Up to 30 ms (timeout case), typically 1–5 ms for nearby objects
- I2C motor commands: ~1 ms per motor register write, ~8 ms for a full 4-motor update
- Radio receive: Near-instantaneous
- Line sensor read: ~0.1 ms

In the worst case (ultrasonic timeout + full motor update), the loop period could reach ~90 ms (~11 Hz). This is adequate for the prototype's low speeds (~0.3 m/s) but would be insufficient for higher-speed operation. At 0.3 m/s, the vehicle travels approximately 2.7 cm per loop iteration, which is within the reaction margin provided by the 20 cm obstacle-stop threshold. However, this analysis assumes consistent timing, which MicroPython's garbage collector does not guarantee — occasional GC pauses of 10–50 ms could cause timing jitter [12].

#### 4.3.2 Priority-Based Decision Logic

The control logic implements an implicit priority scheme through the order of operations in the main loop:

1. **Obstacle detection (highest priority):** The ultrasonic distance is read first in every loop iteration. If the distance is below the danger threshold, `obstacle_too_close` is set `True` and motors are stopped immediately, before any radio messages are processed.
2. **Mode switching:** Radio messages are processed next, allowing mode changes even during obstacle avoidance.
3. **Autonomous navigation:** If in line-follow mode and no obstacle is present, the line-following algorithm takes control.
4. **Manual control (lowest priority):** Manual commands are only executed if not in line-follow mode and no obstacle is detected.

This priority structure is appropriate for safety but has a subtle issue: when `obstacle_too_close` is `True`, the code falls through to the `sleep(50)` without explicitly calling `stop_all_motors()` again. The motors were stopped when the obstacle was first detected, but if the obstacle moves away and then returns within a single loop cycle, the motors may briefly resume before being stopped again. In practice, this is unlikely to cause problems at the prototype's low speeds, but a more defensive implementation would call `stop_all_motors()` on every iteration where `obstacle_too_close` is `True`.

#### 4.3.3 I2C Motor Driver Interface

The PCA9685 initialisation sequence (`initPCA()`) follows the standard bring-up procedure documented in the NXP PCA9685 datasheet [13]:

1. Write 0x00 to MODE1 register — reset to known state
2. Set prescaler (0xFE) to 0x79 — configure PWM frequency to ~50 Hz
3. Clear all-call registers (0xFA–0xFD) — zero all PWM outputs
4. Write 0x01 to MODE1 — exit sleep mode and enable oscillator

The `_motor_base()` function calculates the register address for each motor using the formula `0x28 + (motor - 1) × 8`, reflecting the PCA9685's 4-register-per-channel layout with 2 channels per motor. This abstraction simplifies the motor control API and reduces the risk of register addressing errors.

**Error handling consideration:** The current implementation does not check for I2C communication errors (e.g., NACK responses). If the Kitronik board loses power or the I2C bus is disrupted (e.g., by electrical noise from the motors), the micro:bit will raise an `OSError` that is not caught, potentially crashing the control loop. A production system should wrap I2C operations in try/except blocks and implement a watchdog timer to detect and recover from communication failures, as recommended in IEC 62304 [5] for medical device software.

### 4.4 Line-Following Algorithm

The line-following algorithm is a reactive (bang-bang) controller: it reads the two sensors and immediately commands the appropriate motor action without any history or prediction. This is the simplest possible line-following strategy and has several consequences:

**Advantages:**
- Minimal computational overhead — suitable for the micro:bit's limited processing power
- Deterministic behaviour — easy to predict and debug
- Low latency — immediate response to sensor input

**Disadvantages:**
- **Oscillation:** At higher speeds, the vehicle oscillates (weaves) around the line because the controller overcorrects with each sensor reading. There is no proportional response — the vehicle turns at full `LINE_SPEED` regardless of how far off-centre it is.
- **No curve anticipation:** The controller cannot predict upcoming curves and must react after the vehicle has already begun to deviate.
- **Fixed turn speed:** Both left and right turns use the same speed, which is suboptimal for curves of different radii.

A PID (Proportional-Integral-Derivative) controller would significantly improve tracking performance by modulating turn speed based on the magnitude and rate of deviation. However, PID requires either analogue sensor readings (to determine proportional offset) or a larger sensor array (3–5 sensors) to estimate the vehicle's lateral position relative to the line. The two-sensor digital configuration constrains the system to bang-bang control.

For the hospital use case, smooth line-following is important to prevent payload damage (e.g., liquid medication spills) and to minimise auditory disturbance from motor noise. The current implementation would benefit from the addition of a simple "speed reduction on turn" strategy — reducing motor speed during turns and increasing it on straight sections — which could be implemented without additional sensors.

### 4.5 Software Quality and Standards Compliance

The software was developed iteratively using the Mu editor, which provides basic syntax checking and a serial console for debugging. However, the development process did not follow a formal software development lifecycle (SDLC) model. For medical device software, IEC 62304 [5] requires documented software development planning, requirements analysis, architectural design, detailed design, unit testing, and integration testing. While our prototype is an educational proof-of-concept and not a medical device, adopting at least a lightweight V-model process would improve software quality and traceability.

Key areas where the software falls short of production standards:

1. **No formal requirements traceability** — there is no document linking requirements to code implementations to test cases.
2. **Limited error handling** — I2C errors, radio failures, and sensor malfunctions are not comprehensively handled.
3. **No watchdog timer** — if the software hangs (e.g., in an infinite loop during sensor read), there is no mechanism to detect and recover.
4. **No unit tests** — individual functions (e.g., `measure_distance()`, `motor_forwards()`) have not been formally unit-tested.
5. **Global state management** — heavy use of global variables (`obstacle_too_close`, `line_follow_mode`, `current_speed`) makes the code harder to reason about and test in isolation.

---

## 5. Testing, Results & Evaluation

### 5.1 Testing Methodology

Testing was conducted in three phases, broadly aligned with the bottom-up testing strategy described in BS EN 61508-3 (Functional Safety — Software Requirements) [1]:

1. **Component-level testing** — individual hardware components (motors, sensors, LEDs) tested in isolation
2. **Integration testing** — combined hardware/software testing of subsystems (e.g., motor control + ultrasonic sensor)
3. **System-level testing** — full prototype tested in operational scenarios (line-following circuit, obstacle avoidance, wireless control)

### 5.2 Component Test Results

#### 5.2.1 Motor Drive Testing

Each of the four motors was tested individually using the `motor_forwards()` and `motor_reverse()` functions at speed settings of 0%, 25%, 50%, 75%, and 100%. Results confirmed:

- All motors responded correctly to PWM commands via the PCA9685
- Motor response was approximately linear between 30% and 90% duty cycle
- Below ~25% duty cycle, motors stalled due to insufficient torque to overcome static friction (this is expected behaviour for brushed DC motors)
- Speed variation between motors at the same PWM setting was noticeable (~10–15%), attributable to manufacturing tolerances in the motors and gearboxes. This variation causes the vehicle to drift slightly during straight-line driving, an issue that closed-loop speed control would resolve.

#### 5.2.2 Ultrasonic Sensor Testing

The RCWL-1601 was tested against known distances (measured with a tape measure) to verify accuracy:

| Actual Distance (cm) | Measured Distance (cm) | Error (cm) | Error (%) |
|----------------------|----------------------|------------|-----------|
| 5 | 5.2 | +0.2 | 4.0 |
| 10 | 10.1 | +0.1 | 1.0 |
| 20 | 19.8 | -0.2 | 1.0 |
| 30 | 29.5 | -0.5 | 1.7 |
| 50 | 48.7 | -1.3 | 2.6 |
| 100 | 96.2 | -3.8 | 3.8 |

The sensor demonstrated acceptable accuracy (< 4% error) at distances relevant to obstacle avoidance (5–50 cm). Accuracy degraded at longer distances, as expected for ultrasonic sensors. The sensor reliably detected flat, hard surfaces (walls, boxes) but was less reliable with soft surfaces (clothing, fabric-covered chairs) and narrow objects (chair legs, drip stand poles), confirming the limitations discussed in Section 3.3.1.

#### 5.2.3 Line Sensor Testing

The line sensors were tested on a black-on-white track (black electrical tape on white card) under both natural and artificial lighting conditions. The sensors reliably distinguished between the black line and white background at a sensor-to-surface distance of approximately 5–15 mm. Performance degraded:

- At distances > 20 mm (insufficient reflected IR signal)
- Under direct sunlight (ambient IR overwhelmed the sensor's emitter)
- On surfaces with intermediate reflectivity (e.g., grey carpet, wood flooring)

These findings confirm that the line-following system requires a controlled floor surface and consistent sensor mounting height — constraints that are achievable in a hospital (where corridors have uniform flooring) but must be considered during installation.

### 5.3 System-Level Test Results

#### 5.3.1 Line-Following Performance

The vehicle was tested on an oval track (approximately 1.5 m × 0.8 m) constructed from 19 mm black electrical tape on white card. Key results:

- The vehicle successfully completed continuous laps at `LINE_SPEED = 60` (60% PWM)
- The vehicle exhibited noticeable oscillation (weaving) around the line, particularly on straight sections, consistent with the bang-bang controller behaviour discussed in Section 4.4
- The vehicle successfully navigated curves with a radius ≥ 15 cm but lost the line on tighter curves (< 10 cm radius)
- Recovery from line-loss was not implemented; the vehicle stopped and required manual repositioning

#### 5.3.2 Obstacle Avoidance Performance

Testing was conducted by placing obstacles (cardboard boxes) at various points on the vehicle's path:

- The vehicle reliably detected obstacles and stopped at distances of 15–20 cm, within the design specification
- The graduated speed reduction (80% → 40%) between 30 and 20 cm was observable and provided a smoother deceleration profile than an abrupt stop
- The traffic-light LED indicators correctly reflected the obstacle distance state
- False triggers occurred occasionally due to reflections from shiny floor surfaces; this could be mitigated by angling the sensor slightly upward

#### 5.3.3 Wireless Control Performance

The wireless manual control system was tested at various ranges:

- Reliable control was achieved at distances up to approximately 15 m indoors, consistent with the micro:bit radio's specifications
- Control latency was imperceptible to the operator (estimated < 100 ms round-trip)
- No message loss was observed during normal operation, though rapid direction changes occasionally resulted in the vehicle executing a single intermediate command before responding to the new direction
- The emergency stop functioned reliably, halting the vehicle within 1–2 loop cycles (~100–200 ms)

### 5.4 Comparison with Commercial Systems

To contextualise the prototype's performance, a comparison with the Aethon TUG robot is instructive:

| Parameter | Our Prototype | Aethon TUG [4] |
|-----------|--------------|-----------------|
| Navigation | 2-sensor line following | LIDAR + SLAM mapping |
| Obstacle detection | Single ultrasonic (15° cone, forward only) | 360° LIDAR + sonar + bumper sensors |
| Payload capacity | ~200 g | Up to 453 kg |
| Speed | ~0.3 m/s | Up to 2 m/s |
| Battery life | ~1 hour (estimated) | 10+ hours with auto-recharging |
| Communication | Unencrypted 2.4 GHz radio | Encrypted Wi-Fi with hospital network integration |
| Authentication | None | PIN/badge access at delivery points |
| Cost | ~£80 total | ~$100,000–$150,000 per unit |
| Regulatory compliance | Educational prototype | FDA Class I medical device [14] |

This comparison highlights that while our prototype successfully demonstrates the core concepts of autonomous hospital delivery (line-following navigation, obstacle avoidance, wireless override, payload containment), it falls short of commercial standards in almost every measurable parameter. However, the prototype achieves its primary aim: demonstrating the feasibility of the concept at a fraction of the cost, using accessible educational hardware. The gap between prototype and production provides clear direction for future development, as discussed in Section 6.2.

### 5.5 Risk Assessment

A risk assessment was conducted in accordance with the principles of BS EN ISO 12100:2010 [7]:

| Hazard | Severity | Likelihood | Risk Level | Mitigation |
|--------|----------|------------|------------|------------|
| Collision with person | Low (low mass, low speed) | Medium | Low | Ultrasonic sensor, emergency stop |
| Payload theft/tampering | Medium (medication security) | Medium | Medium | Enclosed compartment (no auth in prototype) |
| Radio interference/hijacking | Medium (loss of control) | Low | Low | Group-based radio isolation (no encryption) |
| Battery fire | High (lithium cells) | Very Low | Low | Protected battery enclosure, no charging during operation |
| Entrapment (fingers in wheels) | Low | Low | Very Low | Wheel guards on chassis |
| Software failure (runaway) | Medium | Low | Low | Emergency stop, obstacle avoidance as independent safety layer |

The risk assessment reveals that the highest residual risks relate to payload security and communication integrity — both areas where the prototype lacks the authentication and encryption features of commercial systems. For any progression toward clinical deployment, these risks would need to be reduced to ALARP (As Low As Reasonably Practicable) levels in accordance with the Management of Health and Safety at Work Regulations 1999 [15].

---

## 6. Conclusions & Future Work

### 6.1 Conclusions

This project has successfully demonstrated a working prototype of an autonomous electric vehicle for hospital delivery, meeting the primary aims set out in the project specification:

1. **Working prototype:** The vehicle operates reliably in three modes — manual wireless control, autonomous line-following, and emergency stop — demonstrating the core functionality required for hospital corridor delivery.

2. **Autonomous navigation:** The line-following system, while limited to a two-sensor bang-bang controller, successfully tracks a predefined route and demonstrates the principle of autonomous guided vehicle navigation applicable to hospital environments.

3. **Obstacle avoidance:** The ultrasonic sensor-based collision avoidance system provides a graduated response (speed reduction then stop) that takes priority over all other operational modes, implementing a fail-safe design philosophy consistent with BS EN ISO 13849-1 [2].

4. **Wireless override:** The compass-based manual control system provides intuitive remote operation with reliable emergency stop capability, demonstrating the manual override requirement critical for hospital safety.

5. **Secure payload delivery:** The enclosed compartment with servo-actuated access demonstrates the concept of secure containment, though authentication at the point of delivery remains an area for development.

The critical evaluation throughout this report has identified significant gaps between the prototype and production-grade hospital delivery robots. These gaps — in sensing capability, navigation complexity, payload capacity, security, and regulatory compliance — are expected and appropriate for an educational proof-of-concept. The prototype's value lies not in its readiness for clinical deployment but in its demonstration that the fundamental principles of autonomous hospital delivery can be implemented using low-cost, accessible hardware.

The project also highlights the importance of a holistic design approach: technical capability alone is insufficient for hospital robotics. As discussed in Section 1.2, the social acceptance, legal compliance, environmental impact, and cybersecurity of such systems are equally critical to their success. The design decisions made throughout this project — from the priority-based safety hierarchy to the traffic-light LED indicators — reflect an attempt to balance technical functionality with these broader considerations.

### 6.2 Future Work

Based on the limitations identified in this report, the following enhancements are recommended for future development:

1. **Enhanced navigation:** Migration from two-sensor line-following to a multi-sensor array (5+ sensors) with PID control, or ideally to camera-based or LIDAR-based SLAM navigation for map-free autonomous routing. This would address the current inability to handle junctions, complex routes, and dynamic obstacle avoidance (manoeuvring around obstacles rather than simply stopping).

2. **360° obstacle detection:** Addition of multiple ultrasonic or ToF sensors around the vehicle perimeter, or a low-cost LIDAR sensor (e.g., RPLIDAR A1), to provide surround awareness critical for safe operation in busy hospital corridors.

3. **Closed-loop motor control:** Integration of wheel encoders and a PID speed controller to achieve consistent, battery-independent speed control and accurate odometry for position estimation.

4. **Secure communication:** Migration from unencrypted micro:bit radio to BLE with pairing or Wi-Fi with TLS encryption, addressing the critical security vulnerability identified in Section 2.3.

5. **Payload authentication:** Implementation of RFID badge reader or PIN-code access at the compartment, ensuring chain-of-custody compliance with pharmaceutical handling regulations [10][11].

6. **Battery management:** Addition of battery voltage monitoring with automatic return-to-base behaviour at low charge, and investigation of autonomous docking and charging.

7. **Microcontroller upgrade:** For production development, migration from the BBC micro:bit to a more capable platform (e.g., Raspberry Pi with ROS, or STM32-based industrial controller) to support the computational demands of advanced navigation, multi-sensor fusion, and secure communications.

8. **Formal compliance:** Development within a structured SDLC (IEC 62304 [5]) and compliance with relevant standards including ISO 3691-4 [6] for driverless industrial trucks and the Medical Devices Regulations 2002 (as amended) [16] if the vehicle is classified as a medical device accessory.

---

## References (Technical Sections)

> **Note:** These references supplement the existing IEEE reference list covering Sections 1.1 and 1.2. Reference numbers should be reconciled during final document compilation.

[1] International Electrotechnical Commission, "IEC 61508: Functional Safety of Electrical/Electronic/Programmable Electronic Safety-Related Systems," IEC, Geneva, 2010.

[2] British Standards Institution, "BS EN ISO 13849-1:2015: Safety of Machinery — Safety-Related Parts of Control Systems — Part 1: General Principles for Design," BSI, London, 2015.

[3] National Institute of Standards and Technology, "NIST Special Publication 800-183: Networks of 'Things'," NIST, Gaithersburg, MD, 2016.

[4] Aethon Inc., "TUG Autonomous Mobile Robot — Technical Specifications and Operational Overview," Aethon, Pittsburgh, PA. [Online]. Available: https://aethon.com/products/. [Accessed: Mar. 18, 2026].

[5] International Electrotechnical Commission, "IEC 62304: Medical Device Software — Software Life Cycle Processes," IEC, Geneva, 2006 (amended 2015).

[6] International Organization for Standardization, "ISO 3691-4:2020: Industrial Trucks — Safety Requirements and Verification — Part 4: Driverless Industrial Trucks and Their Systems," ISO, Geneva, 2020.

[7] International Organization for Standardization, "BS EN ISO 12100:2010: Safety of Machinery — General Principles for Design — Risk Assessment and Risk Reduction," ISO, Geneva, 2010.

[8] J. Oyekan et al., "Automated Guided Vehicles in Healthcare: A Review," *Journal of Medical Robotics Research*, vol. 5, no. 2, pp. 1–15, 2020.

[9] International Electrotechnical Commission, "BS EN 60073:2002: Basic and Safety Principles for Man-Machine Interface, Marking and Identification — Coding Principles for Indicators and Actuators," IEC, Geneva, 2002.

[10] UK Government, "The Human Medicines Regulations 2012," Statutory Instrument 2012 No. 1916, London, 2012.

[11] UK Government, "Misuse of Drugs Act 1971," Chapter 38, London, 1971.

[12] Damien George et al., "MicroPython Documentation — Performance Considerations," MicroPython.org. [Online]. Available: https://docs.micropython.org/. [Accessed: Mar. 18, 2026].

[13] NXP Semiconductors, "PCA9685 — 16-channel, 12-bit PWM Fm+ I2C-bus LED Controller — Product Data Sheet," NXP, Eindhoven, Rev. 4, 2015.

[14] U.S. Food and Drug Administration, "510(k) Premarket Notification Database," FDA, Silver Spring, MD. [Online]. Available: https://www.accessdata.fda.gov/scripts/cdrh/cfdocs/cfpmn/pmn.cfm. [Accessed: Mar. 18, 2026].

[15] UK Government, "The Management of Health and Safety at Work Regulations 1999," Statutory Instrument 1999 No. 3242, London, 1999.

[16] UK Government, "The Medical Devices Regulations 2002," Statutory Instrument 2002 No. 618, London, 2002 (as amended).

---

*End of Technical Report — Sections 2–6*
