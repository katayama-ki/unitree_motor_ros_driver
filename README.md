# unitree_motor_ros_driver

A ROS 1 Noetic driver for Unitree motors using the bundled official
`unitree_actuator_sdk`. The SDK supports A1, B1, and GO-M8010-6 motors,
but this driver has only been tested with GO-M8010-6.

## Usage

### Startup and motor discovery

At startup, the driver scans motor IDs from 0 through `motor_id_scan_max` and
sends BRAKE commands during discovery. Only motors detected during this scan
can be controlled. Startup fails if no motors are detected.

### Topics

The driver subscribes to `~command` and publishes `~state`.
All topics use private names and are resolved under `/unitree_motor_driver` when the
default node name is used.

| Topic | Message | Fields |
| --- | --- | --- |
| `~command` | `MotorCommand` | `id`, `position`, `velocity`, `torque`, `kp`, `kd` |
| `~state` | `MotorState` | `header`, `id`, `mode`, `position`, `velocity`, `torque`, `temperature`, `error` |

Commands must be published periodically to keep a motor active.
Otherwise, the configured `command_timeout` applies.

All physical command and feedback values are expressed on the **output side**
(not the rotor side): position (rad), velocity (rad/s), torque (N·m),
kp (position stiffness, N·m/rad), and kd (damping coefficient, N·m·s/rad).

#### MotorCommand examples

Set `id` to the target motor's ID. All other unspecified fields are zero.

| Purpose | Example parameters |
| --- | --- |
| Track a target position | `position=3.14, kp=1, kd=1` |
| Track a target velocity | `velocity=3.14, kd=1` |
| Damping (resist external motion) | `kd=1` |
| Apply constant feedforward torque | `torque=1` |
| Velocity control with torque feedforward | `velocity=3.14, torque=1, kd=0.1` |
| Hybrid control | `position=1, velocity=1, torque=1, kp=1, kd=1` |
| Zero-torque mode (active friction compensation) | All zero |

#### MotorState

`MotorState` provides feedback from each motor.

`mode` indicates the motor operating mode (normalized by the driver).

| Value | Description |
| --- | --- |
| `BRAKE=0` | Locked (brake) |
| `FOC=1` | FOC closed-loop control |
| `CALIBRATE=2` | Encoder calibration |
| `UNKNOWN=255` | Unknown mode |

`error` is the raw motor error code (`merror`) from the SDK.
For GO-M8010-6:

| Value | Description |
| --- | --- |
| `0` | Normal |
| `1` | Overheating |
| `2` | Overcurrent |
| `3` | Overvoltage |
| `4` | Encoder fault |
| `5–7` | Reserved |

For A1 and B1, `merror` is available, but the meanings of its error codes remain unverified.

### Launch

```sh
roslaunch unitree_motor_ros_driver motor_driver.launch \
  serial_port:=/dev/ttyUSB0 motor_type:=GO-M8010-6
```

Every private parameter is exposed as an argument in `motor_driver.launch`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `serial_port` | required | Serial device path, such as `/dev/serial/by-id/<device-id>` |
| `motor_type` | required | `A1`, `B1`, or `GO-M8010-6` |
| `motor_id_scan_max` | 14 | Highest ID to scan at startup (inclusive); GO capped at 14; A1/B1 up to 255, excluding broadcast ID 187 (`0xBB`) |
| `command_timeout` | 0.5 | Time (s) without a valid command before `timeout_action` is applied to an ACTIVE motor |
| `timeout_action` | `zero_torque` | Action on command timeout: `zero_torque`, `damping`, or `brake` |
| `timeout_damping_kd` | 1.0 | Damping gain (N·m·s/rad) used when `timeout_action=damping` |
| `active_io_rate` | 100 | Target I/O transaction rate per ACTIVE motor (Hz) |
| `idle_io_rate` | 10 | Target I/O transaction rate per other motor (Hz), unless `io_error` is set |
| `io_rate_warn_ratio` | 0.9 | Warn when the measured I/O attempt rate falls below this fraction of its target; measured over approximately one second |
| `io_retry_rate` | 0 | Retry rate (Hz) after `io_error` is set; zero disables automatic retries|
| `active_state_pub_rate` | 100 | Target state publication rate (Hz) per ACTIVE motor |
| `idle_state_pub_rate` | 10 | Target state publication rate (Hz) per motor in any other control state |
| `velocity_limit` | 0 | Reject commands exceeding this velocity limit (rad/s); B1 requires an explicit positive value. For other motors, zero uses the motor profile limit; values above the profile limit are capped |
| `torque_limit` | 0 | Reject commands exceeding this torque limit (N·m); zero uses the profile limit; values above the profile limit are capped |
| `kp_limit` | 200 | Reject commands exceeding this kp limit (N·m/rad) |
| `kd_limit` | 10 | Reject commands exceeding this kd limit (N·m·s/rad) |
| `warn_temperature` | 70 | Issue a warning when motor temperature feedback exceeds this value (°C) |
| `stop_temperature` | 80 | Latch a fault and request BRAKE when temperature feedback exceeds this value (°C) |
| `stop_on_motor_error` | true | Latch a fault and request BRAKE when a nonzero motor error is detected |

*CAUTION*: `velocity_limit`, `torque_limit`, `kp_limit`, and `kd_limit` validate
individual `MotorCommand` fields. They do not guarantee corresponding limits on 
actual motor motion or total closed-loop torque. Feedback exceeding `velocity_limit` 
or `torque_limit` is logged as a warning but does not by itself stop the motor.

## Specifications

The driver provides low-level motor control, communication, and fault handling.
Multi-motor coordination and system-level safety are the responsibility of
higher-level controllers.

- **Independent motor control:**
  Commands, timeouts, and fault states are managed independently for each motor.
  A fault in one motor does not automatically stop other motors.
  Broadcast commands are not supported.

- **Command handling:**
  Motors remain in BRAKE until the first valid command is received.
  Invalid commands are ignored and do not reset the command timeout.

- **Command timeout:**
  When valid commands stop arriving, the driver applies the configured `timeout_action`.
  Normal control resumes when a new valid command is received.
  The default `zero_torque` requests FOC control with all command values set to zero; it does not stop motor motion.

- **Motor faults:**
  Motor errors (when enabled) or excessive temperature cause the driver to request
  BRAKE and latch the fault until node restart, preventing unintended automatic recovery.

- **Command limits:**
  Configured limits validate individual command fields, not actual motor motion
  or output torque. In particular, the torque limit does not constrain the
  total torque generated by feedback control.

- **Communication rates:**
  The motor communication protocol requires a blocking command-response
  transaction for each motor. Communication with motors sharing a serial bus is
  therefore performed sequentially, not simultaneously.
  Configured I/O rates are best-effort targets, not guarantees.
  Achievable rates depend on motor count, communication latency, and errors.

- **Communication errors:**
  Communication failures are reported through `io_error`, with optional
  automatic retries. A failed `sendRecv()` may block for approximately 20 ms
  without a reply, or 40 ms if a partial reply is received.
  Repeated retries to an unresponsive motor can significantly disrupt
  communication with other motors on the same bus.
  Therefore, io_retry_rate defaults to 0 (automatic retries disabled).
  With this setting, no further communication attempts are made to the affected motor
  during normal operation once `io_error` is set.

- **Recovery behavior:**
  When automatic retries are enabled, the latest command is resent if the motor
  remains ACTIVE without a fault or command timeout.
  This may cause sudden motion when communication recovers.
  Higher-level controllers are responsible for ensuring safe recovery.

- **Feedback and monitoring:**
  `MotorState` is published only after successful communication, avoiding stale
  feedback.

- **Safety limitations:**
  BRAKE commands may not reach the motor during communication failures.
  This driver does not replace independent system-level safety measures.
