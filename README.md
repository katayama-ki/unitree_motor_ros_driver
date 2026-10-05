# unitree_motor_ros_driver

ROS 1 Noetic driver for Unitree **A1**, **B1**, and **GO-M8010-6** motors using the
bundled official `unitree_actuator_sdk`. One node owns one serial bus; multiple
motors on that bus share one communication thread. Linux x86_64 and aarch64 are
supported by the bundled SDK libraries.

## Build and run

Place this package in a catkin workspace and build in a Noetic environment:

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make --only-pkg-with-deps unitree_motor_ros_driver
source devel/setup.bash
roslaunch unitree_motor_ros_driver motor_driver.launch \
  serial_port:=/dev/ttyUSB0 motor_type:=GO-M8010-6
```

The process needs permission to open the serial device (typically membership in
`dialout`). For another bus, launch another node with a different `serial_port`
and `node_name`. A bus uses one motor type. Startup sends a zero-parameter BRAKE
to each scanned ID, retries a failed or mismatched reply once, and logs the
detected IDs. No detected motors is a fatal startup error. Startup faults latch
only the affected motor; other motors remain available.

## Topics

Both topics are private: with the default node name, they are
`/unitree_motor_driver/command` and `/unitree_motor_driver/state`.

| Topic | Message | Meaning |
| --- | --- | --- |
| `~command` | `MotorCommand` | `uint8 id`; `float32 position, velocity, torque, kp, kd` |
| `~state` | `MotorState` | Receive timestamp, ID, normalized mode, position, velocity, torque, temperature, raw motor error |

All physical command and feedback values are **output-side**: rad, rad/s, Nm,
Nm/rad, and Nm/(rad/s). Commands always describe FOC; the driver manages modes.
State mode constants are `BRAKE=0`, `FOC=1`, `CALIBRATE=2`, and `UNKNOWN=255`,
independent of the SDK's raw mode values. Temperature is `int8` degrees Celsius;
error is `uint8` raw `merror`.

Example zero FOC command to detected motor ID 0:

```bash
rostopic pub -r 100 /unitree_motor_driver/command \
  unitree_motor_ros_driver/MotorCommand \
  '{id: 0, position: 0.0, velocity: 0.0, torque: 0.0, kp: 0.0, kd: 0.0}'
```

This actively compensates motor friction. Choose `timeout_action` to match the
mechanism: `zero_torque` actively cancels friction; `damping` resists motion;
`brake` uses the protocol's "locked" mode, whose physical behavior needs checking
on the actual motor. Shutdown always attempts BRAKE on every detected motor,
logs each acknowledgment/failure, then closes the port.

## Parameters

Every private parameter is exposed as an argument in `motor_driver.launch`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `serial_port` | required | Serial device path |
| `motor_type` | required | `A1`, `B1`, or `GO-M8010-6` |
| `motor_id_scan_max` | 14 | Scan inclusively from ID 0; GO capped at 14; A1/B1 accept up to 255 and skip broadcast ID 187 (`0xBB`) |
| `command_timeout` | 0.5 | Seconds since latest valid command |
| `active_io_rate` | 100 | Transactions/s per ACTIVE motor |
| `idle_io_rate` | 10 | Transactions/s per IDLE, CMD_TIMED_OUT, or FAULT_LATCHED motor |
| `io_rate_warn_ratio` | 0.9 | Warn below this fraction of IO target; one-second measurement window |
| `io_retry_rate` | 0 | Transactions/s while io_error; zero disables retries until restart |
| `active_state_pub_rate` | 100 | Publication target per ACTIVE motor |
| `idle_state_pub_rate` | 10 | Publication target per other control state |
| `timeout_action` | `zero_torque` | `zero_torque`, `damping`, or `brake` |
| `timeout_damping_kd` | 1.0 | Output-side damping gain; for damping it must be positive, within kd_limit, and representable by the SDK |
| `velocity_limit` | 0 | Absolute output rad/s limit; zero or above hardware maximum uses hardware maximum |
| `torque_limit` | 0 | Absolute output Nm limit; same hardware policy |
| `kp_limit` | 200 | Nonnegative output stiffness limit |
| `kd_limit` | 10 | Nonnegative output damping limit |
| `warn_temperature` | 70 | Warn when temperature is strictly higher |
| `stop_temperature` | 80 | Latch a fault when temperature is strictly higher |
| `stop_on_motor_error` | true | Latch a fault on nonzero merror |

Rates/publication rates and command_timeout must be finite and positive
(`1e-9` through `1e9`); io_retry_rate additionally accepts zero. Limit parameters
must be finite and nonnegative. `io_rate_warn_ratio` must be in `(0, 1]`;
temperature thresholds must be in the feedback range `[-128, 127]`, with warn
strictly lower than stop. Invalid parameters cause failure before serial open.
Publication targets above IO targets produce startup warnings.

Hardware output limits and SDK gear ratios:

| Motor | Gear ratio | Velocity (rad/s) | Torque (Nm) |
| --- | --- | --- | --- |
| A1 | 9.1 | 21 | 33.5 |
| B1 | 8.66 | 15.55 | 140 |
| GO-M8010-6 | 6.33 | 30 | 23.7 |

Rotor commands use `q_out*r`, `dq_out*r`, `tau_out/r`, and gains divided by `r²`.
A1/B1 additionally use `kp/26.07` and `kd*100`, as documented in
[the SDK README](unitree_actuator_sdk/README.md). The SDK encodes B1 kd with x512
and A1 kd with x1024. GO speed and position encoding includes division by 6.2832;
gains use division by 25.6. Input validation follows the actual bundled SDK
encoding/saturation bounds and also checks float rounding at integer boundaries.

## State and communication behavior

- Initially IDLE, each motor sends BRAKE at idle_io_rate. Its first valid command
  moves it to ACTIVE. Invalid/nonfinite commands, undetected/broadcast IDs, and
  values outside configured or protocol limits are ignored and do not feed the
  watchdog. Position has no configured limit, but must fit the protocol.
- An expired ACTIVE command moves to CMD_TIMED_OUT and sends timeout_action at
  idle_io_rate. A new valid command restores ACTIVE.
- Motor errors (when enabled) or excessive temperature set FAULT_LATCHED and
  send zero-parameter BRAKE until node restart. Valid commands are still recorded
  but cannot clear the fault. Safety/error logging runs after every good receive,
  even if publication is rate-limited.
- Two consecutive failed sendRecv calls or mismatched reply IDs set io_error.
  Retry rate overrides the control state's IO rate. A single correct reply
  resets the failure count and clears io_error. Retries use the current control
  state and latest command, including watchdog changes during communication loss.
- Scheduler deadlines, watchdogs, publication limits, and two-second per-ID/per-
  log-type throttles use steady time. Late cycles are dropped; no catch-up burst
  occurs. Control state changes make that motor immediately due, subject to the
  io_error retry override. SDK response waits can block the whole bus for 40 ms
  or more; IO targets depend on motor count, replies, and machine load.
- IO performance counts **transaction attempts**, including failed or mismatched
  replies, in one-second windows. Measurement windows restart when the target
  rate changes. IO-error motors are excluded from active/idle rate warnings.
- States are published only after successful matching replies; no cached state
  timer exists. Each motor has its own publication limit. ROS time is captured
  immediately after SDK receive, while scheduling remains independent of ROS time.
  Repeated receive timestamps (e.g. paused simulated time) are suppressed per motor.

## Implementation and tests

`model.hpp` / `model.cpp` contain conversions, validation, control state, and
deadline rules. `bus_driver` owns all motor contexts under one mutex, released
for each blocking transaction. `sdk_transport` adapts official SDK sendRecv;
`ros_node.cpp` is the only ROS-dependent source. The transport and logging/state
callbacks can be replaced for a future ROS 2 adapter or hardware-free tests.

```bash
catkin_make run_tests_unitree_motor_ros_driver
catkin_test_results
```

Core tests exercise faults, watchdogs, scan/reply validation, retry behavior,
thread ownership, lock release during IO, scheduling, conversion, and shutdown.
SDK protocol tests compare the actual binary's encoded packets and gear ratios.
The ROS integration test emulates two GO motors on a pseudo-terminal and runs
the real ROS node and SDK, checking topic fields, gain encoding, startup fault
isolation, invalid-command watchdog behavior, and shutdown BRAKE packets.
A test-only preload emulates UART low-latency ioctls on that named PTY; it is
never linked into the production driver. Actual RS-485 transport timing is not emulated.
These tests do not replace verification of timing and physical behavior on motors.

The driver is MIT licensed; the bundled SDK retains its own
[license](unitree_actuator_sdk/LICENSE).
