# wuji_hand2_driver

`wuji_hand2_driver` connects one Wuji Hand 2 to ROS2. It publishes state and sensor streams, accepts joint commands, and exposes device controls.

## Run the Node

Select a device by serial number:

```bash
ros2 run wuji_hand2_driver wuji_hand2_node \
  --ros-args -p serial_number:=<serial-number>
```

You can also select a device with `endpoint` or `hand_side`. If no selector is set, the node connects only when discovery finds one unique Wuji Hand 2. It exits when discovery finds multiple candidates.

All topic and service names are relative. Use a ROS namespace to run more than one driver.

## Parameters

The node reads parameters only during startup.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `endpoint` | string | `""` | Wuji SDK address used for direct connection |
| `serial_number` | string | `""` | Selects a device by serial number |
| `hand_side` | string | `""` | Selects a `left` or `right` device. Empty doesn't filter by side |
| `connect_timeout` | double | `10.0` | Overall selection and connection timeout in seconds. Set to `0.0` to wait without a deadline |
| `auto_enable` | bool | `false` | Enables all joints after startup |
| `publish_joint_diagnostics` | bool | `true` | Starts the joint diagnostic stream |
| `publish_imu` | bool | `true` | Starts the IMU stream |
| `publish_fingertip_tactile` | bool | `false` | Starts the fingertip streams for fingers whose tactile module is online. Set it to `true` on a hand with fingertip tactile modules |

When both `serial_number` and `hand_side` are set, the serial number selects the device and the side validates it. When `endpoint` is set with another selector, the node connects to the endpoint and validates the supplied identity.

## Topics

| Topic | Type | Direction | QoS | Rate |
|-------|------|-----------|-----|------|
| `joint_states` | `sensor_msgs/msg/JointState` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |
| `joint_commands` | `sensor_msgs/msg/JointState` | Subscribed | Keep last 1, best effort, volatile | The Wuji SDK accepts up to 1 kHz |
| `joint_diagnostics` | `wuji_msgs/msg/JointDiagnosticsArray` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |
| `imu` | `sensor_msgs/msg/Imu` | Published | Keep last 5, best effort, volatile | About 100 Hz in the Wuji SDK contract |
| `fingertip_tactile/thumb` | `wuji_msgs/msg/FingertipTactile` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |
| `fingertip_tactile/index` | `wuji_msgs/msg/FingertipTactile` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |
| `fingertip_tactile/middle` | `wuji_msgs/msg/FingertipTactile` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |
| `fingertip_tactile/ring` | `wuji_msgs/msg/FingertipTactile` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |
| `fingertip_tactile/pinky` | `wuji_msgs/msg/FingertipTactile` | Published | Keep last 5, best effort, volatile | Device-dependent, and the Wuji SDK doesn't specify a rate |

The fingertip `point_forces` use N. `total_force` also uses N, and `temperature` uses degrees Celsius. IMU angular velocity uses rad/s, and linear acceleration uses m/s². The driver publishes the SDK frame ID and converts the SDK Unix timestamp in microseconds to the ROS timestamp.

Wuji Hand 2 currently doesn't provide an orientation estimate. The driver sets `orientation_covariance[0]` to `-1` to mark orientation unavailable.
`JointState.position` uses rad, `velocity` uses rad/s, and `effort` uses A as the current SDK contract. The effort field is a current-like torque proxy, not measured N·m.
Published `joint_states` entries follow the valid entries in each SDK frame. The driver keeps `name`, `position`, `velocity`, and `effort` aligned, but it doesn't promise dense-index ordering for state messages.

The `joint_diagnostics` topic directly publishes per-joint hardware data from the Wuji SDK, including status, error, electrical, temperature, and communication fields. It isn't a ROS standard health interface. The driver doesn't publish `/diagnostics`.

### Joint Diagnostics Fields

| Field | Unit | Semantics |
|-------|------|-----------|
| `nid` | None | SDK node identifier. The wrapper copies it without conversion |
| `name` | None | Formal URDF joint name mapped from `nid` and the selected hand side |
| `status_word` | None | Raw SDK `uint32` status word. The wrapper doesn't decode or alter it |
| `current` | A | Phase current reported by the SDK. The wrapper passes it through without ROS-side conversion |
| `bus_voltage` | V | Bus voltage reported by the SDK. The wrapper passes it through without ROS-side conversion |
| `mcu_temperature` | °C | Joint controller temperature reported by the SDK. The wrapper passes it through without ROS-side conversion |
| `error_code` | None | Raw SDK `uint16` error value. The wrapper passes it through without ROS-side decoding or reinterpretation |
| `comm_response_rate_pct` | % | Per-joint RS485 bus response rate over the last second, range 0–100 percent. The wrapper passes it through without ROS-side conversion |
| `comm_timeout_total` | Count | Cumulative per-joint RS485 bus timeout count from the SDK communication snapshot. The wrapper passes it through without ROS-side conversion |

`JointDiagnosticsArray.header` contains the converted SDK timestamp and frame ID. The array contains only valid, online joint entries from the SDK frame.

## Joint Commands

Named commands are the recommended input form. Set `name` to formal left-side or right-side URDF joint names. Set `position` to the same length. Named commands can target a subset of joints and can use any order.

Leave `name` empty only for a full 20-joint command. In this form, `position` must have exactly 20 entries in the dense order below. This ordering is a driver contract because ROS doesn't define an anonymous `JointState` order.

For either form, `velocity` and `effort` can be empty. The driver fills an empty field with zero. A nonempty field must match the command joint count. Unknown names, duplicate names, opposite-side names, invalid lengths, NaN, infinity, and values outside the SDK float range reject the complete command. The driver doesn't send a partial update.

| Dense Index | nid | Name Body |
|------------:|----:|-----------|
| 0 | 1 | `thumb_cmc_flex` |
| 1 | 2 | `thumb_cmc_abd` |
| 2 | 3 | `thumb_mcp` |
| 3 | 4 | `thumb_ip` |
| 4 | 6 | `index_finger_mcp_flex` |
| 5 | 7 | `index_finger_mcp_abd` |
| 6 | 8 | `index_finger_pip` |
| 7 | 9 | `index_finger_dip` |
| 8 | 11 | `middle_finger_mcp_flex` |
| 9 | 12 | `middle_finger_mcp_abd` |
| 10 | 13 | `middle_finger_pip` |
| 11 | 14 | `middle_finger_dip` |
| 12 | 16 | `ring_finger_mcp_flex` |
| 13 | 17 | `ring_finger_mcp_abd` |
| 14 | 18 | `ring_finger_pip` |
| 15 | 19 | `ring_finger_dip` |
| 16 | 21 | `pinky_mcp_flex` |
| 17 | 22 | `pinky_mcp_abd` |
| 18 | 23 | `pinky_pip` |
| 19 | 24 | `pinky_dip` |

Prefix each name body with `l_` or `r_` for the selected hand. The `nid` values identify incoming joint data. Dense indexes define full-array command and service order.

> [!CAUTION]
> The driver doesn't implement a command watchdog. It doesn't resend the last command, send zeros, trigger an emergency stop, or disable the hand when command input stops. The controlling application must publish at its required rate and perform an explicit stop operation.

## Services

Services use the default reliable ROS service QoS.

| Service | Type | Behavior |
|---------|------|----------|
| `set_enabled` | `wuji_msgs/srv/SetEnabled` | Enables or disables joints selected by a dense-order 20-entry mask |
| `clear_faults` | `wuji_msgs/srv/ClearFaults` | Clears faults for a dense-order 20-entry mask |
| `set_effort_limit` | `wuji_msgs/srv/SetEffortLimit` | Sets one nonnegative finite SDK current limit in A for all joints |
| `set_effort_limits` | `wuji_msgs/srv/SetEffortLimits` | Sets 20 nonnegative finite SDK current limits in A, in dense order |
| `set_mit_params` | `wuji_msgs/srv/SetMitParams` | Sets 20 nonnegative finite SDK-native `kp` and `kd` values in dense order. The SDK does not define physical units for either value |
| `emergency_stop` | `std_srvs/srv/Trigger` | Calls the Wuji SDK whole-hand hard stop |
| `calibrate_tactile` | `std_srvs/srv/Trigger` | Starts five-finger tactile calibration and polls readiness against a 30-second deadline |

A mask with all entries set to `false` is a valid no-op. Fixed-array services don't accept joint names or partial arrays. A failed command or service reports an error but doesn't stop the node.

`emergency_stop` doesn't create a driver-side latch. A later operation reaches the SDK normally.

Rejected commands and SDK send failures produce throttled ROS error logs. A command failure doesn't stop, disable, or latch the node.

Before calling `calibrate_tactile`, secure the hand and keep all sensor surfaces unloaded. Calibration doesn't disable joints or pause data streams. The driver checks the 30-second polling deadline before each synchronous status call and between polling passes. It retries an individual Wuji SDK status timeout before that deadline. An in-flight synchronous SDK call can extend the service wall time beyond 30 seconds. Other SDK errors end the service call.

## Lifecycle and Health

Startup validates parameters, connects one device, creates the command publisher, starts required `joint_states`, creates controls, starts optional streams, and applies `auto_enable` when requested. A required startup failure stops the node. An optional stream failure disables only that stream.

The node uses a 50 ms timer to transfer latched stream faults from SDK callbacks to the ROS executor. The timer isn't a watchdog and doesn't track frame timing or counts. The node exits with a nonzero status when required `joint_states` ends or reports an error. An optional stream terminal event removes that topic and writes a warning to the process log.

On shutdown, the node stops ROS controls, closes every SDK subscription and the command publisher, attempts to disable all joints, and disconnects. The final disable is best effort. A normal signal shutdown returns zero.

## Hardware Tests

Hardware tests are disabled by default. Run them only with a selected device and an operator present.

The read-only test scans, connects, and observes streams:

```bash
WUJI_HARDWARE_TESTS=1 \
WUJI_HARDWARE_SERIAL=<serial-number> \
./scripts/run_hardware_tests.sh --read-only
```

It skips the fingertip tactile streams by default. Set `WUJI_HARDWARE_FINGERTIP_TACTILE=1` on a hand with fingertip tactile modules to check them too.

The mutating test changes device state. Secure the hand in a safety fixture, keep sensor surfaces unloaded, keep people clear of moving parts, and keep an operator present for the full run. The test doesn't restore the prior enable, effort-limit, MIT, fault, emergency-stop, or tactile state.

```bash
WUJI_HARDWARE_TESTS=1 \
WUJI_MUTATING_HARDWARE_TESTS=1 \
WUJI_HARDWARE_SAFETY_ACK=fixture-unloaded-operator-state-not-restored \
WUJI_HARDWARE_SERIAL=<serial-number> \
./scripts/run_hardware_tests.sh --mutating
```

Don't use `WUJI_HARDWARE_ENDPOINT` for mutating tests. Fatal signals or forced process termination can prevent cleanup and final safety notices.

## Troubleshooting

- If selection times out, run `ros2 run wuji_tools wuji_list` and set an explicit `serial_number` or `endpoint`.
- If the node reports a side mismatch, check the selected device and `hand_side`.
- If a fingertip topic stops, inspect the process log for metadata, payload, digest, or stream errors.
- If the node exits during operation, inspect the process log for the first `joint_states` failure.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
