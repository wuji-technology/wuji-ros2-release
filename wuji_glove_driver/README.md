# wuji_glove_driver

`wuji_glove_driver` connects one Wuji Glove to ROS2 and publishes its SDK data streams. It is a thin data adapter that preserves SDK timestamps, frame identifiers, and payload values. It doesn't create subscriptions, services, or actions, and it doesn't load a model, publish transforms, expose controls, or provide calibration interfaces.

## Run the Node

Select a device by serial number:

```bash
ros2 run wuji_glove_driver wuji_glove_node \
  --ros-args -p serial_number:=<serial-number>
```

You can also use the installed launch file:

```bash
ros2 launch wuji_bringup glove.launch.py \
  serial_number:=<serial-number> namespace:=<namespace>
```

All topic names are relative. Use a ROS namespace when running more than one driver.

## Device Selection

The node applies selectors in this priority order:

1. `endpoint`
2. `serial_number`
3. `hand_side`
4. Unique Wuji Glove discovery

An endpoint selects the connection directly. Any supplied serial number or hand side then validates the connected device. When both serial number and hand side are set, the serial number selects the device and the hand side validates it.

With no selector, the node connects only after discovery finds one unique Wuji Glove. It keeps waiting when none are found and fails when multiple candidates are found. Set `serial_number` to disambiguate multiple devices. Selection, connection, identity reads, and retries share the `connect_timeout` deadline.

## Parameters

The node reads these 13 parameters only during startup. `joint_angles` is always enabled and has no publish parameter.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `endpoint` | string | `""` | Wuji SDK address used for direct connection |
| `serial_number` | string | `""` | Selects a device by serial number |
| `hand_side` | string | `""` | Selects `left` or `right`. Empty doesn't filter by side |
| `connect_timeout` | double | `10.0` | Overall timeout in seconds. Set to `0.0` to wait until ROS shutdown |
| `publish_emf_poses` | bool | `true` | Starts the EMF pose stream |
| `publish_tip_poses` | bool | `true` | Starts the fingertip pose stream |
| `publish_skeleton` | bool | `true` | Starts the hand-skeleton stream |
| `publish_imu` | bool | `true` | Starts the raw palm IMU stream |
| `publish_tactile` | bool | `true` | Starts the tactile grid stream |
| `publish_tactile_residual` | bool | `true` | Starts the tactile residual stream |
| `publish_tactile_binary` | bool | `true` | Starts the tactile binary stream |
| `publish_tactile_zones` | bool | `true` | Starts the tactile-zone stream |
| `publish_tactile_points` | bool | `true` | Starts the tactile point-cloud stream |

`hand_side` accepts `left`, `right`, or an empty value. `connect_timeout` must be finite and nonnegative.

## Topics

All 10 topics use keep-last 5, best-effort, volatile sensor-data QoS. Output rates are device-dependent because the Wuji SDK doesn't specify them.

| Topic | Type | Policy | Enable Parameter |
|-------|------|--------|------------------|
| `joint_angles` | `wuji_msgs/msg/HandJointAngles` | Required | Always enabled |
| `emf_poses` | `wuji_msgs/msg/TrackedPoseArray` | Optional | `publish_emf_poses` |
| `tip_poses` | `wuji_msgs/msg/TrackedPoseArray` | Optional | `publish_tip_poses` |
| `hand_skeleton` | `wuji_msgs/msg/TrackedPoseArray` | Optional | `publish_skeleton` |
| `imu/palm` | `sensor_msgs/msg/Imu` | Optional | `publish_imu` |
| `tactile` | `wuji_msgs/msg/TactileGrid` | Optional | `publish_tactile` |
| `tactile/residual` | `wuji_msgs/msg/TactileGrid` | Optional | `publish_tactile_residual` |
| `tactile/binary` | `wuji_msgs/msg/TactileGrid` | Optional | `publish_tactile_binary` |
| `tactile/zones` | `wuji_msgs/msg/TactileZoneArray` | Optional | `publish_tactile_zones` |
| `tactile/points` | `sensor_msgs/msg/PointCloud2` | Optional | `publish_tactile_points` |

The driver maps the SDK `imu_raw/palm` resource to `imu/palm`. It doesn't publish the SDK-derived `imu_data/palm` resource.

The wrapper converts without deriving new values. It preserves SDK IMU fields, complete skeleton and tactile arrays, and self-described point-cloud fields. `HandJointAngles` preserves each finger's valid angle count and confidence.

## Fault and Shutdown Behavior

The required `joint_angles` stream must start successfully. If it ends or reports an error while running, the node exits with a nonzero status. An optional stream that can't start or later ends is stopped and reported without terminating the other streams.

The driver doesn't reconnect in process. On shutdown it closes every subscription in reverse order, disconnects the device, and releases the SDK runtime. Callback code doesn't close its own subscription.

## Status Demo

Start `glove.launch.py`, then run the read-only status Demo with the same namespace:

```bash
ros2 run wuji_bringup glove_status_demo --ros-args \
  -r __ns:=/<namespace>
```

The Demo prints the five fingers' angles and confidence for every `joint_angles` frame. After 10 seconds, it prints each topic's frame count and ROS receive rate. The rate uses the interval from the first callback through the last callback, so it excludes discovery and first-frame wait time. It succeeds when at least one `joint_angles` frame arrives. Missing optional streams produce warnings.

Use `--duration <seconds>` before `--ros-args` to change the observation time.

Add `--rate-only` before `--ros-args` to suppress per-frame joint angle output while measuring receive rates. Receive rates describe frames processed by the Demo, not the device's source output rate.

## Hardware Tests

Glove hardware tests are read-only and disabled by default. Build with `BUILD_TESTING=ON`, select a device, and explicitly opt in:

```bash
WUJI_GLOVE_HARDWARE_TESTS=1 \
WUJI_GLOVE_HARDWARE_SERIAL=<serial-number> \
./scripts/run_hardware_tests.sh --glove-read-only
```

You can set `WUJI_GLOVE_HARDWARE_ENDPOINT` or `WUJI_GLOVE_HARDWARE_HAND_SIDE` instead. With no selector, the runner uses unique-device discovery. The check only scans, connects, validates identity, subscribes to enabled streams, and observes frames. It doesn't change device state.

## Troubleshooting

- If selection times out, run `ros2 run wuji_tools wuji_list` and set `serial_number`, `endpoint`, or `hand_side`.
- If multiple devices match, set an explicit `serial_number`.
- If the node exits during operation, inspect the process log for the first `joint_angles` fault.
- If an optional topic stops, inspect the process log for that stream's terminal event.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
