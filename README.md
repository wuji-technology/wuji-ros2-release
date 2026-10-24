# wuji_bringup

`wuji_bringup` starts Wuji Hand 2 and Wuji Glove drivers. Hand 2 launch files integrate descriptions and an optional RViz view. The Glove launch contains only its data driver. The package also installs the status and wave Demos described below.

## Wuji Hand 2 Launch

```bash
ros2 launch wuji_bringup hand2.launch.py \
  hand_side:=left \
  model_revision:=1 \
  namespace:=hand
```

| Argument | Default | Description |
|----------|---------|-------------|
| `hand_side` | None | Required `left` or `right` value |
| `model_revision` | `1` | `1` for Wuji Hand 2.1 or `2` for Wuji Hand 2.2 |
| `namespace` | `""` | Shared namespace for both nodes |
| `endpoint` | `""` | SDK endpoint for the selected hand |
| `serial_number` | `""` | Serial number for the selected hand |
| `connect_timeout` | `10.0` | Seconds to wait for a connection |
| `auto_enable` | `false` | Enables the hand after startup |
| `publish_joint_diagnostics` | `true` | Publishes joint diagnostics |
| `publish_imu` | `true` | Publishes IMU data |
| `publish_fingertip_tactile` | `false` | Publishes fingertip tactile data. Set it to `true` on a hand with fingertip tactile modules |

The launch file reads the URDF from the installed `wuji_hand2_1_description` or `wuji_hand2_2_description` package and doesn't copy the model files.

The launch file doesn't publish a static transform from the hand root to a robot or world frame. The model root is the arm-flange mount frame, `l_mount` or `r_mount`. Connect it in the parent robot's TF tree. Hand 2.2 descriptions include fingertip sensor frames. Hand 2.1 descriptions don't include them, and bringup doesn't create replacement frames.

## Wuji Glove Launch

Start one Glove data driver:

```bash
ros2 launch wuji_bringup glove.launch.py \
  serial_number:=<serial-number> namespace:=<namespace>
```

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `""` | Namespace for the driver and all relative topics |
| `endpoint` | `""` | Wuji SDK address for direct connection |
| `serial_number` | `""` | Serial number selector |
| `hand_side` | `""` | `left`, `right`, or empty |
| `connect_timeout` | `10.0` | Overall selection and connection timeout in seconds |
| `publish_emf_poses` | `true` | Publishes EMF poses |
| `publish_tip_poses` | `true` | Publishes fingertip poses |
| `publish_skeleton` | `true` | Publishes the hand skeleton |
| `publish_imu` | `true` | Publishes the raw palm IMU topic |
| `publish_tactile` | `true` | Publishes the tactile grid |
| `publish_tactile_residual` | `true` | Publishes the tactile residual grid |
| `publish_tactile_binary` | `true` | Publishes the tactile binary grid |
| `publish_tactile_zones` | `true` | Publishes tactile zones |
| `publish_tactile_points` | `true` | Publishes the tactile point cloud |

The launch starts one driver node without respawning it and shuts down when the driver exits. It doesn't load a model, publish transforms, start visualization, or expose control and calibration interfaces.

## Run the Demos

Use the view launch and Demos in this order. Run the launch command in one terminal, and run each Demo in another terminal after sourcing the workspace setup file.

`hand2_view.launch.py` accepts the `hand2.launch.py` arguments and also starts RViz. It requires `serial_number` and always runs with `auto_enable:=false`.

```bash
ros2 launch wuji_bringup hand2_view.launch.py \
  serial_number:=<serial-number> hand_side:=<left-or-right> \
  namespace:=<namespace>
```

```bash
ros2 run wuji_bringup hand2_status_demo --ros-args \
  -r __ns:=/<namespace>
```

Stop the view launch after the status check. Start the hand with control enabled:

```bash
ros2 launch wuji_bringup hand2.launch.py \
  serial_number:=<serial-number> hand_side:=<left-or-right> \
  namespace:=<namespace> auto_enable:=true
```

Run the wave Demo in another terminal:

```bash
ros2 run wuji_bringup hand2_wave_demo --ros-args \
  -r __ns:=/<namespace>
```

Use the same `namespace` for each command.

### Status Demo

`hand2_status_demo` counts joint state, joint diagnostics, and IMU frames for 10 seconds by default, and it prints each topic's average receive rate. It requires at least one 20-position joint state, one 20-entry joint diagnostics message, and one IMU message. It subscribes only to raw ROS topics and makes no control or configuration calls. The Demo doesn't independently verify message contents or the device serial number. The launch selector and ROS namespace determine the checked device.

The driver attempts to disable the hand during shutdown, so the complete launch lifecycle isn't physically read-only.

Pressing Ctrl+C ends the status Demo with a zero exit status after it releases its ROS resources.

### Glove Status Demo

With `glove.launch.py` running, use the same namespace in another terminal:

```bash
ros2 run wuji_bringup glove_status_demo --ros-args \
  -r __ns:=/<namespace>
```

`glove_status_demo` subscribes to all 10 relative Glove topics for 10 seconds by default. It prints the five fingers' joint angles and confidence for every `joint_angles` frame. At the end, it prints each topic's frame count and ROS receive rate. The rate uses the interval from the first callback through the last callback, so it excludes discovery and first-frame wait time. Receiving `joint_angles` makes the check succeed. Missing optional streams produce warnings without changing the exit status.

Set a different observation time with `--duration` before `--ros-args`:

```bash
ros2 run wuji_bringup glove_status_demo --duration 30 --ros-args \
  -r __ns:=/<namespace>
```

Add `--rate-only` before `--ros-args` to suppress per-frame joint angle output while measuring receive rates:

```bash
ros2 run wuji_bringup glove_status_demo --rate-only --ros-args \
  -r __ns:=/<namespace>
```

Receive rates describe frames processed by the Demo. They don't represent the device's source output rate when the ROS subscriber drops messages.

### Wave Demo

`hand2_wave_demo` is a minimal command publisher. It publishes anonymous 20-position `JointState` messages to the relative `joint_commands` topic at 100 Hz. The thumb and abduction targets stay at 0 rad. The four fingers move from 0 rad through 1.0 rad and back with a 0.5 Hz cosine wave.

The Demo doesn't read device state, call services, change control parameters, or disable the hand. Run it on an unloaded hand with `auto_enable=true`. Press Ctrl+C to stop the Demo, and then stop the launch process so the driver can disable the hand.

Before starting or restarting the Demo, confirm that the hand is unloaded and at its zero target. Each start publishes zero targets for 1 second before it begins the wave.

## Troubleshooting

### The Status Demo Fails

Confirm that `namespace` matches the view launch and that `publish_joint_diagnostics` and `publish_imu` remain `true`. The Demo requires at least one 20-position joint state, one 20-entry joint diagnostics message, and one IMU message.

### The Wave Demo Doesn't Move the Hand

Start `hand2.launch.py` with `auto_enable:=true`, and use the same namespace for the launch and Demo commands. Check the driver logs and device state if the hand doesn't move.

### The Glove Status Demo Reports No Joint Angles

Confirm that the launch is still running and that the Demo uses the same namespace. `joint_angles` is always enabled. Inspect the driver log for device-selection, connection, or required-stream errors.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
