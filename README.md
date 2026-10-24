# wuji-ros2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-ros2)](https://github.com/wuji-technology/wuji-ros2/releases)

Wuji ROS2 packages connect Wuji Hand 2 and Wuji Glove to the ROS graph. The repository provides source-built drivers, public interfaces, launch integration, device listing, and a vendor package that fetches the Wuji SDK C library. The wrapper keeps device-specific behavior small and uses standard ROS messages where practical.

## Supported Environment

| Area | Supported |
|------|-----------|
| ROS2 | Lyrical |
| Operating system | Ubuntu Linux |
| Architecture | x86_64 and aarch64 |
| Device | Wuji Hand 2 and Wuji Glove |
| Description | Wuji Hand 2.1 and Wuji Hand 2.2 |
| Installation | Source build |

## Repository Structure

```text
├── .github/workflows/              // CI and release workflows
├── driver_common/                  // Shared internal driver infrastructure
├── scripts/                        // Description installer and hardware test entry points
├── test/                           // Repository documentation and hardware-gate tests
├── wuji_bringup/                   // Launch files, RViz config, and Demos
├── wuji_glove_driver/              // Wuji Glove ROS2 driver
├── wuji_hand2_driver/              // Wuji Hand 2 ROS2 driver
├── wuji_msgs/                      // Wuji ROS messages and services
├── wuji_sdk_vendor/                // Wuji SDK C vendor package
├── wuji_tools/                     // Device command-line tools
├── .clang-format                   // C++ formatting rules
└── README.md
```

## Quick Start

### Installation

Clone the repository, install the ROS dependencies, and build the production packages. Run `sudo rosdep init` once if rosdep has never been set up on this machine:

```bash
git clone https://github.com/wuji-technology/wuji-ros2.git
cd wuji-ros2

source /opt/ros/lyrical/setup.bash
sudo apt-get update
sudo apt-get install -y ros-dev-tools
rosdep update
scripts/install_hand2_descriptions.sh
rosdep install --from-paths . --ignore-src -r -y

colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`scripts/install_hand2_descriptions.sh` installs the Wuji Hand 2 description packages from the ROS 2 testing apt repository until they are available in the main ROS 2 apt repository. The script adds the testing repository only for these two packages and removes it before it exits, so other ROS packages on the machine still come from the main repository.

The build downloads and verifies the Wuji SDK archive for the active architecture. For an offline build, extract a matching Wuji SDK C release and add `-DWUJI_SDK_ROOT=<sdk-root>` to the CMake arguments.

### List Devices

List reachable devices before choosing a selector:

```bash
ros2 run wuji_tools wuji_list
```

Use the reported serial number to select one device explicitly.

### Start the Driver

Start one Wuji Hand 2 driver node:

```bash
ros2 run wuji_hand2_driver wuji_hand2_node \
  --ros-args -p serial_number:=<serial-number>
```

For a URDF, robot state publisher, and optional RViz view, use the bringup launch files instead:

```bash
ros2 launch wuji_bringup hand2_view.launch.py \
  serial_number:=<serial-number> hand_side:=<left-or-right> \
  namespace:=<namespace>
```

Start one Wuji Glove driver with its data-only launch:

```bash
ros2 launch wuji_bringup glove.launch.py \
  serial_number:=<serial-number> namespace:=<namespace>
```

When only one Wuji Glove is reachable, all selectors can remain empty. See the [`wuji_glove_driver` guide](wuji_glove_driver/README.md) for its 10 published topics and selection rules.

### Demos and Safety

With the view launch running, check the required raw topics and print received-data statistics:

```bash
ros2 run wuji_bringup hand2_status_demo --ros-args \
  -r __ns:=/<namespace>
```

Wuji Hand 2 currently doesn't provide an orientation estimate. The driver marks orientation unavailable with `orientation_covariance[0] = -1`.

Stop the view launch, then start the driver with control enabled before running the wave Demo:

```bash
ros2 launch wuji_bringup hand2.launch.py \
  serial_number:=<serial-number> hand_side:=<left-or-right> \
  namespace:=<namespace> auto_enable:=true
```

```bash
ros2 run wuji_bringup hand2_wave_demo --ros-args \
  -r __ns:=/<namespace>
```

> [!CAUTION]
> Run motion Demos on an unloaded, secured hand with an operator present. Keep people clear of moving parts. The driver doesn't implement a command watchdog, so stop the Demo and then stop the launch process to let the driver disable the hand.

Before starting or restarting the wave Demo, confirm that the hand is unloaded and at its zero target. Each start publishes zero targets for 1 second before it begins the wave. See the [`wuji_bringup` guide](wuji_bringup/README.md) for the full launch arguments, Demo behavior, and troubleshooting.

The Wuji Glove status Demo is read-only. It prints every received joint-angle frame, then reports frame counts and average receive rates:

```bash
ros2 run wuji_bringup glove_status_demo --ros-args \
  -r __ns:=/<namespace>
```

## Packages

| Package | Purpose |
|---------|---------|
| [`wuji_glove_driver`](wuji_glove_driver/README.md) | Publishes Wuji Glove joint, pose, IMU, tactile, and point-cloud data |
| [`wuji_hand2_driver`](wuji_hand2_driver/README.md) | Publishes Wuji Hand 2 data and exposes command and configuration endpoints |
| [`wuji_msgs`](wuji_msgs/README.md) | Defines pose, joint-angle, tactile, raw joint diagnostic, and control interfaces |
| [`wuji_sdk_vendor`](wuji_sdk_vendor/README.md) | Installs the Wuji SDK C header, shared library, license, and CMake target |
| [`wuji_tools`](wuji_tools/README.md) | Provides the `wuji_list` device scanner |
| [`wuji_bringup`](wuji_bringup/README.md) | Starts Wuji Hand 2 or Wuji Glove drivers and installs status and wave Demos |

See the [Wuji Hand 2 driver interfaces](wuji_hand2_driver/README.md) and [Wuji Glove driver interfaces](wuji_glove_driver/README.md) for topics, parameters, lifecycle behavior, and hardware tests. See the [Wuji SDK reference](https://docs.wuji.tech/docs/wuji-hand/latest/sdk-reference/) for the underlying device API.

## Troubleshooting

1. **The build can't find a description package**

   Run `sudo rosdep init` if rosdep has never been set up on this machine. Then run `rosdep update`, `scripts/install_hand2_descriptions.sh`, and `rosdep install --from-paths . --ignore-src -r -y`, and rebuild.

2. **The Wuji SDK download fails**

   Check access to GitHub Releases. For an offline build, provide an extracted SDK with `-DWUJI_SDK_ROOT=<sdk-root>`.

3. **The driver reports multiple matching devices**

   Run `wuji_list` and set one explicit `serial_number`. You can also select by `endpoint` or `hand_side`.

4. **The status Demo reports missing data**

   Use the same namespace for the launch and Demo. For Wuji Hand 2, keep `publish_joint_diagnostics` and `publish_imu` enabled. For Wuji Glove, `joint_angles` is required and missing optional streams only produce warnings.

5. **The wave Demo doesn't move the hand**

   Start `hand2.launch.py` with `auto_enable:=true`, use the same namespace, and inspect the driver log for rejected commands or SDK errors.

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) for contribution requirements and the complete verification commands. Hardware-changing tests require the documented opt-ins and operator safety setup.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
