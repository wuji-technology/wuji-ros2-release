# Contributing to wuji-ros2

Thank you for contributing to the Wuji ROS2 wrapper.

## Before You Start

The repository contains six ROS2 packages:

- `wuji_bringup`
- `wuji_hand2_driver`
- `wuji_glove_driver`
- `wuji_msgs`
- `wuji_sdk_vendor`
- `wuji_tools`

Keep source, tests, and user documentation within this repository root. Review [Installation](README.md#installation) before changing package dependencies or build configuration.

## Contribution Requirements

- Keep changes self-contained, and don't reference files outside this repository.
- Use C++17 and follow `.clang-format`.
- Add or update focused tests for behavior changes.
- Run the verification commands below before opening a pull request.
- Don't run hardware tests without the opt-ins and safety setup in the [Wuji Hand 2](wuji_hand2_driver/README.md#hardware-tests) and [Wuji Glove](wuji_glove_driver/README.md#hardware-tests) hardware test guides.

## Verify Changes

After completing the source-build setup in [README.md](README.md), run:

```bash
source /opt/ros/lyrical/setup.bash
sudo apt-get update
sudo apt-get install -y ros-dev-tools
rosdep update
scripts/install_hand2_descriptions.sh
rosdep install --from-paths . --ignore-src -r -y
python3 -m pytest -q test
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Reporting Issues

Open an issue with the expected behavior, actual behavior, environment details, and reproduction steps.
