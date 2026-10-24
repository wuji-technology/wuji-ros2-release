# wuji_msgs

`wuji_msgs` defines the public ROS2 messages and services used by Wuji Hand 2 and Wuji Glove drivers.

## Messages

| Interface | Purpose |
|-----------|---------|
| `FingertipTactile.msg` | Timestamped fingertip force vectors, total force, temperature, and metadata digest |
| `JointDiagnostics.msg` | One joint's `nid`, URDF name, electrical values, error code, and communication counters |
| `JointDiagnosticsArray.msg` | Timestamped list of joint diagnostic entries |
| `TrackedPose.msg` | Named tracked pose with confidence |
| `TrackedPoseArray.msg` | Timestamped tracked pose entries |
| `FingerJointAngles.msg` | Named finger joint angles with confidence |
| `HandJointAngles.msg` | Timestamped five-finger joint angles |
| `TactileGrid.msg` | Timestamped tactile grid data |
| `TactileZone.msg` | Named tactile zone values |
| `TactileZoneArray.msg` | Timestamped tactile zone entries |

In `FingertipTactile`, `point_forces` and `total_force` use N. `temperature` uses degrees Celsius.

The Wuji Glove driver uses `TrackedPose`, `TrackedPoseArray`, `FingerJointAngles`, `HandJointAngles`, `TactileGrid`, `TactileZone`, and `TactileZoneArray`. `HandJointAngles` preserves each finger's angle count and confidence instead of projecting the data into a lossy standard joint message.

## Services

| Interface | Request |
|-----------|---------|
| `SetEnabled.srv` | `enabled` and a 20-entry joint mask |
| `ClearFaults.srv` | A 20-entry joint mask |
| `SetEffortLimit.srv` | One effort limit for every joint |
| `SetEffortLimits.srv` | 20 effort limits in dense order |
| `SetMitParams.srv` | 20 `kp` values and 20 `kd` values in dense order |

Each response contains `success` and `message`. Fixed 20-entry arrays use the dense order documented in [`wuji_hand2_driver`](../wuji_hand2_driver/README.md#joint-commands). These services don't carry joint names and don't accept partial arrays.

## Inspect Interfaces

After building and sourcing the workspace:

```bash
ros2 interface show wuji_msgs/msg/HandJointAngles
ros2 interface show wuji_msgs/srv/SetEnabled
```

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
