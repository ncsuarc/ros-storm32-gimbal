# ROS STorM32 Gimbal Driver

This is a driver to communicate with STorM32 gimbal controllers.

**You MUST configure your gimbal with the Windows app before using this
package.**

*This package has been tested on ROS Humble Hawksbill on Ubuntu 22.04.*

## Setting up

You must clone this repository as `storm32_gimbal` into your workspace:

```bash
git clone https://github.com/ncsuarc/storm32_gimbal
```

## Dependencies

Before proceeding, make sure to install all dependencies by running:

```bash
rosdep update
rosdep install storm32_gimbal
python3 -m pip install transforms3d
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
colcon build --symlink-install
```

from the root of your workspace.

## Running

To run, the workspace environment must be overlaid.

```bash
. install/setup.bash
```

Simply launch the `storm32_node.py` node as such:

```bash
ros2 run storm32_gimbal gimbal
```

You can change the port and TF frame ID by passing them as parameters

```bash
ros2 run storm32_gimbal storm32_node.py port:=<device_path> frame:=<frame_name>
```

A sample launch file is available in the `launch` directory and can be run with:

```bash
ros2 launch storm32_gimbal gimbal_launch.py
```

## Interfacing

### The `gimbal_ref` frame

All orientations used by this package are relative to a global reference frame
called `gimbal_ref`. This reference frame is attached to the gimbal, and has a
z axis always pointing away from the center of the earth (i.e. it does not
pitch or roll, but can yaw).

### Topics

This package publishes to two topics:

- `~camera_orientation`: The IMU1 readings as a `QuaternionStamped` message
  (i.e. the orientation of the gimballed link relative to the global
  `gimbal_ref` frame). Since the link is stabilized, this should always be
  approximately the target orientation of the gimbal.
- `~controller_orientation`: The IMU2 readings as a `QuaternionStamped` message
  (i.e. the orientation of the STorM32 board relative to the global
  `gimbal_ref` frame). This should represent the orientation of the gimbal and
  is not stabilized.

You can also set a new target orientation relative to the `gimbal_ref` frame by
publishing a `GimbalOrientation` message to the `~target_orientation` topic.
The `orientation` field is expected to be relative to the `gimbal_ref` frame,
and the `unlimited` field defines whether the controller should attempt to
limit its rotation to hard set limits on the STorM32 controller.
