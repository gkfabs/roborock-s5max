# Roborock S5 Max ROS 2 package

This package provides the ROS 2 description and physical bring-up files for the Roborock S5 Max.

The first model uses the published `353 x 350 x 96.5 mm` envelope and `3.5 kg` mass. Unmeasured wheel, caster, and LiDAR positions are marked as temporary in `urdf/params.xacro`.

## Build

Build the package in a ROS 2 Jazzy workspace:

```bash
colcon build --packages-select roborock_s5max
source install/setup.bash
```

Inspect the model with:

```bash
ros2 launch roborock_s5max monitor_robot.launch.py physical:=false
```

Start physical bring-up with:

```bash
ros2 launch roborock_s5max bringup.launch.py robot_ip:=192.168.4.55
```

The physical launch expects SangamIO on TCP port `5555`. It does not perform the bounded robot takeover.

## Current limits

The wheel, caster, and LiDAR positions are temporary. Calibrate them before navigation tests.

Navigation, SLAM Toolbox, Gazebo, cleaning, mopping, docking, and the operator Makefile remain later milestones.
