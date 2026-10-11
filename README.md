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
ros2 launch roborock_s5max inspect_model.launch.py
```

Start physical bring-up with:

```bash
ros2 launch roborock_s5max bringup.launch.py robot_ip:=192.168.4.55
```

The physical launch expects SangamIO on TCP port `5555`. It does not perform the bounded robot takeover.

## Local demonstration commands

The repository Makefile combines the robot takeover, ROS build, RViz model, bridge, and joystick tools.
The bridge repository defaults to the sibling directory `../vacuum_ros2_bridge-review`.
Set `BRIDGE_REPO` if the bridge is at a different path.

Inspect the model without a robot:

```bash
make model
```

Monitor the physical robot in RViz without wheel commands:

```bash
make rviz
```

Physically undock the robot before you start this command.
The S5 Max driver does not start the LiDAR while the robot is docked.
Close RViz to stop the LiDAR and restore Valetudo.

Start SangamIO on the robot:

```bash
make sangamio
```

Start SangamIO, RViz, LiDAR, the ROS bridge, and joystick control:

```bash
make demo
```

Physically undock the robot and put it on a clear, level floor.
Hold joystick button 3 to enable movement and actuator changes.
Use the analog sticks to control linear and angular speed.
Use buttons 0, 1, and 2 to toggle the vacuum, main brush, and side brush.
Release button 3 to stop wheel movement.
Close RViz to stop all actuators and restore Valetudo.

Create a map with SLAM Toolbox:

```bash
make map NAME=apartment
```

The command stores the occupancy map and SLAM pose graph under
`$HOME/.local/share/roborock-s5max/maps`.
The map name can contain letters, numbers, periods, underscores, and hyphens.
Existing map files are not overwritten.

Drive slowly and revisit the starting area before you finish the map.
Close RViz or press `Ctrl-C` to save the map and restore Valetudo.

The first ROS command builds `roborock_s5max_ros:latest` from
`oomwoo_one:latest`.
This image adds SLAM Toolbox and Nav2.
Use `ROS_BASE_IMAGE` and `ROS_IMAGE` to select different image names.

Stop wheel movement and all cleaning actuators without a reboot:

```bash
make stop
```

The joystick publishes on `/cmd_vel_manual` while its deadman is active.
Future Nav2 commands use `/cmd_vel_nav`.
`twist_mux` gives manual control priority and publishes the selected command on `/cmd_vel`.

The safety adapter gives bumper, cliff, and stale-odometry stops the highest priority.
Safety events remain latched after the sensor becomes clear.
After you remove the hazard, release the safety stop from another terminal:

```bash
make safety-reset
```

Restore the stock robot software and Valetudo:

```bash
make valetudo
```

Use `ROBOT_IP`, `ROS_IMAGE`, `BRIDGE_REPO`, `MAP_DIR`, and `DEMO_SECONDS` to change the local environment.
The bounded takeover programs are in `scripts/`.

## Current limits

The wheel, caster, and LiDAR positions are temporary. Calibrate them before navigation tests.

Saved-map navigation, Gazebo, cleaning, mopping, and docking remain later milestones.
