# Roborock S5 Max ROS 2 package

This package provides the ROS 2 description and physical bring-up files for the Roborock S5 Max.

The first model uses the published `353 x 350 x 96.5 mm` envelope and `3.5 kg` mass. Unmeasured wheel, caster, and LiDAR positions are marked as temporary in `urdf/params.xacro`.

## Build

Build the package in a ROS 2 Jazzy workspace:

```bash
colcon build --packages-select roborock_s5max
source install/setup.bash
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

Monitor the physical robot in RViz without wheel commands:

```bash
make rviz
```

Keep at least 35 cm of clear floor behind the robot.
If the robot is docked, the driver moves it backward before it starts the LiDAR.
Close RViz to stop the LiDAR and restore Valetudo.

Start SangamIO on the robot:

```bash
make sangamio
```

Start SangamIO, RViz, LiDAR, the ROS bridge, and joystick control:

```bash
make teleop
```

Put the robot on a clear, level floor.
If it is docked, keep at least 35 cm of clear floor behind it.
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

Localize on a saved map and start Nav2:

```bash
make navigate NAME=apartment
```

Put the robot at a known map position or start from its mapped dock.
If it is docked, keep at least 35 cm of clear floor behind it.
The command loads the saved SLAM pose graph and starts RViz.
It does not send a movement command during startup.
Make sure that the LiDAR scan aligns with the map before you set a goal.
If the pose is wrong, use `2D Pose Estimate` in RViz.
After localization is correct, use `Nav2 Goal` to select a clear nearby target.
The joystick remains available as a higher-priority manual override when connected.
Close RViz or press `Ctrl-C` to stop Nav2 and restore Valetudo.

Record the dock staging pose for one saved map:

```bash
make dock-calibrate NAME=apartment
```

Start the robot on the dock. The command undocks and opens RViz. Use `2D Pose
Estimate` if the live scan does not align with the saved map. Type `record` in
the terminal after the alignment is correct. The command stores the most
recent RViz estimate in `apartment.dock.json` and returns to stable charging.
The command does not replace an existing staging file.

Return to the dock:

```bash
make back2dock NAME=apartment
```

The command starts localization and shows RViz. Make sure that the live scan
aligns with the map. Use `2D Pose Estimate` if the pose is wrong. Type `dock`
in the terminal after the pose is correct. Nav2 travels to the saved staging
pose. The beacon controller makes the final approach and confirms charging.

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

The bumper adapter publishes the left and right contact states.
If the robot touches an obstacle during forward motion, contact recovery starts.
It stops, moves backward slowly, turns away from the contact, and stops again.
If the bumper remains pressed, contact recovery holds a zero command.

If a cliff signal starts during forward motion, the safety adapter stops and moves backward slowly once.
The safety stop releases when all cliff signals clear.

A stale-odometry event remains latched.
After odometry recovers, release that safety stop from another terminal:

```bash
make safety-reset
```

Restore the stock robot software and Valetudo:

```bash
make valetudo
```

Run the first autonomous coverage slice on an existing map:

```bash
make coverage NAME=apartment
```

RViz starts before autonomous movement. Make sure that the robot pose and LiDAR scan align with the map. Then type `clean` in the terminal.

The OOMWOO reactive executor controls short cleaning rows. Nav2 controls longer travel between rows. The contact reflex overrides both sources during a bumper recovery. It releases control after the reverse and turn sequence.

The cleaning actuator manager starts the fan and both brushes when coverage starts. It stops all cleaning actuators when coverage ends or the session stops. A joystick can override the route when it is connected.

Use `ROBOT_IP`, `ROS_IMAGE`, `BRIDGE_REPO`, `MAP_DIR`, and `DEMO_SECONDS` to change the local environment.
The bounded takeover programs are in `scripts/`.

## Current limits

The wheel, caster, and LiDAR positions are temporary. Calibrate them before navigation tests.

The coverage command can automatically undock. It does not yet call the tested
return-to-dock command when coverage ends.

The contact recovery does not yet record contact in the OOMWOO tactile map. The coverage executor resumes the current route after the bounded recovery.

Gazebo, complete cleaning, mopping, and docking remain later milestones.
