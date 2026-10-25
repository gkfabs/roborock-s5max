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
pose. OpenNav Docking makes the final approach and confirms charging.

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

The OOMWOO reactive executor controls short cleaning rows. Nav2 controls longer travel between rows. The contact reflex overrides both sources during a bumper recovery. It releases control after the reverse and turn sequence. OOMWOO records the contact point and continues past the blocked segment without sending another reverse command.

The cleaning actuator manager starts the fan and both brushes when coverage starts. It stops all cleaning actuators when coverage ends or the session stops.

After successful coverage, the robot sends one Nav2 `DockRobot` action. OpenNav Docking controls staging, final alignment, contact, retries, and charge confirmation.

The S5 Max dock-pose estimator combines the LiDAR dock face with the infrared beacon. It publishes the final robot pose in the `odom` frame.

A joystick can override the route when it is connected.

Use `ROBOT_IP`, `ROS_IMAGE`, `BRIDGE_REPO`, `MAP_DIR`, and `DEMO_SECONDS` to change the local environment.
The bounded takeover programs are in `scripts/`.

Run the OpenNav Docking test with the Gazebo window:

```bash
make dock-sim
```

Run the same test without the Gazebo window:

```bash
make dock-sim-headless
```

Run all docking scenarios:

```bash
make dock-sim-matrix
```

Run temporary LiDAR loss during each OpenNav Docking state:

```bash
make dock-sim-lidar-outage-matrix
```

Gazebo supplies LiDAR, odometry, dock contact, and charging data. All tests use one launch file and one runner. They use the same estimator, dock plugin, and Docking Server configuration as the robot.

## Current limits

The wheel, caster, and LiDAR positions are temporary. Calibrate them before navigation tests.

The coverage command requires a saved dock staging pose. It automatically
returns only after the coverage planner reports successful completion.

Contact recovery records a temporary no-go pocket in the OOMWOO coverage route. A run stops after 25 contact recoveries instead of continuing without a bound.

The docking matrix covers centered, lateral-offset, heading-error, beacon-loss, missed-contact, delayed-charging, and dock-undock-cycle scenarios.

The dock-undock-cycle scenario docks the robot, disables charging, and returns the robot to the staging pose.

Complete mopping and unattended recovery from a failed cleaning run remain later milestones.
