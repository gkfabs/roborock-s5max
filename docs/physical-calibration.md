# Roborock S5 Max Physical Calibration

This file records physical tests for the ROS 2 model and navigation values.

## Wheel odometry

The straight test commanded `0.20 m`. The measured travel was `0.1911 m`.

The initial difference was `4.45%`. This result is inside the initial five-percent tolerance.

The full-turn test measured `351.6°` from wheel odometry. The IMU integration measured `358.4°`.

SangamIO publishes a wheel scale of `0.000798 m/tick`. It publishes an effective wheel track of `0.229 m`.

## LiDAR alignment

The flat-wall test confirmed the forward scan direction. The URDF uses zero LiDAR yaw because SangamIO aligns scan angles.

The RViz test placed the LiDAR optical center at `x = 0.060 m` and `y = 0.000 m`.

The scan remained usable during manual movement. Record a ROS bag before a final translation calibration.

## IMU

The bridge uses `0.001 rad/s` per gyroscope value. SangamIO reports acceleration in `m/s²`.

A stationary capture measured a gravity magnitude near `10.015 m/s²`.

## Navigation limits

The first navigation tests use `0.15 m/s` forward speed and `0.05 m/s` reverse speed.

The maximum angular speed is `0.6 rad/s`. The navigation radius is `0.185 m`, including a small body margin.

Keep cleaning actuators off during navigation calibration.
