SHELL := /usr/bin/env bash

ROBOT_IP ?= 192.168.4.55
ROBOT_USER ?= root
ROBOT := $(ROBOT_USER)@$(ROBOT_IP)
ROS_BASE_IMAGE ?= oomwoo_one:latest
ROS_IMAGE ?= roborock_s5max_ros:latest
OOMWOO_TOOLS_REV ?= d7f937d1d0fa0c17e9e3035b51446fab7014c7f3
ROS_WORKSPACE ?= /tmp/s5_ros_ws
BRIDGE_REPO ?= $(CURDIR)/../vacuum_ros2_bridge-review
ROBOT_DESCRIPTION_REPO ?= $(CURDIR)
DEMO_SECONDS ?= 1800
MAP_DIR ?= $(HOME)/.local/share/roborock-s5max/maps
NAME ?=
SIM_SCENARIO ?= centered
SIM_LIDAR_PROFILE ?= realistic
SIM_LIDAR_GEOMETRY_OUTAGE_START ?= 0.0
SIM_LIDAR_GEOMETRY_OUTAGE_DURATION ?= 0.0
SIM_LIDAR_GEOMETRY_OUTAGE_STATE ?=
SIM_LIDAR_GEOMETRY_OUTAGE_STATE_DELAY ?= 0.25
SIM_LIDAR_OUTAGE_DURATION ?= 2.0
SIM_LIDAR_OUTAGE_STATES ?= INITIAL_PERCEPTION CONTROLLING WAIT_FOR_CHARGE
SIM_SCENARIOS ?= centered lateral-offset heading-error beacon-loss missed-contact delayed-charging dock-undock-cycle

.PHONY: help valetudo sangamio teleop map navigate coverage rviz dock-signals dock-signals-undocked redock-test dock-calibrate back2dock dock-sim dock-sim-headless dock-sim-matrix dock-sim-lidar-outage-matrix stop safety-reset ros-image ros-build

help:
	@echo "make valetudo  - stop the ROS session and restore the stock robot software"
	@echo "make sangamio  - take over the robot and start SangamIO"
	@echo "make teleop    - start RViz and control the physical robot with a joystick"
	@echo "make map NAME=<name> - create and save a map with SLAM Toolbox"
	@echo "make navigate NAME=<name> - localize on a saved map and start Nav2"
	@echo "make coverage NAME=<name> - run mapped-area coverage with contact recovery"
	@echo "make rviz      - monitor the physical robot in RViz without movement"
	@echo "make dock-signals - show the passive dock infrared signals"
	@echo "make dock-signals-undocked - undock and show the dock infrared signals"
	@echo "make redock-test - undock, make a bounded final dock approach, and confirm charging"
	@echo "make dock-calibrate NAME=<name> - save the map staging pose and redock"
	@echo "make back2dock NAME=<name> - navigate to the saved staging pose and redock"
	@echo "make dock-sim - run OpenNav Docking with the Gazebo window"
	@echo "make dock-sim-headless - run OpenNav Docking without the Gazebo window"
	@echo "make dock-sim-matrix - run every OpenNav Docking scenario"
	@echo "make dock-sim-lidar-outage-matrix - test LiDAR loss during OpenNav states"
	@echo "make stop      - stop wheels and cleaning actuators without a reboot"
	@echo "make safety-reset - release a cleared physical safety stop"

valetudo:
	@docker stop --timeout 30 s5max-navigation s5max-mapping s5max-joystick-demo \
		s5max-rviz-monitor s5max-coverage >/dev/null 2>&1 || true
	@ROBOT="$(ROBOT)" ./scripts/robot-mode.sh valetudo

sangamio:
	@ROBOT="$(ROBOT)" DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/robot-mode.sh sangamio

ros-image:
	@docker build --build-arg BASE_IMAGE="$(ROS_BASE_IMAGE)" \
		--build-arg OOMWOO_TOOLS_REV="$(OOMWOO_TOOLS_REV)" \
		-t "$(ROS_IMAGE)" -f docker/Dockerfile .

ros-build: ros-image
	@mkdir -p "$(ROS_WORKSPACE)/build" "$(ROS_WORKSPACE)/install" "$(ROS_WORKSPACE)/log"
	@docker run --rm --network host \
		-v "$(BRIDGE_REPO):/ws/src/vacuum_ros2_bridge:ro" \
		-v "$(ROBOT_DESCRIPTION_REPO):/ws/src/roborock_s5max:ro" \
		-v "$(ROS_WORKSPACE)/build:/ws/build" \
		-v "$(ROS_WORKSPACE)/install:/ws/install" \
		-v "$(ROS_WORKSPACE)/log:/ws/log" \
		-w /ws "$(ROS_IMAGE)" bash -lc \
		'source /opt/ros/jazzy/setup.bash && colcon build \
		--base-paths /ws/src /opt/oomwoo-ros2-tools/src \
		--packages-select vacuum_ros2_bridge roborock_s5max \
		oomwoo_clean oomwoo_clean_ui oomwoo_coverage \
		oomwoo_localization oomwoo_localization_msgs oomwoo_msgs \
		--symlink-install'

dock-sim: ros-build
	@ROS_IMAGE="$(ROS_IMAGE)" ROS_WORKSPACE="$(ROS_WORKSPACE)" \
		BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		./scripts/run-opennav-dock-sim.sh

dock-sim-headless: ros-build
	@ROS_IMAGE="$(ROS_IMAGE)" ROS_WORKSPACE="$(ROS_WORKSPACE)" \
		BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		SIM_GUI=0 ./scripts/run-opennav-dock-sim.sh

dock-sim-matrix: ros-build
	@ROS_IMAGE="$(ROS_IMAGE)" ROS_WORKSPACE="$(ROS_WORKSPACE)" \
		BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		SIM_SCENARIOS="$(SIM_SCENARIOS)" \
		./scripts/run-dock-sim-matrix.sh

dock-sim-lidar-outage-matrix: ros-build
	@ROS_IMAGE="$(ROS_IMAGE)" ROS_WORKSPACE="$(ROS_WORKSPACE)" \
		BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		SIM_LIDAR_OUTAGE_STATES="$(SIM_LIDAR_OUTAGE_STATES)" \
		SIM_LIDAR_GEOMETRY_OUTAGE_DURATION="$(SIM_LIDAR_OUTAGE_DURATION)" \
		./scripts/run-dock-lidar-outage-matrix.sh

rviz: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/run-rviz-monitor.sh

dock-signals: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/run-dock-signals.sh

dock-signals-undocked: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" UNDOCK=1 \
		./scripts/run-dock-signals.sh

redock-test: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" UNDOCK=1 REDOCK=1 \
		./scripts/run-dock-signals.sh

dock-calibrate: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" MAP_DIR="$(MAP_DIR)" NAME="$(NAME)" \
		DOCK_MODE=calibrate ./scripts/run-docking.sh

back2dock: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" MAP_DIR="$(MAP_DIR)" NAME="$(NAME)" \
		DOCK_MODE=return ./scripts/run-docking.sh

teleop: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/run-joystick-demo.sh

map: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" MAP_DIR="$(MAP_DIR)" NAME="$(NAME)" \
		./scripts/run-mapping.sh

navigate: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" MAP_DIR="$(MAP_DIR)" NAME="$(NAME)" \
		./scripts/run-navigation.sh

coverage: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" MAP_DIR="$(MAP_DIR)" NAME="$(NAME)" \
		./scripts/run-coverage.sh

stop:
	@ROS_IMAGE="$(ROS_IMAGE)" ROS_WORKSPACE="$(ROS_WORKSPACE)" \
		BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		./scripts/stop-active-session.sh

safety-reset:
	@ROS_IMAGE="$(ROS_IMAGE)" ROS_WORKSPACE="$(ROS_WORKSPACE)" \
		BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		./scripts/reset-safety.sh
