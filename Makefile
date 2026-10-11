SHELL := /usr/bin/env bash

ROBOT_IP ?= 192.168.4.55
ROBOT_USER ?= root
ROBOT := $(ROBOT_USER)@$(ROBOT_IP)
ROS_BASE_IMAGE ?= oomwoo_one:latest
ROS_IMAGE ?= roborock_s5max_ros:latest
ROS_WORKSPACE ?= /tmp/s5_ros_ws
BRIDGE_REPO ?= $(CURDIR)/../vacuum_ros2_bridge-review
ROBOT_DESCRIPTION_REPO ?= $(CURDIR)
DEMO_SECONDS ?= 1800
MAP_DIR ?= $(HOME)/.local/share/roborock-s5max/maps
NAME ?=

.PHONY: help valetudo sangamio demo map navigate model rviz stop safety-reset ros-image ros-build

help:
	@echo "make valetudo  - stop the demo and restore the stock robot software"
	@echo "make sangamio  - take over the robot and start SangamIO"
	@echo "make demo      - start RViz and control the physical robot with a joystick"
	@echo "make map NAME=<name> - create and save a map with SLAM Toolbox"
	@echo "make navigate NAME=<name> - localize on a saved map and start Nav2"
	@echo "make model     - inspect the S5 Max model in RViz without the robot"
	@echo "make rviz      - monitor the physical robot in RViz without movement"
	@echo "make stop      - stop wheels and cleaning actuators without a reboot"
	@echo "make safety-reset - release a cleared physical safety stop"

valetudo:
	@docker stop --timeout 30 s5max-navigation s5max-mapping s5max-joystick-demo \
		s5max-rviz-monitor >/dev/null 2>&1 || true
	@ROBOT="$(ROBOT)" ./scripts/robot-mode.sh valetudo

sangamio:
	@ROBOT="$(ROBOT)" DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/robot-mode.sh sangamio

ros-image:
	@docker build --build-arg BASE_IMAGE="$(ROS_BASE_IMAGE)" \
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
		'source /opt/ros/jazzy/setup.bash && colcon build --packages-select vacuum_ros2_bridge roborock_s5max --symlink-install'

model: ros-build
	@docker run --rm --name s5max-rviz-model --network host \
		--user "$(shell id -u):$(shell id -g)" \
		-e DISPLAY="$(DISPLAY)" \
		-e QT_X11_NO_MITSHM=1 \
		-e LIBGL_ALWAYS_SOFTWARE=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v "$(BRIDGE_REPO):/ws/src/vacuum_ros2_bridge:ro" \
		-v "$(ROBOT_DESCRIPTION_REPO):/ws/src/roborock_s5max:ro" \
		-v "$(ROS_WORKSPACE)/build:/ws/build" \
		-v "$(ROS_WORKSPACE)/install:/ws/install:ro" \
		-v "$(ROS_WORKSPACE)/log:/ws/log" \
		-w /ws "$(ROS_IMAGE)" bash -lc \
		'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch roborock_s5max inspect_model.launch.py'

rviz: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		ROBOT_DESCRIPTION_REPO="$(ROBOT_DESCRIPTION_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/run-rviz-monitor.sh

demo: ros-build
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
