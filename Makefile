SHELL := /usr/bin/env bash

ROBOT_IP ?= 192.168.4.55
ROBOT_USER ?= root
ROBOT := $(ROBOT_USER)@$(ROBOT_IP)
ROS_IMAGE ?= oomwoo_one:latest
ROS_WORKSPACE ?= /tmp/s5_ros_ws
BRIDGE_REPO ?= $(CURDIR)/../vacuum_ros2_bridge-review
ROBOT_DESCRIPTION_REPO ?= $(CURDIR)
DEMO_SECONDS ?= 1800

.PHONY: help valetudo sangamio demo model ros-build

help:
	@echo "make valetudo  - stop the demo and restore the stock robot software"
	@echo "make sangamio  - take over the robot and start SangamIO"
	@echo "make demo      - start SangamIO, the ROS bridge, and joystick control"
	@echo "make model     - inspect the S5 Max model in RViz without the robot"

valetudo:
	@docker stop --timeout 3 s5max-joystick-demo >/dev/null 2>&1 || true
	@ROBOT="$(ROBOT)" ./scripts/robot-mode.sh valetudo

sangamio:
	@ROBOT="$(ROBOT)" DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/robot-mode.sh sangamio

ros-build:
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

demo: ros-build
	@ROBOT="$(ROBOT)" ROBOT_IP="$(ROBOT_IP)" ROS_IMAGE="$(ROS_IMAGE)" \
		ROS_WORKSPACE="$(ROS_WORKSPACE)" BRIDGE_REPO="$(BRIDGE_REPO)" \
		DEMO_SECONDS="$(DEMO_SECONDS)" \
		./scripts/run-joystick-demo.sh
