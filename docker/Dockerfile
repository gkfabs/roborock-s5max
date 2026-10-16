ARG BASE_IMAGE=oomwoo_one:latest
FROM ${BASE_IMAGE}

ARG OOMWOO_TOOLS_REV=d7f937d1d0fa0c17e9e3035b51446fab7014c7f3

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        python3-scipy \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-ros-gz-interfaces \
        ros-jazzy-slam-toolbox && \
    rm -rf /var/lib/apt/lists/* && \
    git clone --filter=blob:none --no-checkout \
        https://github.com/makerspet/oomwoo-ros2-tools.git \
        /opt/oomwoo-ros2-tools && \
    git -C /opt/oomwoo-ros2-tools checkout "$OOMWOO_TOOLS_REV" && \
    rm -rf /opt/oomwoo-ros2-tools/.git
