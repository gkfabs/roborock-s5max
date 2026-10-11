ARG BASE_IMAGE=oomwoo_one:latest
FROM ${BASE_IMAGE}

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox && \
    rm -rf /var/lib/apt/lists/*
