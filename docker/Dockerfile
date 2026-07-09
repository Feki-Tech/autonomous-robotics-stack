# Autonomous Robotics Stack — container images.
#
#   dev  interactive development shell: toolchain + ROS 2 + Gazebo, no source
#        baked in; mount the repository at /ws (see docker-compose.yml).
#   sil  self-contained software-in-the-loop image: builds the workspace and
#        launches the full simulation pipeline by default.
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO} AS base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-${ROS_DISTRO}-ros-gz \
        cmake \
        ninja-build \
        g++ \
    && rm -rf /var/lib/apt/lists/*
COPY docker/entrypoint.sh /usr/local/bin/ars-entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ars-entrypoint.sh"]
CMD ["bash"]

FROM base AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
        clang-format \
        clang-tidy \
        gdb \
        git \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws

FROM base AS sil
WORKDIR /ws
COPY src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-select ars_ros2 ars_sim \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log
CMD ["ros2", "launch", "ars_sim", "sil.launch.py"]
