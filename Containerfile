ARG ROS_DISTRO=jazzy
FROM ubuntu:24.04

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release locales software-properties-common sudo \
    git build-essential cmake pkg-config python3-pip shellcheck \
    colmap python3-numpy python3-scipy python3-yaml python3-opencv python3-matplotlib python3-pytest \
    usbutils udev v4l-utils less nano vim tmux htop tree \
    mesa-utils libgl1-mesa-dri libglx-mesa0 libegl1 libxkbcommon-x11-0 \
    libxcb-xinerama0 libxcb-cursor0 libxcb-keysyms1 libxcb-image0 libxcb-render-util0 \
    libxcb-icccm4 libxcb-shape0 libxcb-randr0 libxcb-xfixes0 \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy apt repository
RUN install -m 0755 -d /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-librealsense2 \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Convenience shell setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" >> /etc/bash.bashrc \
    && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42}" >> /etc/bash.bashrc

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /work
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
