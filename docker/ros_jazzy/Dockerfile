# ROS 2 Jazzy side of the TurtleBot3 Lime x Isaac Sim 6.2 tutorials.
#
# Isaac Sim runs natively on the host (it ships its own ROS 2 Jazzy libraries).
# Everything "ROS" - RViz, Nav2, MoveIt 2, ros2_control, YOLO - runs in this container
# and talks to Isaac Sim over DDS on the host network.
#
# Build:  docker build -t lime_ros_jazzy:latest docker/ros_jazzy
# Run:    ./docker/run_ros.sh
#
# The apt list below is also the recipe for a native install on Ubuntu 24.04:
#   sudo apt install ros-jazzy-desktop <the packages below>
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Optional: use a faster / closer Ubuntu mirror (replaces archive.ubuntu.com AND
# security.ubuntu.com), e.g.
#   docker build --build-arg UBUNTU_MIRROR=http://ftp.udx.icscoe.jp/Linux/ubuntu/ ...
ARG UBUNTU_MIRROR=""
RUN if [ -n "$UBUNTU_MIRROR" ]; then \
      sed -i -E "s#http://(archive|security)\.ubuntu\.com/ubuntu/?#${UBUNTU_MIRROR}#g" \
        /etc/apt/sources.list.d/ubuntu.sources && \
      grep -h "^URIs" /etc/apt/sources.list.d/ubuntu.sources; \
    fi

RUN apt-get update && apt-get install -y --no-install-recommends \
      # Navigation
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-pointcloud-to-laserscan \
      # Manipulation
      ros-jazzy-moveit \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-gripper-controllers \
      ros-jazzy-joint-state-topic-hardware-interface \
      # Robot description
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-realsense2-description \
      # Vision
      ros-jazzy-vision-msgs \
      ros-jazzy-vision-msgs-rviz-plugins \
      ros-jazzy-cv-bridge \
      ros-jazzy-rqt-image-view \
      # Tools
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-topic-tools \
      ros-jazzy-rmw-fastrtps-cpp \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      git \
      # Screen capture of RViz for the tutorial media
      ffmpeg \
      imagemagick \
    && rm -rf /var/lib/apt/lists/*

# YOLO (Ultralytics, AGPL-3.0) with CPU PyTorch - plenty fast for a 640x480 camera.
# numpy<2 keeps the apt-built cv_bridge working.
RUN pip3 install --no-cache-dir --break-system-packages \
      --index-url https://download.pytorch.org/whl/cpu torch torchvision \
 && pip3 install --no-cache-dir --break-system-packages \
      "numpy<2" "ultralytics>=8.3" \
 && pip3 uninstall -y --break-system-packages opencv-python || true \
 && pip3 install --no-cache-dir --break-system-packages "opencv-python-headless<4.12" "numpy<2"

# Behavior trees in Python (tutorials 15 and 16). Nav2's own behavior trees (BehaviorTree.CPP) come with
# ros-jazzy-navigation2 above. (A separate layer, so adding it did not rebuild the layers above.)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-py-trees \
      ros-jazzy-py-trees-ros \
    && rm -rf /var/lib/apt/lists/*

# DDS profile shared with the host (UDP only: shared memory does not cross the container boundary
# reliably). The repository is mounted at /lime by docker/run_ros.sh, so both sides read the same file.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/lime/config/fastdds.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/lime/config/fastdds.xml

# Workspace mounted from the repo at runtime: /lime (repo root), /lime/ros2_ws
WORKDIR /lime/ros2_ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
