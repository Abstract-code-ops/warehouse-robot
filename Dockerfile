# ============================================================
#  Base: Ubuntu 22.04 (Jammy)
# ============================================================
FROM osrf/ros:humble-desktop-full

LABEL maintainer="aa2933395@gmail.com"
LABEL description="Cognitive AMR — ROS2 Humble + Gazebo + Nav2 + AI"

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ── System dependencies ──────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-foxglove-bridge \
    ros-humble-laser-filters \
    ros-humble-ros-gz \
    git curl wget htop tmux nano \
    && rm -rf /var/lib/apt/lists/*

# ── Python AI stack ──────────────────────────────────────────
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ── ROS workspace setup ──────────────────────────────────────
RUN mkdir -p /ros2_ws/src

# ── Manual Package Downloads ─────────────────────────────────
WORKDIR /ros2_ws/src

# Clone the ira_laser_tools repository
# downloading from sudo apt does not work for some reason
# Using the -b flag ensures you get the correct ROS 2 branch
# Use the full URL and tell git not to prompt for credentials
RUN git clone --depth 1 -b ros2 https://github.com/iralabdisco/ira_laser_tools.git
# just incase there are hidden system libraries to install we do this
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

WORKDIR /ros2_ws

# ── Environment sourcing ─────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# ── Entrypoint ───────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
