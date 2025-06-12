# Use official ROS 2 Humble base
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install required OS packages
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-opencv \
    libopencv-dev \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-sensor-msgs \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /root

# Clone your repo
RUN git clone https://github.com/MateuszGrudzinski/bve_with_YOLO.git ros2_ws

# Install Python dependencies from the repo
COPY --from=python:3.10-slim /usr/local/bin/pip /usr/local/bin/pip
RUN pip install --upgrade pip && \
    pip install --no-cache-dir -r ros2_ws/requirements.txt

# Source ROS and build the workspace
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Default command
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && bash"]

