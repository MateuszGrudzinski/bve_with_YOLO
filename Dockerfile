# Stage 0: build environment
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# 1) Install OS dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-opencv \
    libopencv-dev \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    python3-pyqt5 \
    python3-opengl \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages that only pip can provide
RUN pip3 install --no-cache-dir \
    open3d \
    PyOpenGL \
    PyOpenGL_accelerate

# 2) Create ROS2 workspace
WORKDIR /root/ros2_ws

# 3) Clone your repo into src/
RUN mkdir -p src && \
    git clone https://github.com/MateuszGrudzinski/bve_with_YOLO.git src/bve_with_YOLO

# 4) Install Python requirements from your package
#    Assuming your requirements.txt lives at src/bve_with_YOLO/requirements.txt
RUN pip3 install --no-cache-dir -r src/bve_with_YOLO/requirements.txt

# 5) Source ROS2 and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 6) Runtime
#    You can override CMD with `docker run ... bash` to get a shell
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch bird_view bve_autopilot_launch.py"]

