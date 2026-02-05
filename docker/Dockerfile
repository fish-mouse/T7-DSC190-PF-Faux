FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 1. INSTALL SYSTEM TOOLS
# Added 'asio-cmake-module' and 'io-context' to fix the dependency chain error
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-depthai-ros \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-ackermann-msgs \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-io-context \
    libusb-1.0-0-dev \
    git \
    build-essential \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-msgs \
    ros-humble-tf-transformations \
    python3-transforms3d \
    python3-vcstool \
    ros-humble-tf2-ros \
    cmake \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. INSTALL PYTHON AI LIBRARIES
RUN pip3 install \
    numpy \
    opencv-python \
    scipy \
    ultralytics \
    depthai \
    pyyaml \
    blobconverter \
    psutil

# 3. SETUP WORKSPACE
WORKDIR /pf_sim
COPY docker/racer.repos .

# 4. DOWNLOAD DRIVERS
RUN mkdir -p src
RUN vcs import src < racer.repos

# 5. BUILD THE SYSTEM
RUN . /opt/ros/humble/setup.sh && \
	colcon build --symlink-install --parallel-workers 1 --cmake-args -DWITH_CUDA=OFF

# 6. SETUP ENTRYPOINT
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
