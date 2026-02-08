FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 1. INSTALL SYSTEM TOOLS
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
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

# 2. INSTALL PYTHON LIBRARIES (including Cython for range_libc)
# Pin numpy<1.24 to stay compatible with system transforms3d (uses np.float)
RUN pip3 install \
    "numpy<1.24" \
    opencv-python \
    scipy \
    pyyaml \
    psutil \
    cython

# 3. SETUP WORKSPACE
WORKDIR /pf_sim
COPY docker/racer.repos .

# 4. DOWNLOAD EXTERNAL DEPENDENCIES
RUN mkdir -p src
RUN vcs import src < racer.repos

# Debug: Show the directory structure
RUN echo "=== src/ contents ===" && ls -la src/ && \
    echo "=== src/external/ contents ===" && ls -la src/external/ && \
    echo "=== range_libc contents ===" && ls -la src/external/range_libc/

# 5. BUILD RANGE_LIBC PYTHON BINDINGS (CPU-only, no CUDA)
WORKDIR /pf_sim/src/external/range_libc/pywrapper
RUN WITH_CUDA=OFF python3 setup.py install
WORKDIR /pf_sim

# 6. COPY THE PARTICLE FILTER SOURCE CODE
COPY particle_filter/ /pf_sim/src/particle_filter/particle_filter/
COPY package.xml /pf_sim/src/particle_filter/
COPY setup.py /pf_sim/src/particle_filter/
COPY setup.cfg /pf_sim/src/particle_filter/
COPY resource/ /pf_sim/src/particle_filter/resource/
COPY config/ /pf_sim/src/particle_filter/config/
COPY launch/ /pf_sim/src/particle_filter/launch/
COPY maps/ /pf_sim/src/particle_filter/maps/

# 7. BUILD ROS2 PACKAGES (skip range_lib CMake package - we already built Python bindings)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers 1 --packages-skip range_lib

# 8. SETUP ENTRYPOINT
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
