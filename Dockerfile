FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    python3-pip \
    python3-numpy \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-sensor-msgs \
    ros-noetic-std-srvs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY src ./src
COPY scripts ./scripts

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release && \
    chmod +x /catkin_ws/scripts/*.sh /catkin_ws/src/dual_cam_full_calib/scripts/*.py"

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
