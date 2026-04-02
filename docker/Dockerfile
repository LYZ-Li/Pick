FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl vim nano tmux htop tree \
    catkin \
    python3-pip python3-pytest-cov python3-colcon-common-extensions \
    python3-vcstool python3-rosdep python3-dev \
    iputils-ping net-tools iproute2 dnsutils traceroute nmap iperf3 tcpdump \
    usbutils udev \
    libpcl-dev pcl-tools \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-msgs \
    ros-humble-moveit-task-constructor-capabilities \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-task-constructor-msgs \
    ros-humble-moveit-task-constructor-visualization \
    ros-humble-geometric-shapes \
    ros-humble-tf2-eigen \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf-transformations \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-ur \
    ros-humble-ur-robot-driver \
    ros-humble-ur-client-library \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-gripper-controllers \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    numpy scipy open3d transforms3d pyyaml

RUN rosdep init || true && rosdep update --rosdistro=${ROS_DISTRO}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
WORKDIR /root/ros2_ws

COPY dependencies.repos /tmp/dependencies.repos

RUN mkdir -p src && \
    vcs import src < /tmp/dependencies.repos
    
RUN git clone https://github.com/atenpas/gpd.git /tmp/gpd && \
    cd /tmp/gpd && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/gpd
RUN ldconfig

RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro=${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
