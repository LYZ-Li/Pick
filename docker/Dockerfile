FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV COLORTERM=truecolor
ENV CLICOLOR=1
ENV CLICOLOR_FORCE=1
ENV FORCE_COLOR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl vim nano tmux htop tree \
    python3-pip python3-pytest-cov python3-colcon-common-extensions \
    python3-vcstool python3-rosdep python3-dev python3-opencv python3-reportlab \
    iputils-ping net-tools iproute2 dnsutils traceroute nmap iperf3 tcpdump \
    usbutils udev \
    libpcl-dev pcl-tools \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-msgs \
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
    ros-humble-aruco-ros \
    ros-humble-foxglove-bridge \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    numpy scipy open3d transforms3d pyyaml

RUN rosdep init || true && rosdep update --rosdistro=${ROS_DISTRO}

RUN printf '%s\n' \
    'source /opt/ros/${ROS_DISTRO}/setup.bash' \
    'if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi' \
    'export TERM=${TERM:-xterm-256color}' \
    'export COLORTERM=${COLORTERM:-truecolor}' \
    'export CLICOLOR=1' \
    'export CLICOLOR_FORCE=1' \
    'export FORCE_COLOR=1' \
    'alias ls="ls --color=auto"' \
    'alias grep="grep --color=auto"' \
    'alias diff="diff --color=auto"' \
    'export PS1="\[\e[1;33m\][docker:\h]\[\e[0m\] \[\e[1;32m\]\u\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\\$ "' \
    >> /root/.bashrc
WORKDIR /root/ros2_ws

COPY dependencies.repos /tmp/dependencies.repos

RUN mkdir -p src && \
    vcs import src < /tmp/dependencies.repos && \
    if [ -f "src/ros2_robotiq_gripper/ros2_robotiq_gripper-not-released.${ROS_DISTRO}.repos" ]; then \
        vcs import src --skip-existing --input "src/ros2_robotiq_gripper/ros2_robotiq_gripper-not-released.${ROS_DISTRO}.repos"; \
    fi

COPY ros2_ws/src /root/ros2_ws/src

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

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'cd /root/ros2_ws' \
    'source /opt/ros/${ROS_DISTRO}/setup.bash' \
    'colcon build --symlink-install "$@"' \
    > /usr/local/bin/build_workspace && \
    chmod +x /usr/local/bin/build_workspace

CMD ["/bin/bash"]
