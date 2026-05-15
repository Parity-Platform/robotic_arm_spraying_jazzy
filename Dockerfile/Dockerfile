FROM eprosima/vulcanexus:jazzy-desktop

ARG USERNAME=vulcanexus_user

# Install basic CLI tools
RUN apt-get update && apt-get install -y \
    nano \
    sudo \
    bash-completion \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 and simulation-related packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-laser-geometry \
    ros-jazzy-sensor-msgs-py \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-ur-description \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-ur-robot-driver \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-joy \
    ros-jazzy-hri-msgs \
    ros-jazzy-rosbag2-cpp \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Build gz_ros2_control from source (apt 1.2.17 has a threading bug in GazeboSimSystem::initSim)
RUN bash -c "\
    source /opt/vulcanexus/jazzy/setup.bash && \
    mkdir -p /opt/gz_ros2_control_ws/src && \
    git clone https://github.com/ros-controls/gz_ros2_control.git \
        --branch jazzy --depth 1 \
        /opt/gz_ros2_control_ws/src/gz_ros2_control && \
    cd /opt/gz_ros2_control_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    chmod -R a+rX /opt/gz_ros2_control_ws/install"

# Python tooling + deps for numpy-stl
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    && pip3 install --no-cache-dir --break-system-packages numpy-stl \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user with passwordless sudo privileges and rename the existing ubuntu user (UID 1000) to match the desired username
RUN usermod -l $USERNAME ubuntu && \
    usermod -d /home/$USERNAME -m $USERNAME && \
    groupmod -n $USERNAME ubuntu && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Auto-source Vulcanexus environment for every new terminal
RUN echo "source /opt/vulcanexus/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/gz_ros2_control_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

# Switch to the non-root user
USER ${USERNAME}
