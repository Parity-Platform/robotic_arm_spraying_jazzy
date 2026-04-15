FROM ubuntu-vulcanexus:jazzy-desktop

# Define arguments for the new user creation
ARG USERNAME=vulcanexus_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Refresh ROS 2 key and repo
RUN rm -f /etc/apt/sources.list.d/ros2*.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

# Install basic CLI tools and dependencies
RUN apt-get update && apt-get install -y \
    nano \
    sudo \
    bash-completion \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 and simulation-related packages
RUN apt-get update && apt install -y \
    ros-jazzy-pcl-conversions ros-jazzy-pcl-ros \
    ros-jazzy-ur-description \
    ros-jazzy-tf-transformations \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface

# Python tooling + deps for numpy-stl
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
 && pip3 install --no-cache-dir numpy-stl \
 && rm -rf /var/lib/apt/lists/*

    
# Create a non-root user with passwordless sudo privileges
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER ${USERNAME}
