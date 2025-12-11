# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

# Set the maintainer information for this Dockerfile
LABEL maintainer="Sahruday Patti <sahrudaypatti@gmail.com>"
LABEL version="1.0"
LABEL description="ROS 2 development environment"


# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive

# Set the default shell to bash for RUN commands
# This ensures all RUN commands use bash inastead of sh
SHELL ["/bin/bash", "-c"]

# Update the system and install essential tools
# This step upgrades all packages and installs utilities needed for development
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends apt-utils wget curl git build-essential \
    vim sudo lsb-release locales bash-completion tzdata gosu gedit htop nano libserial-dev

# Install additional tools required for ROS 2 development
# These packages help with building and managing ROS 2 workspaces
RUN apt-get update -q && \
    apt-get install -y gnupg2 iputils-ping usbutils \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-pip python3-rosdep python3-vcstool

# Set up the ROS 2 environment
# This ensures that ROS 2 commands are available in the shell
# rosdep is a tool for installing system dependencies for ROS packages
RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Install additional ROS 2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-reconfigure

# Install PUMA 560 dependencies: ROS2 Controllers, MoveIt2, and Gazebo integration
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-py \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-planners-ompl \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control

# Install Mesa graphics drivers
# Mesa is an open-source implementation of OpenGL and other graphics APIs
# It's crucial for 3D rendering in many applications, including RViz in ROS
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:kisak/kisak-mesa

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        mesa-utils \
        libgl1-mesa-dri \
        libegl-mesa0 \
        libgbm1 \
        libegl-dev \
        libgles-dev

# Install noVNC packages for macOS GUI support (also available on Linux if needed)
# These packages enable browser-based GUI access via VNC
RUN apt-get update && \
    apt-get install -y \
        xvfb \
        x11vnc \
        novnc \
        python3-websockify \
        openbox \
        xterm \
        fonts-dejavu \
        dbus-x11

# Create necessary directories
RUN mkdir -p /etc/udev/rules.d && \
    mkdir -p /root/ros2_ws && \
    mkdir -p /root/ros2_ws/src && \
    mkdir -p /root/scripts && \
    mkdir -p /root/container-setup-scripts

# Copy the entire project into the container
# Since we're using the parent directory as context, we can copy everything
# COPY workspace/ /root/ros2_ws/src/
COPY .vscode /root/ros2_ws/.vscode

# Copy configuration files from devcontainer directories
COPY .devcontainer/docker/workspace.sh /root/
COPY .devcontainer/docker/entrypoint.sh /root/
COPY .devcontainer/scripts/start-vnc.sh /root/
COPY scripts/ /root/scripts/
COPY container-setup-scripts/ /root/container-setup-scripts/

# Make scripts executable
RUN chmod +x /root/workspace.sh /root/entrypoint.sh /root/start-vnc.sh
RUN chmod +x /root/container-setup-scripts/*.sh

# Run the workspace setup script
# This typically installs workspace dependencies and builds the ROS 2 packages
# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    software-properties-common

# Ensure the install_gazebo.sh script is executable
RUN chmod +x /root/container-setup-scripts/install_gazebo.sh

# Run the install_gazebo.sh script
RUN /root/container-setup-scripts/install_gazebo.sh

WORKDIR /root
RUN ./workspace.sh

# Ensure the ROS 2 workspace is sourced in every new shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint for the container
# This script will be run every time the container starts
ENTRYPOINT ["/root/entrypoint.sh"]

# Set the default command
# This keeps the container running indefinitely, allowing you to exec into it
CMD ["/bin/bash", "-c", "tail -f /dev/null"]