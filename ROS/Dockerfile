# https://hub.docker.com/layers/osrf/ros/humble-desktop-full/images/sha256-7fb1df4b06d8eb76a5918b80a9c40ca3eff9b887abebbe0a29e3c862967fdaa0?context=explore
FROM osrf/ros:humble-desktop-full

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    ros-humble-topic-tools \
    python3-colcon-common-extension \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
ARG USER_NAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Clone the 2024-2025 repository
ARG REPO_URL=https://github.com/UA-NASA-Robotics/2024-2025-Robot-Code.git
RUN git clone ${REPO_URL} repo

# Create a symlink from repo's ROS folder to ~/ros2_ws
RUN ln -s ${HOME}/repo/ROS ${HOME}/ros2_ws

# Install ROS dependencies using rosdep
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths ros2_ws --ignore-src -r -y"

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --workspace ros2_ws"

# Source the ROS 2 environment and the workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the default command to bash
CMD ["/bin/bash"]