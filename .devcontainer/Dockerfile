FROM ros:humble

# Arguments for user creation
ARG USERNAME=david
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# System upgrades and core dependencies
RUN apt-get update && \
    apt dist-upgrade -y && \
    apt-get install -y --no-install-recommends \
        python-is-python3 \
        libopencv-dev \
        libboost-all-dev \
        openssl \
        git \
        gdb \
        libi2c-dev \
        i2c-tools \
        libcurl4-openssl-dev \
        libssl-dev \
        curl \
        ros-humble-imu-tools \
        ros-humble-teleop-twist-joy \
        ros-humble-image-transport \
        ros-humble-desktop \
        ros-humble-turtlesim \
        nano \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS for all users and the specific user
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Install Node.js as the david user
USER $USERNAME
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash

ENV NVM_DIR=/home/$USERNAME/.nvm \
    NODE_VERSION=20.10.0

# Install node and npm
RUN . $NVM_DIR/nvm.sh \
    && nvm install $NODE_VERSION \
    && nvm alias default $NODE_VERSION \
    && nvm use default

# Switch back to root for the entrypoint
USER root

ENTRYPOINT ["./ros_entrypoint.sh"]
CMD [ "bash" ]