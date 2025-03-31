FROM docker.io/osrf/ros:humble-desktop-full
ARG USERNAME=diffdan_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y git

ENV SHELL /bin/bash

# Configure Colcon Mixin
RUN colcon mixin remove default
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Install Gazebo Fortres (IGN)
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros-gz

# Install Gazebo Classic
RUN apt-get install -y ros-$ROS_DISTRO-gazebo-ros-pkgs

# Setup CycloneDDS
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp


# Configure User
USER $USERNAME

# Ensure workspace directories exist before cloning
RUN mkdir -p /home/diffdan_user/ws/src

# Switch back to the workspace root
WORKDIR /home/diffdan_user/ws

# Install dependencies
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

ENV SHELL="/bin/bash"
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --continue-on-error

RUN echo "source /opt/ros/humble/setup.bash" >> /home/diffdan_user/.bashrc
RUN echo "source /home/diffdan_user/ws/install/setup.bash || echo 'Workspace not Ready. Run colcon build at /home/diffdan_user/ws'" >> /home/diffdan_user/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/diffdan_user/.bashrc

CMD ["/bin/bash"]