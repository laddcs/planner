FROM osrf/ros:noetic-desktop-full AS base
SHELL ["/bin/bash", "-c"]

LABEL maintainer Bryce Ford "ford.1009@osu.edu"

# To avoid tzdata asking for geographic location...
ENV DEBIAN_frontend noninteractive

# Set the working directory to /root
ENV DIRPATH /root
WORKDIR $DIRPATH

# Install dependancies
RUN apt-get update && apt-get install --no-install-recommends \ 
    git \
    python3-catkin-tools \
    ros-noetic-mavros -y \
    ros-noetic-mavros-extras -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    libgstreamer-plugins-base1.0-dev
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

####################################################
# Overlay Development Image for Planner Simulation #
####################################################
FROM base AS overlay

# Create an overlay catkin workspace
RUN mkdir -p /overlay_ws/src/planner
WORKDIR /overlay_ws
RUN catkin init && catkin config --merge-devel
WORKDIR /overlay_ws/src
RUN wstool init
WORKDIR /overlay_ws
COPY ./commander ./src/planner/commander
COPY ./planner ./src/planner/planner
COPY ./planner_msgs ./src/planner/planner_msgs
COPY ./dependencies.rosinstall ./src/planner/

# Pull in dependencies via wstool and build the workspace
RUN source /opt/ros/noetic/setup.bash
RUN wstool merge -t src src/planner/dependencies.rosinstall && \
    wstool update -t src && \
    rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]