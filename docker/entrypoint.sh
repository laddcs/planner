#!/bin/bash
# Basic entrypoint for ROS / Catkin Docker containers

# Source ROS Noetic

source /opt/ros/noetic/setup.bash

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  echo "source /overlay_ws/devel/setup.bash" >> ~/.bashrc
  source /overlay_ws/devel/setup.bash
  echo "Sourced autonomy overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"