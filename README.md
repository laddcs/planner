# planner

## Overview
The repository contains packages for planning and tracking trajectories in a wildland burn environment. The following packages are included in the repo:
- commander: Handles system internal state, Pixhawk mode management, recieving guidance goals from QGroundControl, and tracking trajectories.
- planner: Handles planning requests, constructs trajecotry from planning output.

### Installing planner

Create a catkin workspace:

This folder will probably be already created since the previous process would have created it. If it is not present, do:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --merge-devel
cd ~/catkin_ws/src
wstool init
```

#### Clone this repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/laddcs/planner.git
```
