# Gazebo & ROS Simulator

![|Solid](https://www.streetdrone.com/wp-content/uploads/2020/05/ros-02.png)

_________________
## Short Introduction

There are **two workspaces**:

* `bfmc_workspace`, which contains the necessary files for running the simulator
* `startup_workspace`, where we provided some useful libraries for running your Python code in the Gazebo simulator

## How to Start

- Install [Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04/). For best performances, avoid using a virtual machine.
- Install [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu). We suggest using the Desktop Full Install. Make sure you follow all the instructions and pay attention to the ***Initialize rosdep*** subchapter.
- Install [Gazebo9](https://zoomadmin.com/HowToInstall/UbuntuPackage/gazebo9).
- Quick [YouTube tutorial for workspace setup](https://www.youtube.com/watch?v=sIydBiat4bc&feature=emb_logo&ab_channel=BoschRomania).
     
### Workspace Setup

Clone the repository inside the `Documents` folder (or perform a commit on the repository)

#### 1. bfmc_workspace

- Open a terminal in `~/Documents/BFMC_Simulator/bfmc_workspace` directory and run the following command:

```sh
catkin_make
```

- Three folders – `src`, `devel`, `build` – will be generated inside the `bfmc_workspace` directory. In order to make this workspace available, you need to source the `setup.bash` file in the same terminal.

```sh
source devel/setup.bash
```

- Gazebo needs to know where the workspace's packages and models are:
    - export `models_pkg` to the `GAZEBO_MODEL_PATH` variable
    - export the `src` folders to `ROS_PACKAGE_PATH` variable

```sh
# You can either run these commands each time you want to use the workspace, or add them to the end of the `~/.bashrc` file and source it.
# Don't forget to replace {YOUR_USER} with your actually user name
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/BFMC_Simulator/bfmc_workspace/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/BFMC_Simulator/bfmc_workspace/src:$ROS_PACKAGE_PATH"
```

- Finally, try running the following command:

```sh
# Depending on your machine, it may take 1 to 3 minutes for the world to completely load.
roslaunch sim_pkg map_with_all_objects.launch
```

- The `[REST.cc:205] ERR` may appear. In order to solve it, see [this thread](https://answers.gazebosim.org/question/25030/gazebo-error-restcc205-error-in-rest-request/?answer=25048#post-id-25048).



#### 2. startup_workspace

- Open a terminal, navigate to the `~/Documents/BFMC_Simulator/startup_workspace` directory and run the following commands:

```sh
source ~/Documents/BFMC_Simulator/startup_workspace/devel/setup.bash 
catkin_make
source devel/setup.bash
```

- With the simulator started, run the following command:

```sh
rosrun startup_package main.py
```

---

# Release notes
- Environmental elements added (people around the map)
- Ramp added;
- Made files executable. (BFMC_Simulator/startup_workspace/src/startup_package/src/main.py; BFMC_Simulator/bfmc_workspace/src/sim_pck/launch/bash_script/delay.sh; BFMC_Simulator/bfmc_workspace/src/traffic_light_pkg/src/tl_talker.py; BFMC_Simulator/bfmc_workspace/src/traffic_light_pkg/src/tl_listener.py)
- Fixed details in guide.
- Added highwy separators launchers.
