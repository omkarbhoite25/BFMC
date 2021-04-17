if [ -d ~/Documents/BFMC_Simulator/bfmc_workspace ] 
then
    cd ~/Documents/BFMC_Simulator/bfmc_workspace
    catkin_make
    source devel/setup.bash
    export GAZEBO_MODEL_PATH="~/Documents/BFMC_Simulator/bfmc_workspace/src/models_pkg:$GAZEBO_MODEL_PATH"
    export ROS_PACKAGE_PATH="~/Documents/BFMC_Simulator/bfmc_workspace/src:$ROS_PACKAGE_PATH"
else
  #just for omkar
    cd ~/bfmc/BFMC_Simulator/bfmc_workspace
    catkin_make
    source devel/setup.bash
    export GAZEBO_MODEL_PATH="/home/omkar/bfmc/BFMC_Simulator/bfmc_workspace/src/models_pkg:$GAZEBO_MODEL_PATH"
    export ROS_PACKAGE_PATH="/home/omkar/bfmc/BFMC_Simulator/bfmc_workspace/src:$ROS_PACKAGE_PATH"
fi

roslaunch sim_pkg map_with_all_objects.launch
