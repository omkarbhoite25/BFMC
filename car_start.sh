if [ -d ~/Documents/BFMC_Simulator/startup_workspace ] 
then
  cd ~/Documents/BFMC_Simulator/startup_workspace
  source ~/Documents/BFMC_Simulator/bfmc_workspace/devel/setup.bash
else
  #just for omkar
  cd /home/omkar/bfmc/BFMC_Simulator/startup_workspace
  source /home/omkar/bfmc/BFMC_Simulator/bfmc_workspace/devel/setup.bash
fi

catkin_make
source devel/setup.bash
rosrun startup_package main.py --track RACE --src=86 --dst=216 --ros=True
