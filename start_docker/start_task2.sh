#!/bin/bash

#### INIT

# declare workspace path for easier syntax
WORKSPACE_PATH=/CMR_ws

# variable for sourcing commands for ROS and workspace setup files
SOURCE_COMMANDS="source /opt/ros/melodic/setup.bash; source $WORKSPACE_PATH/devel/setup.bash"

# arbitrary duration of the bag recording
BAG_REC_DURATION=20
    
#### ROS initialization - First steps before running the simulations

# source ros bash files and start ros master (roscore)
terminator -e "bash -c '$SOURCE_COMMANDS; \
roscore; exec bash'" &
sleep 3

# set ROS parameters
terminator -e "bash -c '$SOURCE_COMMANDS; \
rosparam set /use_sim_time true; exec bash'" &
sleep 1

#### Question 3 - Running the trajectory tracking simulation on the dynamic model (with linear tyre model)

# start recording data ros bag
terminator -e " bash -c '$SOURCE_COMMANDS; \
cd $WORKSPACE_PATH/src/car_traj_ctrl/script; \
rosbag record -O traj_ctrl_results_dyn.bag -a --duration=$BAG_REC_DURATION; exec bash'" &
sleep 2

# launch the node
terminator -e "bash -c '$SOURCE_COMMANDS; \
cd $WORKSPACE_PATH/src/car_traj_ctrl/script; \
roslaunch car_traj_ctrl car_traj_ctrl.launch; exec bash'" &

sleep $BAG_REC_DURATION
sleep 10

# after recording shut down the launch file
pkill -f "roslaunch car_traj_ctrl car_traj_ctrl.launch"
sleep 2

# plot results after recording has finished
terminator -e " bash -c '$SOURCE_COMMANDS; \
cd $WORKSPACE_PATH/src/car_traj_ctrl/script; \
python plot_result_trajctrl_dyn.py \"traj_ctrl_results_dyn.bag\"; exec bash'"

sleep infinity