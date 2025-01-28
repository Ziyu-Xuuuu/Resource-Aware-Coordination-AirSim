#!/bin/bash

# FOR TESTING WITH ALL 10 DRONES, AirSim wrapper is ODOM only
# ALGORITHM: 
algorithm="RAGH"
echo $algorithm

# Frequencies to iterate over
frequencies=(10)
frequency=20

pub_const_frequencies=(10)
pub_const_frequency=20

counter=0

# NOTE: start roscore in separate terminal
# roscore

# run the unity airsim executable
# /home/sgari/sandilya_ws/unity_airsim_workspace/image_covering_10drones_cam2.x86_64 &
# sleep 4

# /home/sgari/sandilya_ws/unity_airsim_workspace/image_covering_10drones.x86_64 &
# sleep 4

# /home/sgari/sandilya_ws/unity_airsim_workspace/image_covering_10drones_newcam.x86_64 &
# sleep 4

# /home/sgari/sandilya_ws/unity_airsim_workspace/image_covering_10drones_limFOV.x86_64 &
# sleep 4

# set log dir for processing later
log_directory="/home/sgari/sandilya_ws/unity_airsim_workspace/experiments_data_imcov/roslogs/log_${frequency}_${i}_${evadermotion}_${algorithm}"
export ROS_LOG_DIR=$log_directory
echo $ROS_LOG_DIR

# launch main odom nodes
update_control_frequency=$(echo "scale=2; 1/$pub_const_frequency" | bc)
echo $update_control_frequency
# roslaunch airsim_ros_pkgs airsim_node_10quadrotors_imcov_SS.launch update_control_every_n_sec:="$update_control_frequency" &

# odom-only launch file works for any number of drones
roslaunch airsim_ros_pkgs airsim_node_10quadrotors_imcov_odom.launch update_control_every_n_sec:="$update_control_frequency" &
sleep 1

# run clock publisher node
rosrun image_covering_coordination clock_publisher.py _replanning_freq:=$frequency &

# launch pedata 
roslaunch image_covering_coordination connectivity_mesh_neighbors.launch &
# roslaunch image_covering_coordination connectivity_mesh_neighbors_DFS.launch &
sleep 1.5

# launch path msg publisher for rviz
roslaunch image_covering_coordination odom_to_path_nodes_10robots.launch &
sleep 2

# launch rviz for visualization
roslaunch image_covering_coordination rviz.launch &

# # total min distance logger
# python2 /home/sgari/sandilya_ws/unity_airsim_workspace/AirSim/ros/src/multi_target_tracking/scripts/total_min_distance_logger.py "$i" "$algorithm" "$frequency" "$evadermotion" "$pub_const_frequency" &

