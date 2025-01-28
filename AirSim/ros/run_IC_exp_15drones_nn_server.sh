#!/bin/bash

# Frequencies to iterate over
frequencies=(10)
frequency=20

pub_const_frequencies=(10)
pub_const_frequency=20

# Set log directory for processing later (adjusted inside the loop)
log_directory_base="/mnt/ros/logs"

# Define the algorithms to iterate over
algorithms=("RAG_NN")

# BATCH 1
num_nearest_neighbors_list=(0 1 2 3 4 5 6 7)

# Outer loop to iterate over algorithms
for algorithm in "${algorithms[@]}"; do
    echo "Running algorithm: $algorithm"

    # Loop over 0-5 nearest neighbors
    # for num_nearest_neighbors in $(seq 0 5); do
    for num_nearest_neighbors in "${num_nearest_neighbors_list[@]}"; do
        echo "Running with num_nearest_neighbors: $num_nearest_neighbors"

        # Loop over random seeds
        for random_seed in $(seq 1 50); do
            echo "Running with random seed: $random_seed"

             # Start roscore
            roscore &
            sleep 5

            # Start the unity airsim executable
            /home/airsim_user/image_covering_15drones_limFOV_nogui_limmeshes.x86_64 &
            sleep 5

            # Set and export log directory for this iteration NOTE!!! REMEMBER TO RENAME THE FOLDER BASED ON SPEED AT THE END
            log_directory="${log_directory_base}/log_${random_seed}_${algorithm}_${num_nearest_neighbors}nn_0p2mbps"
            export ROS_LOG_DIR=$log_directory
            echo $ROS_LOG_DIR

            # Launch main odom nodes
            update_control_frequency=$(echo "scale=2; 1/$pub_const_frequency" | bc)
            echo $update_control_frequency
            roslaunch airsim_ros_pkgs airsim_node_10quadrotors_imcov_odom.launch update_control_every_n_sec:="$update_control_frequency" &
            sleep 1

            # Run clock publisher node
            rosrun image_covering_coordination clock_publisher.py _replanning_freq:=$frequency &

            # Launch connectivity mesh neighbors
            roslaunch image_covering_coordination connectivity_mesh_neighbors.launch num_drones:=15 &
            sleep 1.5

            # Launch path msg publisher for rviz
            # roslaunch image_covering_coordination odom_to_path_nodes_15robots.launch &
            # sleep 2

            # Launch rviz for visualization
            # roslaunch image_covering_coordination rviz.launch &
            # sleep 1.5

            # Execute the algorithm with the current random seed
            roslaunch image_covering_coordination image_covering_15drones_RAG.launch algorithm_to_run:=$algorithm random_seed:=$random_seed num_nearest_neighbors:=$num_nearest_neighbors is_server_experiment:=1 &
            sleep 2

            roslaunch image_covering_coordination total_score_collective_FOV_DEBUG.launch algorithm_to_run:=$algorithm experiment_number:=$random_seed num_nearest_neighbors:=$num_nearest_neighbors is_server_experiment:=1 &
            
            # NOTE!!! SET SLEEP TIME BASED ON THE EVADER MOTION
            sleep 550

            # kill all ros nodes after you know data has been logged. NOTE: It restarts roscore
            rosnode kill -a
            sleep 5

            roslaunch_ids=$(pgrep -f roslaunch) 
            if [ -n "$roslaunch_ids" ]; then
                kill -9 $roslaunch_ids
            fi

            airsim_ids=$(pgrep -f airsim) 
            if [ -n "$airsim_ids" ]; then
                kill -9 $airsim_ids
            fi

            # Find and kill the unity process NOTE!!! ensure that
            first_process_pid=$(pgrep -o image_covering)
            if [ -n "$first_process_pid" ]; then
                echo "Closing first process"
                kill -9 "$first_process_pid"
                # wait "$first_process_pid" 2>/dev/null
            else
                echo "First process not found or already closed"
            fi
            
            sleep 5

            # double check unity is killed 
            first_process_pid=$(pgrep -o image_covering)
            if [ -n "$first_process_pid" ]; then
                echo "Closing first process"
                kill -9 "$first_process_pid"
                # wait "$first_process_pid" 2>/dev/null
            else
                echo "First process not found or already closed"
            fi

            airsim_ids=$(pgrep -f airsim) 
            if [ -n "$airsim_ids" ]; then
                kill -9 $airsim_ids
            fi

            sleep 5

        done
    done

done # End of algorithms loop
