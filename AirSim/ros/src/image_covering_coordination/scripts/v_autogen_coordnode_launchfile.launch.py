import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generate a ROS 2 launch description for multiple drones.
    """
    num_robots = int(sys.argv[1])  # Get the number of robots from command-line arguments
    type_name = sys.argv[2]  # Get the node type name from command-line arguments

    # Declare launch arguments
    algorithm_to_run = DeclareLaunchArgument('algorithm_to_run', default_value='RAG_NN', description='Algorithm to run')
    random_seed = DeclareLaunchArgument('random_seed', default_value='0', description='Random seed')
    num_nearest_neighbors = DeclareLaunchArgument('num_nearest_neighbors', default_value='3', description='Number of nearest neighbors')

    drone_nodes = []  # List to store drone nodes
    for i in range(1, num_robots + 1):
        drone_nodes.append(
            Node(
                package='image_covering_coordination',  # Package name
                executable=type_name,  # Node executable type
                name=f'coordination_{i}',  # Node name
                parameters=[{
                    'drone_name': f'Drone{i}',
                    'robot_index_ID': i,
                    'number_of_drones': num_robots,
                    'algorithm_to_run': LaunchConfiguration('algorithm_to_run'),
                    'random_seed': LaunchConfiguration('random_seed'),
                    'num_nearest_neighbors': LaunchConfiguration('num_nearest_neighbors'),
                    'camera_name': 'front_center',
                    'image_topic': f'/camera_{i}/image',
                    'communication_range': 40.0,
                }]
            )
        )

    return LaunchDescription([
        algorithm_to_run,
        random_seed,
        num_nearest_neighbors,
        *drone_nodes  # Unpack the drone nodes list
    ])

# How to run:
# ros2 launch <package_name> generated_launch_file.launch.py --show-args
