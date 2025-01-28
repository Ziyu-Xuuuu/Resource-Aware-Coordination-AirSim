import sys

num_robots = int(sys.argv[1])
type_name = sys.argv[2]

drone_nodes = ""
for i in range(1, num_robots+1):
  drone_nodes += f"""
  <node pkg="image_covering_coordination" type="{type_name}" name="coordination_{i}">
    <param name="drone_name" value="Drone{i}"/>
    <param name="robot_index_ID" type="int" value="{i}"/>
    <param name="number_of_drones" value="{num_robots}"/>
    <param name="algorithm_to_run" value="$(arg algorithm_to_run)"/>
    <param name="random_seed" value="$(arg random_seed)"/>
    <param name="num_nearest_neighbors" value="$(arg num_nearest_neighbors)"/>

    <param name="camera_name" value="front_center"/>
    <param name="image_topic" value="/camera_{i}/image"/>
    <param name="communication_range" type="double" value="40"/>
  </node>
"""

launch_file_contents = f"""
<launch>
  <arg name="algorithm_to_run" default="RAG_NN"/>
  
  <arg name="random_seed" default="0"/>

  <arg name="num_nearest_neighbors" default="3"/>

  {drone_nodes}

</launch>
"""

with open('generated.launch', 'w') as f:
  f.write(launch_file_contents)

# how to run :
# python generate_launch_file.py <num_robots> <type_name>
