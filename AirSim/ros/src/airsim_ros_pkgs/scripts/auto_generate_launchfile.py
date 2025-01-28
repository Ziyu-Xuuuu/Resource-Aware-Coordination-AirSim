import sys

num_pursuers = int(sys.argv[1]) 
num_evaders = int(sys.argv[2])
type_pursuers = sys.argv[3]
type_evaders = sys.argv[4]

pursuer_nodes = ""
for i in range(1, num_pursuers+1):
  pursuer_nodes += f"""
  <!-- Drone{i} -->
  <node name="{type_pursuers}_drone{i}" pkg="airsim_ros_pkgs" type="{type_pursuers}" output="$(arg output)">
    <param name="is_vulkan" type="bool" value="true" />
    <param name="update_airsim_img_response_every_n_sec" type="double" value="$(arg update_airsim_img_response_every_n_sec)" />
    <param name="update_airsim_control_every_n_sec" type="double" value="$(arg update_control_every_n_sec)" />
    <param name="publish_clock" type="bool" value="$(arg publish_clock)" />
    <param name="host_ip" type="string" value="$(arg host)" />
    <param name="drone_name" type="string" value="Drone{i}" />
  </node>
"""

evader_nodes = ""
for i in range(num_pursuers+1, num_pursuers+num_evaders+1):
  evader_nodes += f"""
  <!-- Drone{i} -->
  <node name="{type_evaders}_drone{i}" pkg="airsim_ros_pkgs" type="{type_evaders}" output="$(arg output)">
    <param name="is_vulkan" type="bool" value="true" />
    <param name="update_airsim_img_response_every_n_sec" type="double" value="$(arg update_airsim_img_response_every_n_sec)" />
    <param name="update_airsim_control_every_n_sec" type="double" value="$(arg update_control_every_n_sec)" />
    <param name="publish_clock" type="bool" value="$(arg publish_clock)" />
    <param name="host_ip" type="string" value="$(arg host)" />
    <param name="drone_name" type="string" value="Drone{i}" />
  </node>  
"""

launch_file_contents = f"""
<launch>
  <arg name="output" default="log"/>
  <arg name="publish_clock" default="false"/>
  <arg name="is_vulkan" default="true"/>
  <arg name="host" default="localhost" />
  <arg name="update_control_every_n_sec" default="0.1" />
	<arg name="update_airsim_img_response_every_n_sec" default="0.1" />


  {pursuer_nodes}

  {evader_nodes}

  <!-- Static transforms -->
  <include file="$(find airsim_ros_pkgs)/launch/static_transforms.launch"/>  
</launch>
"""

with open('generated.launch', 'w') as f:
  f.write(launch_file_contents)