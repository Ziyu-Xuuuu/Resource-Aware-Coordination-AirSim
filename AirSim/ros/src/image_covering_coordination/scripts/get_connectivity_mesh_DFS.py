#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from math import sqrt, atan2, degrees
from airsim_ros_pkgs.msg import Neighbors, NeighborsArray
from geometry_msgs.msg import Quaternion
import tf
import networkx as nx
from collections import deque

# Communication range
# RANGE = 150  # m. Typical range for 2.4GHz radio control is 100-300m

class MeshNetwork:
    def __init__(self):
        rospy.init_node("connectivity_mesh_node")
        self.drones = {}
        self.neighbors = {}
        self.drones_received = set()
        self.num_drones = rospy.get_param('~num_drones')
        self.communication_range = rospy.get_param('~communication_range')

        # for debugging
        # self.num_drones =10
        # self.communication_range=150

        self.numdrones=self.num_drones

        self.neighbors = {i: {"in": set(), "out": set()} for i in range(1, self.numdrones+1)}  # Initialize neighbors for drones
        
        self.neighbors_range_bearing = {i: {"robots_range_bearing_ids": set(), "robots_range_bearing_values": list()} for i in range(1, self.numdrones+1)}
        
        self.neighbors_pub = rospy.Publisher('/neighbors_data', NeighborsArray, queue_size=10)

        self.graph = nx.Graph()

        self.setup_subscribers()

    def update_graph(self):
        self.graph.clear()
        for i in self.drones:
            for j in self.neighbors[i]["out"]:
                self.graph.add_edge(i, j)

    def dfs_order(self):
        def dfs_visit(node, parent):
            if 'visited' not in self.graph.nodes[node]:
                self.graph.nodes[node]['visited'] = True
                self.graph.nodes[node]['parent'] = parent
                for neighbor in self.graph[node]:
                    if 'visited' not in self.graph.nodes[neighbor]:
                        dfs_visit(neighbor, node)

        # Initialize all nodes as not visited
        for node in self.graph.nodes():
            if 'visited' in self.graph.nodes[node]:
                del self.graph.nodes[node]['visited']

        for node in self.graph.nodes():
            if 'visited' not in self.graph.nodes[node]:
                dfs_visit(node, None)

        order = 0
        for node in nx.dfs_preorder_nodes(self.graph):
            self.graph.nodes[node]['dfs_order'] = order
            order += 1

        # Assign decision_making_id based on DFS order
        for node in self.graph.nodes():
            decision_making_id = self.graph.nodes[node]['dfs_order'] + 1 # adding 1 so that the indexing starts at 1, just like drone id

            # Here you can set decision_making_id for each drone
            # print("Drone {} has decision_making_id {}".format(node, decision_making_id))

        print(" ")

    def pose_callback(self, msg, args):
        # saves the pose of each drone the self.drones dictionary with id as key
        drone_id = args[0]
        self.drones[drone_id] = msg.pose.pose
        self.drones_received.add(drone_id)

    def compute_dist(self, pose_i, pose_j):
        return sqrt((pose_i.position.x - pose_j.position.x) ** 2 +
                                (pose_i.position.y - pose_j.position.y) ** 2 +
                                (pose_i.position.z - pose_j.position.z) ** 2)

    def compute_neighbors(self):
        # print(self.drones)
        for i, pose_i in self.drones.items():
            self_drones_rb_values = []  # Reset for each drone i
            self_drones_rb_ids = []

            for j, pose_j in self.drones.items():
                if i == j:
                    continue

                dist = self.compute_dist(pose_i, pose_j)
                bearing = self.calculate_bearing(pose_i, pose_j) # radians
                self_drones_rb_values.append(dist)
                self_drones_rb_values.append(bearing)
                self_drones_rb_ids.append(j)

                if dist <= self.communication_range:
                    # NOTE: assuming bi-directional communication, adding in-communication drones as both
                    # in and out neighbors
                    self.neighbors[i]["in"].add(j)
                    self.neighbors[i]["out"].add(j)
                    
                    self.neighbors[j]["in"].add(i)
                    self.neighbors[j]["out"].add(i)


            # Update Neighbors message for drone i
            self.neighbors_range_bearing[i]['robots_range_bearing_ids'] = self_drones_rb_ids
            self.neighbors_range_bearing[i]['robots_range_bearing_values'] = self_drones_rb_values
        # print(self.neighbors)

    def setup_subscribers(self):
        for i in range(self.numdrones):
            # FOR ODOM ONLY WRAPPER
            posesub_topic = "/airsim_node_odom" + "/Drone" + str(i + 1) + "/odom_local_ned"

            # FOR SS WRAPPER
            # wrapper_str = "/airsim_node_SS_drone"
            # wrapper_str = "/airsim_node_imagecovering_drone"
            # posesub_topic = wrapper_str + str(i + 1) + "/Drone" + str(i + 1) + "/odom_local_ned"


            rospy.Subscriber(posesub_topic, Odometry, self.pose_callback, [i + 1])

    def publish_neighbors_data(self):
        neighbors_msg_array = NeighborsArray()
        neighbors_msg_array.neighbors_array = []

        for drone_id, neighbors_data in self.neighbors.items():
            neighbors_msg = Neighbors()
            neighbors_msg.drone_id = drone_id

            # Retrieve and set the decision_making_id for the drone
            neighbors_msg.decision_making_id = self.graph.nodes[drone_id]['dfs_order'] + 1
            print("Drone {} has decision_making_id {}".format(drone_id, neighbors_msg.decision_making_id))

            neighbors_msg.in_neighbor_ids = list(neighbors_data["in"])
            neighbors_msg.out_neighbor_ids = list(neighbors_data["out"])
            neighbors_msg.header.stamp = rospy.Time.now()

            # Additional code for graph nodes and edges
            if drone_id == 1:  # Only for drone_id 1
                graph_nodes = list(self.graph.nodes())
                # Flatten the graph edges into a list
                graph_edges = [node for edge in self.graph.edges() for node in edge]
                neighbors_msg.graph_nodes = graph_nodes
                neighbors_msg.graph_edges = graph_edges

            quaternion =Quaternion(x=self.drones[drone_id].orientation.x, y=self.drones[drone_id].orientation.y, z=self.drones[drone_id].orientation.z, 
                w=self.drones[drone_id].orientation.w)
            euler_angles = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            _, _, yaw = euler_angles

            neighbors_msg.self_drone_pose = [self.drones[drone_id].position.x, self.drones[drone_id].position.y, self.drones[drone_id].position.z, -yaw]
            
            neighbors_msg.robots_range_bearing_ids = self.neighbors_range_bearing[drone_id]["robots_range_bearing_ids"]
            neighbors_msg.robots_range_bearing_values = self.neighbors_range_bearing[drone_id]["robots_range_bearing_values"]
            
            neighbors_msg_array.neighbors_array.append(neighbors_msg)

        # Publish the array of neighbors data as a single message
        neighbors_msg_array.header.stamp = rospy.Time.now()
        self.neighbors_pub.publish(neighbors_msg_array)

    def compute_range_bearing(self, pose_i, pose_j):
        # NOTE bearing goes from x1,y1 of pose_i to x2,y2 of pose_j, in RADIANS
        # Calculate distance between poses 
        dx = pose_j.position.x - pose_i.position.x
        dy = pose_j.position.y - pose_i.position.y
        range = sqrt(dx**2 + dy**2)

        # Calculate bearing in RADIANS
        # bearing = atan2(dx, dy)
        bearing = atan2(dy, dx)

        return range, bearing

    def calculate_bearing(self, pose1, pose2):
        # Calculate bearing from pose1 to pose2
        dy = pose2.position.y - pose1.position.y
        dx = pose2.position.x - pose1.position.x
        bearing = atan2(dy, dx)  # Bearing in radians
        # return degrees(bearing)  # Convert to degrees if needed
        return bearing

    def compute_neighbors_and_order(self):
        self.compute_neighbors()
        self.update_graph()
        self.dfs_order()
        self.publish_neighbors_data()

if __name__ == '__main__':
    mesh_network = MeshNetwork()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if len(mesh_network.drones_received) == mesh_network.numdrones:
            mesh_network.compute_neighbors_and_order()
            mesh_network.drones_received.clear()
            rate.sleep()
