#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from math import sqrt
from airsim_ros_pkgs.msg import Neighbors, NeighborsArray

# Communication range
# RANGE = 150  # m. Typical range for 2.4GHz radio control is 100-300m

class MeshNetwork:
    def __init__(self):
        rospy.init_node("connectivity_mesh_node")
        self.drones = {}
        self.neighbors = {}
        self.num_drones = rospy.get_param('~num_drones')
        self.communication_range = rospy.get_param('~communication_range')
        self.numdrones=self.num_drones

        self.neighbors = {i: {"in": set(), "out": set()} for i in range(1, self.numdrones+1)}  # Initialize neighbors for drones
        self.neighbors_pub = rospy.Publisher('neighbors_data', NeighborsArray, queue_size=10)
        self.setup_subscribers()

    def pose_callback(self, msg, args):
        # saves the pose of each drone the self.drones dictionary with id as key
        drone_id = args[0]
        self.drones[drone_id] = msg.pose.pose

    def compute_dist(self, pose_i, pose_j):
        return sqrt((pose_i.position.x - pose_j.position.x) ** 2 +
                                (pose_i.position.y - pose_j.position.y) ** 2 +
                                (pose_i.position.z - pose_j.position.z) ** 2)

    def compute_neighbors(self):
        # these are only pursuers
        # print(self.drones)
        for i, pose_i in self.drones.items():
            for j, pose_j in self.drones.items():
                if i == j:
                    continue

                dist = self.compute_dist(pose_i, pose_j)

                if dist <= self.communication_range:
                    # NOTE: assuming bi-directional communication, adding in-communication drones as both
                    # in and out neighbors
                    self.neighbors[i]["in"].add(j)
                    self.neighbors[i]["out"].add(j)
                    
                    self.neighbors[j]["in"].add(i)
                    self.neighbors[j]["out"].add(i)
        # print(self.neighbors)

    def setup_subscribers(self):
        for i in range(self.numdrones):
            wrapper_str = "/airsim_node_odom_drone"
            # wrapper_str = "/airsim_node_imagecovering_drone"
            posesub_topic = wrapper_str + str(i + 1) + "/Drone" + str(i + 1) + "/odom_local_ned"
            rospy.Subscriber(posesub_topic, Odometry, self.pose_callback, [i + 1])

    def publish_neighbors_data(self):
        neighbors_msg_array = NeighborsArray()
        neighbors_msg_array.neighbors_array = []

        for drone_id, neighbors_data in self.neighbors.items():
            neighbors_msg = Neighbors()
            neighbors_msg.drone_id = drone_id
            neighbors_msg.in_neighbor_ids = list(neighbors_data["in"])
            neighbors_msg.out_neighbor_ids = list(neighbors_data["out"])
            neighbors_msg.header.stamp = rospy.Time.now()
            neighbors_msg_array.neighbors_array.append(neighbors_msg)

        # Publish the array of neighbors data as a single message
        neighbors_msg_array.header.stamp = rospy.Time.now()
        self.neighbors_pub.publish(neighbors_msg_array)

if __name__ == '__main__':
    mesh_network = MeshNetwork()

    # rate = rospy.Rate(5)
    rate = rospy.Rate(2)
    # counter=0
    while not rospy.is_shutdown():
        mesh_network.compute_neighbors()
        mesh_network.publish_neighbors_data()
        # counter+=1
        # print(counter)
        rate.sleep()
