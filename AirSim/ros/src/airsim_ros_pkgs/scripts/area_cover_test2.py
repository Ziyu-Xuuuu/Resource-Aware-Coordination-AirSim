#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

rospy.init_node('image_subscriber_node', anonymous=True)

def image_callback(image):
    print("Received an Image")
    # Your processing logic here

def odom_callback(odom):
    print("Received odom data")
    # Your processing logic here

rospy.Subscriber("/airsim_node_imagecovering_drone1/Drone1/front_center/Scene", Image, image_callback)
rospy.Subscriber("/airsim_node_imagecovering_drone1/Drone1/odom_local_ned", Odometry, odom_callback)

rospy.spin()
