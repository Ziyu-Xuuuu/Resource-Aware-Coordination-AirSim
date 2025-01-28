#!/usr/bin/env python2

import rospy
# from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.msg
# from nav_msgs.msg import Odometry
import nav_msgs.msg
from message_filters import ApproximateTimeSynchronizer, Subscriber


rospy.init_node('image_to_pointcloud_node', anonymous=False)

def gotdata(image, odom):
    assert image.header.stamp == odom.header.stamp
    print("got an Image and Odom")

image_sub = Subscriber("/airsim_node_imagecovering_drone1/Drone1/front_center/Scene", sensor_msgs.msg.Image)
odom_sub = Subscriber("/airsim_node_imagecovering_drone1/Drone1/odom_local_ned", nav_msgs.msg.Odometry)

ats = ApproximateTimeSynchronizer([image_sub, odom_sub], queue_size=1000, slop=100)
ats.registerCallback(gotdata)
rospy.spin()