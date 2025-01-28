#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import tf
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2
# from tempfile import TemporaryFile

class PointCloudToNpy:
    def __init__(self):    
        rospy.init_node('save_pointcloud_npy', anonymous=False)
        pclsub_topic = "/area_coverage_pointcloud"
        self.image_sub = rospy.Subscriber(pclsub_topic, PointCloud2, self.pcl_callback)
        self.frame = np.zeros((0, 6))
        
    def pcl_callback(self, ros_point_cloud):
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)
        
        comb = np.hstack((xyz, rgb))
        with open('moving_frame_6quadrotor_2.npy', 'wb') as f:
            np.save(f, comb)

        print("saved pointcloud data as numpy array")

if __name__ == '__main__':
    try:
        image_to_pointcloud = PointCloudToNpy()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error")
