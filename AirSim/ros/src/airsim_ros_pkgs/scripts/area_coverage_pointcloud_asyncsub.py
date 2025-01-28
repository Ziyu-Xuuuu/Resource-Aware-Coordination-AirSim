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

""" Camera parameters
header: 
  seq: 9271
  stamp: 
    secs: 1701
    nsecs: 212518307
  frame_id: "front_center/Scene_optical"
height: 144
width: 256
distortion_model: ''
D: []
K: [128.0, 0.0, 128.0, 0.0, 128.0, 72.0, 0.0, 0.0, 1.0]
R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P: [128.0, 0.0, 128.0, 0.0, 0.0, 128.0, 72.0, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
---
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]

FOV calculation:
w, h = 1280, 720
fx, fy = 1027.3, 1026.9

# Go
fov_x = np.rad2deg(2 * np.arctan2(w, 2 * fx))
fov_y = np.rad2deg(2 * np.arctan2(h, 2 * fy))

"""

class ImageToPointCloud:
    def __init__(self):    
        rospy.init_node('image_to_pointcloud_node', anonymous=False)
        self.drone_name = rospy.get_param('~drone_name')
        # print(self.drone_name)
        imagesub_topic = "/airsim_node_imagecovering_" + self.drone_name + "/D" + self.drone_name[1:] + "/front_center/Scene"
        self.image_sub = rospy.Subscriber(imagesub_topic, Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/airsim_node_imagecovering_drone1/Drone1/front_center/Scene", Image, self.image_callback)

        odomsubtopic = "/airsim_node_imagecovering_" + self.drone_name + "/D" + self.drone_name[1:] + "/odom_local_ned"
        self.odom_sub = rospy.Subscriber(odomsubtopic, Odometry, self.odom_callback)
        # self.odom_sub = rospy.Subscriber("/airsim_node_imagecovering_drone1/Drone1/odom_local_ned", Odometry, self.odom_callback)

        self.bridge = CvBridge()
        self.global_pointcloud = PointCloud2()
        
        self.pc_pub = rospy.Publisher('/area_coverage_pointcloud', PointCloud2, queue_size=100)

        # airsim camera FOV angles
        self.fov_y = 1.0248 # in rads; 58.716 deg
        self.fov_x = 1.5708 # in rads; 90.0 deg

        # quadrotor pre-set flight altitude: 38.66 m. Area calculated based on this
        self.altitude = 38.66 #m
        self.area_fov = 4 * self.altitude * self.altitude * math.tan(self.fov_y / 2) * math.tan(self.fov_x / 2) 

        self.pose_transform = None  #initialized to none for now
        self.odom_callback_throttle_counter = 10
        # self.tflistener = tf.TransformListener()
        # print("Initialization complete, waiting for messages...")
        
    def image_callback(self, image_msg):
        # Assume image is in BGR format
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        pointcloud = self.create_pointcloud(cv_image)
    
        self.pc_pub.publish(pointcloud)
        # print("Received IMAGE msg")

    def odom_callback(self, odom_msg):
        
        if not (self.odom_callback_throttle_counter == 100):
            self.odom_callback_throttle_counter+=1
            return
        else:
            # Process odometry message to get pose transform
            pose_transform = self.get_pose_transform(odom_msg)
            self.pose_transform = pose_transform # assign class variable here for access from image_callback
            self.odom_callback_throttle_counter=0
            # print("Received ODOM msg")

    def convert_image_msg_to_cv2(self, image_msg):
        # Convert ROS Image to OpenCV format
        # Assuming image is in BGR8 encoding
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return cv_image

    def get_pose_transform(self, odom_msg):
        position= odom_msg.pose.pose.position
        orientation_quat = odom_msg.pose.pose.orientation
        orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        euler_angles = tf.transformations.euler_from_quaternion(orientation_list)
        yaw = euler_angles[2]
        rotation_matrix = tf.transformations.euler_matrix(0, 0, yaw)
        rotation_matrix = rotation_matrix[:3, :3]

        # translation_matrix = np.array([[1, 0, position.x],
        #                             [0, 1, position.y],
        #                             [0, 0, 1]])
        translation_matrix = np.array([[1, 0, position.y],
                                    [0, 1, -position.x],
                                    [0, 0, 1]])
        transform_matrix = np.matmul(translation_matrix, rotation_matrix)
        return transform_matrix
    
    def create_pointcloud(self, cv_image):
        # Extract color array
        # colors = cv_image.reshape(-1,3).tolist()
        colors = cv_image.reshape(-1,3) 
        colors_unint = self.pack_rgba_to_uint(colors)

        # Create fields with color 
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 16, PointField.UINT32, 1),
                ]

        # Create header and depth array 
        header = Header()
        header.stamp = rospy.Time.now()
        # header.frame_id = "Drone1"
        header.frame_id = "world_ned"
        # header.frame_id = "Drone1/odom_local_ned"

        height, width = cv_image.shape[:2]
        depth_array = depth_array = np.full((height, width), 0.0)
        depth_array = depth_array.reshape(-1)
        
        # find covered area given by camera parameters and altitude of drone
        # xyarr1 = self.generate_coordinates(rows=144, cols=256, cell_spacing=0.09123)
        xy_array = self.generate_coordinates_centered(rows=144, cols=256, cell_spacing=0.09123) # transform to world frame based on updated pose
        xyz_array = np.column_stack((xy_array, depth_array) )

        xyzrgb_arr = np.column_stack((xyz_array, colors_unint))
        # Create pointcloud
        pointcloud = pc2.create_cloud(header, fields, xyzrgb_arr)
        
        return pointcloud
    
    def generate_coordinates(self, rows, cols, cell_spacing):
        # Create grid indices
        grid_indices = np.indices((rows, cols), dtype=float)

        # Calculate coordinates
        x_coords = grid_indices[1] * cell_spacing
        y_coords = grid_indices[0] * cell_spacing

        # Reshape coordinates into a 2-column array
        coordinates = np.column_stack((x_coords.ravel(), y_coords.ravel()))

        return coordinates

    def generate_coordinates_centered(self, rows, cols, cell_spacing):
        # Create grid indices centered around 0,0
        grid_indices = np.indices((rows, cols), dtype=float)

        # Calculate coordinates from the center
        x_coords = (grid_indices[1] - cols // 2) * cell_spacing
        y_coords = (grid_indices[0] - rows // 2) * -cell_spacing

        # Reshape coordinates into a 2-column array
        coordinates = np.column_stack((x_coords.ravel(), y_coords.ravel()))

        # Combine translation and rotation into a single transformation matrix
        transform_matrix = self.pose_transform
        transform_matrix_str = str(transform_matrix)
        log_message = "TRANSFORM MATRIX: {}".format(transform_matrix_str)
        #rospy.loginfo_throttle(1, log_message)

        # Apply transformation to coordinates
        ones_column = np.ones((len(coordinates), 1))
        # homogeneous_coords = np.hstack((coordinates, np.zeros_like(coordinates[:, :1]), ones_column))
        homogeneous_coords = np.hstack((coordinates, ones_column))
        transform_matrix =  transform_matrix.astype(float)
        homogeneous_coords = homogeneous_coords.astype(float)

        transformed_coords = np.matmul(transform_matrix, homogeneous_coords.T).T[:, :2]

        upper_left_coords_msg = "UPPER LEFT CORNER (x, y): {}".format(transformed_coords[0])
        #rospy.loginfo_throttle(1, upper_left_coords_msg)

        center_coords_msg = "CENTER (x, y): {}".format(transformed_coords[18560])
        #rospy.loginfo_throttle(1, center_coords_msg)

        return transformed_coords

    def pack_rgba_to_uint(self, arr):
        # Extracting RGBA channels
        r = arr[:, 0]
        g = arr[:, 1]
        b = arr[:, 2]

        b = b.flatten() 
        g = g.flatten()
        r = r.flatten()
        # a = a.flatten()

        # Packing RGBA values into a single unsigned integer per row
        packed = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | (b.astype(np.uint32))
        return packed

if __name__ == '__main__':
    try:
        image_to_pointcloud = ImageToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error")
