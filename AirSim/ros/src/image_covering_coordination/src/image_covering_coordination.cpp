// C++ headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// Custom message headers
#include <airsim_ros_pkgs/Neighbors.h>
#include <airsim_ros_pkgs/NeighborsArray.h>

// Airsim library
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

// #include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "ros/ros.h"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <math_common.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <unordered_map>
#include <memory>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/QuaternionStamped.h>


class ImageCoveringCoordination
{

private:
    // ROS node handle
    ros::NodeHandle node_handle_;

    // ROS subscribers
    ros::Subscriber connectivity_mesh_sub_;
    ros::Subscriber odom_sub_;

    // ROS publishers
    ros::Publisher image_pub_;

    // TF listener
    // tf::TransformListener* tf_listener_;
    // TF2 listener: Buffer not recognized as a type
    // tf2_ros::Buffer tfBuffer_;
    // tf2_ros::TransformListener tf2_listener_(tfBuffer_);

    // Class variables
    std::string drone_name_self_;
    std::string camera_name_;
    double communication_range_;

public:
    ImageCoveringCoordination()
    {

        // Get ROS params
        ros::param::get("~drone_name", drone_name_self_);
        ros::param::get("~camera_name", camera_name_);
        ros::param::get("~communication_range", communication_range_);

        // ROS publishers
        image_pub_ = node_handle_.advertise<sensor_msgs::Image>("/covered_image", 10);

        // ROS subscribers
        connectivity_mesh_sub_ = node_handle_.subscribe("/neighbors_data", 500, &ImageCoveringCoordination::connectivityMeshCallback, this);
        odom_sub_ = node_handle_.subscribe("/odom", 100, &ImageCoveringCoordination::odomCallback, this);

        // // TF listener
        // tf_listener_ = new tf::TransformListener();  
    }

    // Callback functions
    void connectivityMeshCallback(const airsim_ros_pkgs::NeighborsArray& msg); // Save connectivity data

    void collectRotatedCameraImages(const airsim_ros_pkgs::NeighborsArray& msg); 

    void odomCallback(const nav_msgs::Odometry& msg); // Get pose

};

// Save connectivity data
void ImageCoveringCoordination::connectivityMeshCallback(const airsim_ros_pkgs::NeighborsArray& msg) {

} 

// Rotate camera to 8 predefined poses and save SS images from each pose to a private var
void ImageCoveringCoordination::collectRotatedCameraImages(const airsim_ros_pkgs::NeighborsArray& msg) {

}

// Get pose
void ImageCoveringCoordination::odomCallback(const nav_msgs::Odometry& msg) {

}


int main(int argc, char** argv)
{

    // Initialize ROS node
    ros::init(argc, argv, "image_covering_coordination_node");

    ImageCoveringCoordination node;

    ros::spin();

    return 0;
}