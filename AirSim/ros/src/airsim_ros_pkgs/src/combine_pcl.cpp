#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <fstream>
#include <iostream>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class CombinePointCloud {
private:
    ros::Subscriber image_sub;
    std::string pclsub_topic;
    bool isFirstMessage;
    pcl::PointCloud<pcl::PointNormal>::Ptr buffer; // Buffer to store points from the first message

    ros::Publisher pub;

public:
    CombinePointCloud() : isFirstMessage(true), buffer(new pcl::PointCloud<pcl::PointNormal>) {
        ros::NodeHandle nh;
        pclsub_topic = "/area_coverage_pointcloud";
        image_sub = nh.subscribe(pclsub_topic, 1, &CombinePointCloud::pclCallback, this);
        pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 100); // Advertise the publisher
    }
    
    bool comparePoint(const pcl::PointNormal& p1, const pcl::PointNormal& p2);
    bool equalPoint(const pcl::PointNormal& p1, const pcl::PointNormal& p2);

    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);

    ~CombinePointCloud() {} // destructor
};

bool CombinePointCloud::comparePoint(const pcl::PointNormal& p1, const pcl::PointNormal& p2) {
    // comparison logic here, comparing the distance between points
    // compare based on Euclidean distance between points
    return false;
}

bool CombinePointCloud::equalPoint(const pcl::PointNormal& p1, const pcl::PointNormal& p2) {
    // Define your equality logic here, checking if points are equal
    return false;
}

void CombinePointCloud::pclCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    if(point_cloud_msg->data.empty())
    {
      ROS_WARN("Received an empty cloud message. Skipping further processing");
      return;
    }

    // Convert ROS message to PCL-compatible data structure
    ROS_INFO_STREAM("Received a cloud message with " << point_cloud_msg->height * point_cloud_msg->width << " points");
    ROS_INFO("Converting ROS cloud message to PCL compatible data structure");
    pcl::PointCloud<pcl::PointNormal> pclCloud;
    pcl::fromROSMsg(*point_cloud_msg, pclCloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr p(new pcl::PointCloud<pcl::PointNormal>(pclCloud));

    // If it's the first message, store the points in the buffer
    if (isFirstMessage) {
        *buffer = *p;
        isFirstMessage = false;
    } 
    else {
        // Combine points from the buffer and new points
        *p += *buffer;

        // Remove duplicates
        ROS_INFO("Removing duplicates from combined point cloud");
        std::sort(p->begin(), p->end(), [this](const pcl::PointNormal& p1, const pcl::PointNormal& p2) {
            return this->comparePoint(p1, p2);
        });

        auto unique_end = std::unique(p->begin(), p->end(), [this](const pcl::PointNormal& p1, const pcl::PointNormal& p2) {
            return this->equalPoint(p1, p2);
        });

        p->erase(unique_end, p->end());

        // Update the buffer with combined unique points
        *buffer = *p;
    }



    // for publishing after removing duplicates
    // Convert result back to ROS message and publish it
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*buffer, output);
      ROS_INFO_STREAM("Publishing a cloud message with " << output.height * output.width << " points");
      ROS_INFO_STREAM("Total area covered (m^2): "<< output.height * output.width * 0.09123);
      pub.publish(output);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "combine_pcls");
    CombinePointCloud combinePointCloud;
    ros::spin();
    return 0;
}
