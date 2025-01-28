#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>

class AreaOccupancyHashing {
private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher grid_pub;
    std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> occupied_cells;
    double grid_resolution; // Resolution of the grid

public:
    AreaOccupancyHashing() : grid_resolution(0.09123) { // Set your desired grid resolution
        pcl_sub = nh.subscribe("/input_point_cloud", 1, &AreaOccupancyHashing::pointCloudCallback, this);
        grid_pub = nh.advertise<sensor_msgs::PointCloud2>("/occupancy_grid", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *cloud);

        // Process each point in the point cloud
        for (const auto& point : cloud->points) {
            int cell_x = static_cast<int>(point.x / grid_resolution);
            int cell_y = static_cast<int>(point.y / grid_resolution);

            std::pair<int, int> cell_key = std::make_pair(cell_x, cell_y);

            // Check if cell is already marked as occupied
            if (occupied_cells.find(cell_key) == occupied_cells.end()) {
                // Mark cell as occupied
                occupied_cells.insert(cell_key);

                // Do additional processing or marking of the cell in your occupancy grid here
            }
        }

        // Publish the occupancy grid (for visualization or further processing)
        sensor_msgs::PointCloud2 occupancy_grid;
        // Populate occupancy_grid message with the marked cells
        // ...

        grid_pub.publish(occupancy_grid);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "area_occupancy_hashing");
    AreaOccupancyHashing AreaOccupancyHashing;
    ros::spin();
    return 0;
}
