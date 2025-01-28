// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

int main()
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
        client.confirmConnection();

        std::string drone_name;
        std::cout << "Enter the name of quadrotor e.g. Drone1" << std::endl;
        std::cin >> drone_name;

        std::cout << "Press Enter to get FPV image from " << drone_name << std::endl;
        std::cin.get();
        // const std::vector<ImageRequest> request{ ImageRequest("0", ImageType::Scene), ImageRequest("1", ImageType::DepthPlanar, true), ImageRequest("2", ImageType::Segmentation) };
        
        // check how the segmentation classes can be obtained here
        // const std::vector<ImageRequest> request{ ImageRequest("0", ImageType::Scene), ImageRequest("0", ImageType::DepthPlanar, true), ImageRequest("0", ImageType::Segmentation, false, false) }; 

        // const std::vector<ImageRequest> request{ ImageRequest("0", ImageType::Scene), ImageRequest("0", ImageType::DepthPlanar, true), ImageRequest("0", ImageType::Segmentation) }; 
/*         const std::vector<ImageRequest> request{ ImageRequest("front_center", ImageType::Scene), ImageRequest("front_center", ImageType::DepthPlanar, true), ImageRequest("front_center", ImageType::Segmentation) }; 
        const std::vector<ImageResponse>& response = client.simGetImages(request, drone_name);
        std::cout << "# of images received: " << response.size() << std::endl;

        // if (!response.size()) {
        if (response.size()) {
            std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;
            std::string path = "/home/sgari/Documents/AirSim/onboard_images";
            // std::getline(std::cin, path);

            for (const ImageResponse& image_info : response) {
                std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
                std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

                if (path != "") {
                    std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
                    std::cout << "file path: " << file_path << std::endl;
                    if (image_info.pixels_as_float) {
                        Utils::writePFMfile(image_info.image_data_float.data(), image_info.width, image_info.height, file_path + ".pfm");
                    }
                    else {
                        std::ofstream file(file_path + ".png", std::ios::binary);
                        file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                        file.close();
                    }
                }
            }
        }
 */        
        std::cout << "Press Enter to arm the drone" << std::endl;
        std::cin.get();
        
        // client.enableApiControl(true);
        client.enableApiControl(true, drone_name); // can select the name of the quadrotor eg "Drone1", "Drone2" etc        
        client.armDisarm(true);

/*         auto barometer_data = client.getBarometerData("", drone_name);
        std::cout << "Barometer data \n"
                  << "barometer_data.time_stamp \t" << barometer_data.time_stamp << std::endl
                  << "barometer_data.altitude \t" << barometer_data.altitude << std::endl
                  << "barometer_data.pressure \t" << barometer_data.pressure << std::endl
                  << "barometer_data.qnh \t" << barometer_data.qnh << std::endl;

        auto imu_data = client.getImuData("",drone_name);
        std::cout << "IMU data \n"
                  << "imu_data.time_stamp \t" << imu_data.time_stamp << std::endl
                  << "imu_data.orientation \t" << imu_data.orientation << std::endl
                  << "imu_data.angular_velocity \t" << imu_data.angular_velocity << std::endl
                  << "imu_data.linear_acceleration \t" << imu_data.linear_acceleration << std::endl;

        auto gps_data = client.getGpsData("",drone_name);
        std::cout << "GPS data \n"
                  << "gps_data.time_stamp \t" << gps_data.time_stamp << std::endl
                  << "gps_data.gnss.time_utc \t" << gps_data.gnss.time_utc << std::endl
                  << "gps_data.gnss.geo_point \t" << gps_data.gnss.geo_point << std::endl
                  << "gps_data.gnss.eph \t" << gps_data.gnss.eph << std::endl
                  << "gps_data.gnss.epv \t" << gps_data.gnss.epv << std::endl
                  << "gps_data.gnss.velocity \t" << gps_data.gnss.velocity << std::endl
                  << "gps_data.gnss.fix_type \t" << gps_data.gnss.fix_type << std::endl;

        auto magnetometer_data = client.getMagnetometerData("",drone_name);
        std::cout << "Magnetometer data \n"
                  << "magnetometer_data.time_stamp \t" << magnetometer_data.time_stamp << std::endl
                  << "magnetometer_data.magnetic_field_body \t" << magnetometer_data.magnetic_field_body << std::endl;
        // << "magnetometer_data.magnetic_field_covariance" << magnetometer_data.magnetic_field_covariance // not implemented in sensor

        // get the ground truth pose of quadrotor
        auto quadrotor_pose = client.simGetGroundTruthKinematics(drone_name);
        std::cout << "Ground truth pose data \n"
                  << "quadrotor_pose.pose.position " << quadrotor_pose.pose.position << std::endl
                << "quadrotor_pose.pose.orientation " << quadrotor_pose.pose.orientation << std::endl
                << "quadrotor_pose.twist.angular " << quadrotor_pose.twist.angular << std::endl;
 */
        std::cout << "Press Enter to takeoff" << std::endl;
        std::cin.get();
        float takeoff_timeout = 5;
        client.takeoffAsync(takeoff_timeout, drone_name)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync(drone_name)->waitOnLastTask();

        std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl;
        std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.enableApiControl(true,drone_name);

        auto position = client.getMultirotorState(drone_name).getPosition();
        float z = position.z(); // current position (NED coordinate system).
        constexpr float speed = 3.0f;
        constexpr float size = 10.0f;
        constexpr float duration = size / speed;
        DrivetrainType drivetrain = DrivetrainType::ForwardOnly;
        YawMode yaw_mode(true, 0);

        std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(speed, 0, z, duration, drivetrain, yaw_mode, drone_name);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, speed, z, duration, drivetrain, yaw_mode, drone_name);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(-speed, 0, z, duration, drivetrain, yaw_mode, drone_name);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, -speed, z, duration, drivetrain, yaw_mode, drone_name);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client.hoverAsync(drone_name)->waitOnLastTask();

        std::cout << "Press Enter to land" << std::endl;
        std::cin.get();
        float land_timeout = 10;
        client.landAsync(land_timeout, drone_name)->waitOnLastTask();

        std::cout << "Press Enter to disarm" << std::endl;
        std::cin.get();
        client.armDisarm(false, drone_name);
    }
    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return 0;
}
