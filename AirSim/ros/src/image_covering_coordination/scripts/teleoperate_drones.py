import airsim
import pygame
import math
import numpy as np
from PIL import Image

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Drones to control
drones = {}

def uint7_to_uint8(value_uint7):
    # Extend 7-bit value to 8 bits by shifting left by 1 bit
    # and then setting the least significant bit to 0
    return (value_uint7 << 1) & 0xFF

def get_segmentation_labels(vehiclename):
    responses = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False)], vehiclename)
    segmentation_img = responses[0].image_data_uint8
    segmentation_img = np.frombuffer(segmentation_img, dtype=np.uint8)
    segmentation_img = segmentation_img.reshape(responses[0].height, responses[0].width, -1)

    # Get unique rows 
    unique_colors = np.unique(segmentation_img.reshape(-1,3), axis=0)

    # Print number of unique colors
    print("Number of unique colors:", len(unique_colors))
    
    return segmentation_img

def takeoff_all():
    print('Taking off all drones to 40 m altitude...')
    for name in drones:
        client.enableApiControl(True, name)
        # client.takeoffAsync(vehicle_name=name)
        client.moveToZAsync(-30, 8, vehicle_name=name) # for image covering
        # client.moveToZAsync(-8, 8, vehicle_name=name) # for active SLAM with kimera-multi
        # state = client.getMultirotorState(vehicle_name=name)
        # s = pprint.pformat(state)
        # print(f"State of {name}: {s}")

def land_all():
    print('Landing all drones...')
    for name in drones:
        client.moveToZAsync(0, 5, vehicle_name=name)
        # client.landAsync(vehicle_name=name)
        # client.enableApiControl(False, name)

# Add drones to the list
# num_drones = 6
drone_start_id = 1
drone_end_id = 10
# for i in range(1, num_drones + 1):
for i in range(drone_start_id, drone_end_id + 1):
    drone_name = f"Drone{i}"
    drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name) 

# Initialize Pygame
pygame.init()

# Set up the display, enter key inputs here
screen = pygame.display.set_mode((100, 100))

down_angle_center = -90 # for roll

# Main loop for controlling drones
current_drone = "Drone1" #default, will change through user input later
success = client.simSetSegmentationObjectID("pavement[\w]", 20, True)  #simGetSegmentationObjectID
segmentation_img = get_segmentation_labels(current_drone)
# pil_img = Image.fromarray(segmentation_img)
# pil_img.show()

takeoff_all()
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            # print("A key has been pressed")

            if event.key == pygame.K_l:
                running = False  # Quit if L is pressed
            
            if event.key == pygame.K_n:
                current_drone = input("Enter your drone name: ")
            
            if event.key == pygame.K_UP:
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
                client.moveByVelocityZAsync(vx=8, vy=0, z=position.z_val, duration=1.5, vehicle_name=current_drone)

            if event.key == pygame.K_DOWN:
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
                client.moveByVelocityZAsync(vx=-8, vy=0, z=position.z_val, duration=1.5, vehicle_name=current_drone)

            if event.key == pygame.K_LEFT:
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
                client.moveByVelocityZAsync(vx=0, vy=-8, z=position.z_val, duration=1.5, vehicle_name=current_drone)

            if event.key == pygame.K_RIGHT:
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
                client.moveByVelocityZAsync(vx=0, vy=8, z=position.z_val, duration=1.5, vehicle_name=current_drone)

            if event.key == pygame.K_q: 
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
                # client.rotateByYawRateAsync(yaw_rate=-3.142, duration=1, vehicle_name=current_drone)
                # client.moveByRollPitchYawrateThrottleAsync(roll=0, pitch=0, yaw_rate=-1.57, throttle=0.5, duration=0.8, vehicle_name =current_drone)
                client.moveByRollPitchYawrateZAsync(roll=0, pitch=0, yaw_rate=0.785, z=position.z_val, duration=0.8, vehicle_name =current_drone)

            if event.key == pygame.K_e:
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
                # client.rotateByYawRateAsync(yaw_rate=3.142, duration=1, vehicle_name=current_drone)
                # client.moveByRollPitchYawrateThrottleAsync(roll=0, pitch=0, yaw_rate=1.57, throttle=0.5, duration=0.8, vehicle_name =current_drone)
                # client.moveByRollPitchYawrateZAsync(roll=0, pitch=0, yaw_rate=-0.785, z=position.z_val, duration=0.8, vehicle_name =current_drone)
                client.rotateToYawAsync(1.571, vehicle_name =current_drone)

            # testing changes to camera angle
            if event.key == pygame.K_h:
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(math.radians(down_angle_center-15), 0, 0)) #radians, order is ROLL, PITCH, YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

            if event.key == pygame.K_k:
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(math.radians(down_angle_center+15), 0, 0)) #radians, order is ROLL, PITCH, YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

            if event.key == pygame.K_u:
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(0, math.radians(down_angle_center+15), 0)) #radians, order is ROLL, PITCH, YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

            if event.key == pygame.K_j:
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(math.radians(0), math.radians(down_angle_center-15), 0)) #radians, order is PITCH ROLL YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

            if event.key == pygame.K_y:
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(0, 0, math.radians(down_angle_center-15))) #radians, order is PITCH ROLL YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

            if event.key == pygame.K_j:
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(0, 0, math.radians(down_angle_center+15))) #radians, order is PITCH ROLL YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

            if event.key == pygame.K_o:
                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                print("cam info before pressing o", camera_info)
                print("changing camera pose")
                position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position

                camera_pose = airsim.Pose(airsim.Vector3r(position.x_val - 0.021, position.y_val, position.z_val + 0.0112), airsim.to_quaternion(math.radians(down_angle_center), 0, 0)) #radians, order is PITCH ROLL YAW
                client.simSetCameraPose("front_center", camera_pose, vehicle_name=current_drone)

                camera_info = client.simGetCameraInfo("front_center", vehicle_name=current_drone)
                print("cam info before pressing o", camera_info)

                # NOTE the angles here are absolute in world frame
                client.moveByRollPitchYawZAsync(0.,0., 0.7854, position.z_val, duration=3, vehicle_name=current_drone)
                # client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(position.x_val, position.y_val, position.z_val), airsim.to_quaternion(0, 0, math.radians(45))))

            if event.key == pygame.K_c:
                segmentation_img = get_segmentation_labels(current_drone)
                pil_img = Image.fromarray(segmentation_img)
                # pil_img.show()



# Clean up
land_all()
airsim.wait_key('Press any key disable/switch off api control')
for i in range(1, 7):
    drone_name = f"Drone{i}"    
    client.enableApiControl(False, drone_name) # exits Unity cleanly
pygame.quit()
