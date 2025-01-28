import airsim
import pprint
import math

# constants
CAM_NAME = "front_center"
IS_EXTERNAL_CAM = True
DRONE_NAME = "Drone1"

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Drones to control
drones = {}

def takeoff_all():
    print('Taking off all drones to 40 m altitude...')
    for name in drones:
        client.enableApiControl(True, name)
        # client.takeoffAsync(vehicle_name=name)
        client.moveToZAsync(-40, 8, vehicle_name=name) # for image covering
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

# def switch_drone(name):
#     if name in drones:
#         print(f"Switching control to {name}...")
#         # Handle the switch to control the specified drone

# Add drones to the list
num_drones = 6
for i in range(1, num_drones + 1):
    drone_name = f"Drone{i}"
    drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name) 

# Main loop for controlling drones
current_drone = "Drone1" #default, will change through user input later
takeoff_all()
print("changing camera pose by rotating quadrotor")
position = client.getMultirotorState(vehicle_name=current_drone).kinematics_estimated.position
# client.moveByRollPitchYawrateZAsync(roll=0, pitch=0, yaw_rate=0.785, z=position.z_val, duration=0.8, vehicle_name =current_drone)

# NEED TO FIX CAMERA ANGLE SETTING
cam_info = client.simGetCameraInfo(CAM_NAME, vehicle_name = DRONE_NAME) # crashes here
# camera_info = client.simGetCameraInfo(str("front_center"))
camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(25), 0, 0)) #radians; sets camera to absolute pose, not relative
client.simSetCameraPose(CAM_NAME, camera_pose, vehicle_name = DRONE_NAME)

# Clean up
land_all()
airsim.wait_key('Press any key disable/switch off api control')
for i in range(1, 7):
    drone_name = f"Drone{i}"    
    client.enableApiControl(False, drone_name) # exits Unity cleanly
pygame.quit()
