import airsim
import pygame
import math

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

# Add drones to the list
# num_drones = 6
drone_start_id = 3
drone_end_id = 4
# for i in range(1, num_drones + 1):
for i in range(drone_start_id, drone_end_id + 1):
    drone_name = f"Drone{i}"
    drones[drone_name] = client.getMultirotorState(vehicle_name=drone_name) 

# Initialize Pygame
pygame.init()

# Set up the display, enter key inputs here
screen = pygame.display.set_mode((100, 100))

# Main loop for controlling drones
current_drone = "Drone3" #default, will change through user input later
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
                client.moveByRollPitchYawrateZAsync(roll=0, pitch=0, yaw_rate=-0.785, z=position.z_val, duration=0.8, vehicle_name =current_drone)
            
            # testing changes to camera angle
            if event.key == pygame.K_c:
                print("changing camera pose")
                camera_info = client.simGetCameraInfo("front_center")
                camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(25), 0, 0)) #radians
                client.simSetCameraPose("front_center", camera_pose)


# Clean up
land_all()
airsim.wait_key('Press any key disable/switch off api control')
for i in range(1, 7):
    drone_name = f"Drone{i}"    
    client.enableApiControl(False, drone_name) # exits Unity cleanly
pygame.quit()
