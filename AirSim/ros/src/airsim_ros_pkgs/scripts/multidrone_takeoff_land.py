import airsim
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable API control and takeoff for drones named "Drone1" through "Drone6"
airsim.wait_key('Press any key to takeoff all drones')
for i in range(1, 7):
    drone_name = f"Drone{i}"
    client.enableApiControl(True, drone_name)

    # .join() is blocking; will wait until this drone has taken off and reached state
    # client.takeoffAsync(vehicle_name=drone_name).join()
    # client.moveToZAsync(-15, 5, vehicle_name=drone_name).join()

    # client.takeoffAsync(vehicle_name=drone_name)
    client.moveToZAsync(-15, 5, vehicle_name=drone_name)
    state = client.getMultirotorState(vehicle_name=drone_name)
    s = pprint.pformat(state)
    print(f"State of {drone_name}: {s}")

airsim.wait_key('Press any key to land all drones')
# Disable API control for all drones
# for i in range(1, 7):
#     drone_name = f"Drone{i}"
#     client.enableApiControl(False, drone_name)


for i in range(1, 7):
    drone_name = f"Drone{i}"
    # client.landAsync(vehicle_name=drone_name).join()
    
    # decrease altitude quickly before landing
    client.moveToZAsync(0, 5, vehicle_name=drone_name)
    # client.landAsync(vehicle_name=drone_name)
    # client.enableApiControl(False, drone_name)

airsim.wait_key('Press any key disable/switch off api control')
for i in range(1, 7):
    drone_name = f"Drone{i}"    
    client.enableApiControl(False, drone_name) # exits Unity cleanly

