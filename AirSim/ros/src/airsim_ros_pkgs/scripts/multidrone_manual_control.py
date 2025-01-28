import airsim
import cv2
import numpy as np
import os
import pprint
# import setup_path 
import tempfile
import math

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name="Drone1")
f2 = client.takeoffAsync(vehicle_name="Drone2")
f1.join()
f2.join()

# setting camera gimbal pose api call works correctly.
# airsim.wait_key('Press any key to set camera gimbal to 15-degree pitch')
# camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(15), 0, 0)) #radians
# client.simSetCameraPose("front_center", camera_pose, vehicle_name = "Drone1")

state1 = client.getMultirotorState(vehicle_name="Drone1")
s = pprint.pformat(state1)
print("state: %s" % s)
state2 = client.getMultirotorState(vehicle_name="Drone2")
s = pprint.pformat(state2)
print("state: %s" % s)

airsim.wait_key('Press any key to move vehicles')
f1 = client.moveToPositionAsync(-5, 5, -10, 5, vehicle_name="Drone1")
f2 = client.moveToPositionAsync(5, -5, -10, 5, vehicle_name="Drone2")
f1.join()
f2.join()

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")


