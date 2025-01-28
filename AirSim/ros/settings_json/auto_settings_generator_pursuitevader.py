import json

num_pursuers = 10
num_evaders = 0

vehicles = {}
for i in range(1, num_pursuers+1):
    vehicles["Drone{}".format(i)] = {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Armed",
        "EnableCollisionPassthrough": "true",
        "EnableCollisions": "false",
        "AllowAPIAlways": "true",
        "Cameras": {
            "front_center": {
                "CaptureSettings": [
                    {
                        "ImageType": 0,
                        "FOV_Degrees": 90 
                    }
                ]
            }
        }
    }

for i in range(num_pursuers+1, num_pursuers+num_evaders+1):
    vehicles["Drone{}".format(i)] = {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Armed"
    }

settings = {
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor", 
  "ClockSpeed": 1,
  "ViewMode": "NoDisplay",
  "Vehicles": vehicles
}

with open('settings_autogen.json', 'w') as outfile:
    json.dump(settings, outfile, indent=4)
