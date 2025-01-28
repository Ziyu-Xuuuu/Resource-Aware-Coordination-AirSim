import json
from collections import OrderedDict

def generate_settings(num_drones):
    vehicles = OrderedDict()
    for i in range(1, num_drones + 1):
        vehicles["Drone{}".format(i)] = OrderedDict({
            "VehicleType": "SimpleFlight",
            "DefaultVehicleState": "Armed",
            "EnableCollisionPassthrough": True,
            "EnableCollisions": False,
            "AllowAPIAlways": True,
            "Cameras": OrderedDict({
                "front_center": OrderedDict({
                    "CaptureSettings": [
                        OrderedDict({
                            "ImageType": 5,
                            "FOV_Degrees": 90
                        })
                    ],
                    "Gimbal": OrderedDict({
                        "Stabilization": 1,
                        "Pitch": 1,
                        "Roll": 1,
                        "Yaw": 1
                    }),
                    "SegmentationSettings": OrderedDict({
                        "InitMethod": "None",
                        "MeshNamingMethod": "StaticMeshName",
                        "OverrideExisting": True
                    })
                })
            })
        })

    settings = OrderedDict([
        ("SeeDocsAt", "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md"),
        ("SettingsVersion", 1.2),
        ("SimMode", "Multirotor"),
        ("ClockSpeed", 1),
        ("ViewMode", "NoDisplay"),
        ("SegmentationSettings", OrderedDict([
            ("InitMethod", "None"),
            ("MeshNamingMethod", "StaticMeshName"),
            ("OverrideExisting", True)
        ])),
        ("Vehicles", vehicles)
    ])

    with open('settings_autogen.json', 'w') as outfile:
        json.dump(settings, outfile, indent=4)

# Change the number of drones as needed
num_drones = 45
generate_settings(num_drones)
