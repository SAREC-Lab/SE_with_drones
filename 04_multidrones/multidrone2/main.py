import dronekit_sitl
import dronekit
import json
import os
from copter import UAV_Copter

def load_json(path2file):
    d = None
    try:
        with open(path2file) as f:
            d = json.load(f)
    except Exception as e:
        exit('Invalid path or malformed json file! ({})'.format(e))

    return d

config = load_json("homework_testcases/test5.json")
print(config)

# A list of sitl instances.
sitls = []

# A list of drones. (dronekit.Vehicle)
vehicles = []

# A list of lists of lists (i.e., [ [ [lat0, lon0, alt0], ...] ...]
# These are the waypoints each drone must go to!
routes = []

# This is really temporary for this assignment so we can track IDs for each drone
# Next week we'll integrate with Dronology and replace it.
copters = []

# Start up all the drones specified in the json configuration file
for i, v_config in enumerate(config):
    copter = UAV_Copter()
    home = v_config['start']
    vehicle, sitl = copter.connect_vehicle(i, home)
    sitls.append(sitl)
    vehicles.append(vehicle)
    routes.append(v_config['waypoints'])
    vehicle_id = str("UAV-" + str(i))
    copter.setvalues(sitl, vehicle, v_config['waypoints'], vehicle_id)
    copters.append(copter)

for copter in copters:
    print("\nVehicle ID: " + copter.getid())
    copter.print_drone_state()
    print(copter.waypoints)
    print("\n")
    









