import dronekit_sitl
import dronekit
import json
import os
import time
import flightmanager
from ground_control_station import SimpleGCS

def load_json(path2file):
    d = None
    try:
        with open(path2file) as f:
            d = json.load(f)
    except Exception as e:
        exit('Invalid path or malformed json file! ({})'.format(e))

    return d

config = load_json("nd.json") #"homework_testcases/test1.json")

# A list of drones. (dronekit.Vehicle)
vehicles = []

# A list of lists of lists (i.e., [ [ [lat0, lon0, alt0], ...] ...]
# These are the waypoints each drone must go to!
routes = []

ARDUPATH = "/home/uav/git/ardupilot"
gcs = SimpleGCS(ARDUPATH)
gcs.connect()


# Start up all the drones specified in the json configuration file
for i, v_config in enumerate(config):

    home = v_config['start']
    print("Activating Virtual Drone...." + str(home))
    name = "UAV-" + str(i)

    #Registers and activates drone
    print("Home")
    print(home)
    vehicle = gcs.registerDrone(home,name)

    vehicles.append(vehicle)
    routes.append(v_config['waypoints'])
    vehicle_id = str("UAV-" + str(i))

target_altitude = 10
for vehicle in vehicles:
    flightmanager.arm_and_takeoff(vehicle,target_altitude)
    #print(vehicle.location.global_relative_frame)

# Wait for them all to take-off
takeoffcount = len(vehicles)
print("Waiting for all UAVs to takeoff")
while takeoffcount > 0:
    takeoffcount = 0
    for vehicle in vehicles:
        if vehicle.location.global_relative_frame.alt < target_altitude *.9:
            print ("Not yet:" + str(vehicle.location.global_relative_frame.alt))
            takeoffcount = takeoffcount+1
    time.sleep(1)
print ("Takoff phase completed")

# Extract the first waypoint (Need something more sophisticated)
for i,vehicle in enumerate(vehicles):
    vehiclewaypoint = (routes[i])[0]
    print(vehiclewaypoint)
    flightmanager.start_flight(vehicle,vehiclewaypoint,10)


    









