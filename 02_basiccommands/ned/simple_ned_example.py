#!/ usr / bin / env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
Added code for flying using NED Velocity Vectors - Jane Cleland-Huang 1/15
"""
import math
import os
import time
from math import sin, cos, atan2, radians, sqrt

from dronekit import connect, VehicleMode, LocationGlobalRelative

from ned.flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned.ned_utilities import ned_controller

################################################################################################
# Set up option parsing to get connection string
################################################################################################
import argparse

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto or NEDs.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

################################################################################################
# Start SITL if no connection string specified
################################################################################################
if not connection_string:
    import dronekit_sitl

    # sitl = dronekit_sitl.start_default()
    # connection_string = sitl.connection_string()
    ardupath = "/home/uav/git/ardupilot"
    home = "41.7144367,-86.2417136,221,0"
    sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
    sitl_args = ['-I{}'.format(0), '--home', home, '--model', '+', '--defaults', sitl_defaults]
    sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
    sitl.launch(sitl_args, await_ready=True)

    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + 0 * 10)
    connection_string = ':'.join([tcp, ip, port])

    # vehicle = dronekit.connect(conn_string)
    # vehicle.wait_ready(timeout=120)

################################################################################################
# Connect to the Vehicle
################################################################################################
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print(vehicle.location.global_relative_frame.lon)


################################################################################################
# ARM and TAKEOFF
################################################################################################

# function:   	arm and takeoff
# parameters: 	target altitude (e.g., 10, 20)
# returns:	n/a

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("home: " + str(vehicle.location.global_relative_frame.lat))

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .95:
            print("Reached target altitude")
            break
        time.sleep(1)


################################################################################################
# function:    Get distance in meters
# parameters:  Two global relative locations
# returns:     Distance in meters
################################################################################################
def get_distance_meters(locationA, locationB):
    # approximate radius of earth in km
    R = 6373.0

    lat1 = radians(locationA.lat)
    lon1 = radians(locationA.lon)
    lat2 = radians(locationB.lat)
    lon2 = radians(locationB.lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = (R * c) * 1000

    # print("Distance (meters):", distance)
    return distance


################################################################################################
# Fly to
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed):
    print("Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(
        vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon))
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance = get_distance_meters(currentTargetLocation, vehicle.location.global_frame)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_meters(currentTargetLocation, vehicle.location.global_frame)
        print(remainingDistance)
        if remainingDistance < 1:
            print("Reached target")
            break
        time.sleep(1)

def point_on_circle(radius, angle_indegrees, latitude, longitude):
    # Convert from degrees to radians
    lon = (radius * math.cos(angle_indegrees * math.pi / 180)) + longitude
    # print("lat: %f", lon)
    lat = (radius * math.sin(angle_indegrees * math.pi / 180)) + latitude
    # print("lon: %f", lat)
    # logfile.write(str(lat)+","+str(lon)+"\n")

    return Location(lat, lon)


############################################################################################
# Main functionality
############################################################################################
arm_and_takeoff(10)

log1 = CoordinateLogger()
log2 = CoordinateLogger()
currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
log2.add_data(currentLocation.lon,currentLocation.lat)
log1.add_data(currentLocation.lon,currentLocation.lat)
#print("Current position: {1},{2} ".format(currentLocation.lat, currentLocation.lon))
print("Current position: " + str(currentLocation.lat) + "," + str(currentLocation.lon))

# Get current location of vehicle and establish a conceptual circle around it for flying
center = Location(vehicle.location.global_relative_frame.lat,
                  vehicle.location.global_relative_frame.lon)  # 83.456453, 43.987633)

startingPosition = vehicle.location.global_relative_frame
# Establish an instance of CoordinateLogger

nedcontroller = ned_controller()
nextTarget = LocationGlobalRelative(currentLocation.lat, -86.2415000, 10)

print("Target position: " + str(nextTarget.lat) + ", " + str(nextTarget.lon))

# Add target location to both logs.
# Note this is the final target
log2.add_data(nextTarget.lon,nextTarget.lat)
log1.add_data(nextTarget.lon,nextTarget.lat)


currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
###########################################################################
# Create the target coordinates
###########################################################################
print ("Center: " + str(center.lat) + " " + str(center.lon))

currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
distance_to_target = get_distance_meters(currentLocation, nextTarget)
closestDistance=distance_to_target
ned = nedcontroller.setNed(currentLocation, nextTarget)



print("Distance: " + str(distance_to_target))
while get_distance_meters(currentLocation,nextTarget) > 1:
    ned = nedcontroller.setNed(currentLocation, nextTarget)
    nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, vehicle)
    #Log data
    log1.add_data(vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat)
    currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    distance_to_target = get_distance_meters(currentLocation, nextTarget)
    print ("Current Pos: ({0},{1}) Target Pos: {2},{3} NED: {4},{5} Distance: {6}".format(
        str(currentLocation.lat), str(currentLocation.lon), str(nextTarget.lat), str(nextTarget.lon), str(ned.north),
        str(ned.east), str(
            distance_to_target)))

print("Returning to Launch")




vehicle.mode = VehicleMode("RTL")


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()



###########################################################################
# Plot graph
###########################################################################
plotter = GraphPlotter(log1.lat_array,log1.lon_array,log2.lat_array,log2.lon_array,"Longitude","Latitude","NED vs. target Coordinates")
plotter.scatter_plot()
