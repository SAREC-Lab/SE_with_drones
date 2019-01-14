#!/ usr / bin / env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
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

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
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
            break;
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

# Get current location of vehicle and establish a conceptual circle around it for flying
center = Location(vehicle.location.global_relative_frame.lat,
                  vehicle.location.global_relative_frame.lon)  # 83.456453, 43.987633)
radius = .0001
angle = 0  #Starting angle is directly east
currentLocation = center
startingPos = point_on_circle(radius, angle, center.lat, center.lon)  # First position on circle perimeter
firstTargetPosition = LocationGlobalRelative(startingPos.lat, startingPos.lon, 10)
fly_to(vehicle, firstTargetPosition, 10)

# Establish a starting angle to compute next position on circle
angle = 3

# Establish an instance of CoordinateLogger
log1 = CoordinateLogger()

nedcontroller = ned_controller()

while angle <= 360:
    print("\nNew target" + str(angle))
    nextTarget = (point_on_circle(radius, angle, center.lat, center.lon))

    currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    distance_to_target = get_distance_meters(currentLocation, nextTarget)
    closestDistance=distance_to_target
    ned = nedcontroller.setNED(currentLocation,nextTarget)

    while distance_to_target > 1:
        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        distance_to_target = get_distance_meters(currentLocation, nextTarget)
        if distance_to_target > closestDistance:
            break
        else:
            closestDistance = distance_to_target
        print ('Current Pos: (' + str(currentLocation.lat) + "," + str(currentLocation.lon) +
               ') Target Pos: ' + str(nextTarget.lat) + ' Target  lon: ' + str(nextTarget.lon) + ' Distance: ' + str(distance_to_target));
        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        ned = nedcontroller.setNED(currentLocation, nextTarget)
        nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, vehicle)  # changed from 10 to 1!
        log1.add_data(currentLocation.lat,currentLocation.lon)

    angle = angle + 45

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

###########################################################################
# Create the target coordinates
###########################################################################
print ("Perfect plot")
log2 = CoordinateLogger()
angle = 0
while angle <= 360:
    point = (point_on_circle(radius, angle, center.lat, center.lon))
    log2.add_data(point.lat,point.lon)
    angle = angle + 1

###########################################################################
# Plot graph
###########################################################################
#plotter = GraphPlotter("Longitude","Latitude","NED vs target Coordinates")
plotter = GraphPlotter(log1.lat_array,log1.lon_array,log2.lat_array,log2.lon_array,"Longitude","Latitude","NED vs. target Coordinates")
#plotter.addLog(log1)
#plotter.addLog(log2)
plotter.scatter_plot()
