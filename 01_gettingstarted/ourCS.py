#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
Runs experiment either for SITL or physical drone
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import sin, cos, sqrt, atan2, radians, sqrt
import logging
import time
import os
import time
import datetime

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
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=False)
vehicle.wait_ready(timeout=120)


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

    print("Distance (meters):", distance)
    return distance


################################################################################################
# LOGGING FUNCTIONALITY

# Field 0: reserved by logging function
# Field 1: command tag (e.g., GOTO-1-START, continue)
# Field 2: latitude
# Field 3: longitude
# Field 4: altitude
# Field 5: speed in meters  (replace with vehicle.velocity if velocity vector is needed)
# Field 6: timestamp in milliseconds
# Field 7: optional text field
# Field 8: battery voltage
# Field 9: battery level
################################################################################################
logging.basicConfig(filename='NASA1.log', level=logging.DEBUG, filemode='w')
print "Setting up log file"

# EDIT THESE ROWS
experimentNumber = 1
experimentDescription = "Fly from home location to another waypoint"
droneType = "Virtual"
experimenter_initials = "JCH"

# Do not change this one
logging.debug(
    ",METADATA," + "DroneType: " + droneType + " (" + experimenter_initials + "): " + datetime.datetime.now().strftime(
        "%I:%M%p, %B %d %Y") + " Exp #: " + str(experimentNumber) + ": " + experimentDescription)


# function:    log message
# parameters:  Vehicle, command issued, optional text
# returns:     n/a
def logmessage(vehicle, command, extra):
    # cmds = vehicle.commands
    # cmds.download()
    # cmds.wait_ready()
    xv = vehicle.velocity[0]
    xy = vehicle.velocity[1]
    xz = vehicle.velocity[2]
    speed = sqrt((xv * xv) + (xy * xy) + (xz * xz))
    # print "X velocity " + str(xv)
    logging.debug("," + command + "," + str(vehicle.location.global_relative_frame.lat) + "," + str(
        vehicle.location.global_relative_frame.lon) + "," + str(vehicle.location.global_relative_frame.alt) + "," + str(
        speed) + "," + str(time.time()) + "," + extra + "," + str(vehicle.battery.voltage) + "," + str(
        vehicle.battery.level) + ",")


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

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(2)

    print "home: " + str(vehicle.location.global_relative_frame.lat)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(2)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    logmessage(vehicle, "Takeoff-1-START", "")
    while True:
        # print " Altitude: ", vehicle.location.global_relative_frame.alt
        logmessage(vehicle, "continue", "")
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .95:
            print "Reached target altitude"
            break
        time.sleep(1)


arm_and_takeoff(10)
logmessage(vehicle, "Takeoff-1-END", "")


################################################################################################
# Fly to
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed, startTag, endTag):
    print "Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(
        vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon)
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance = get_distance_meters(currentTargetLocation, vehicle.location.global_frame)

    logmessage(vehicle, startTag, str(remainingDistance))
    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_meters(currentTargetLocation, vehicle.location.global_frame)
        print remainingDistance
        logmessage(vehicle, "continue", str(remainingDistance));
        if remainingDistance < 1:
            print "Reached target"
            break;
        time.sleep(1)
    logmessage(vehicle, endTag, "")


################################################################################################
# Experiment Code
################################################################################################

nasa1 = LocationGlobalRelative(41.714816, -86.241941, 20);
nasa2 = LocationGlobalRelative(41.714816, -86.241941, 20);  # Right now they are the same!
points = [nasa1, nasa2]
close = 100000000

# get closest point
for point in points:
    if (close > get_distance_meters(vehicle.location.global_relative_frame, point)):
        close = get_distance_meters(vehicle.location.global_relative_frame, point)
        closePoint = point

# fly to closest point
print "Flying to closest waypoint"
# print out coordinates here..

fly_to(vehicle, closePoint, 10, "START_MISSION", "END_MISSION")

# print "Fly to waypoint 2"
# fly_to(vehicle,LocationGlobalRelative(41.714701,-86.240239,40),5,"GOTO-2-START","GOTO-2-END")

# print "Returning to Launch"
# logmessage(vehicle,"RTL","")
# vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
# print "Close vehicle object"
# vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()