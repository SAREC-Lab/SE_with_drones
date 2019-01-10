#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
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
#Set up option parsing to get connection string
################################################################################################
import argparse  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


################################################################################################
#Start SITL if no connection string specified
################################################################################################
if not connection_string:
    import dronekit_sitl
    #sitl = dronekit_sitl.start_default()
    #connection_string = sitl.connection_string()
    ardupath ="/home/uav/git/ardupilot"
    home = "41.7144367,-86.2417136,221,0"
    sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
    sitl_args = ['-I{}'.format(0), '--home', home, '--model', '+', '--defaults', sitl_defaults]
    sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
    sitl.launch(sitl_args, await_ready=True)

    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + 0 * 10)
    connection_string = ':'.join([tcp, ip, port])

    #vehicle = dronekit.connect(conn_string)
    #vehicle.wait_ready(timeout=120)

################################################################################################
# Connect to the Vehicle
################################################################################################
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)



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
        time.sleep(1)

    print "home: " + str(vehicle.location.global_relative_frame.lat)
    
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude


    while True:
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95: 
            print "Reached target altitude"
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

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = (R * c) * 1000

    print("Distance (meters):", distance)
    return distance




################################################################################################
# Fly to
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed, startTag, endTag):
    print "Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon)
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame)

    while vehicle.mode.name=="GUIDED":
        remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame)
        print remainingDistance
        if remainingDistance< 1:
            print "Reached target"
            break;
        time.sleep(1)


if __name__ == '__main__':
    arm_and_takeoff(10)
    print "Fly to waypoint 1"
    fly_to(vehicle, LocationGlobalRelative(41.71500, -86.24230, 20), 10, "GOTO-1-START", "GOTO-1-END")
    print
    "Returning to Launch"
    vehicle.mode = VehicleMode("RTL")
    # Close vehicle object before exiting script
    print
    "Close vehicle object"
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()
