#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative,Command
import time
from math import sin, cos, sqrt, atan2, radians, sqrt
import logging
import time
import os
import time
import datetime
from pymavlink import mavutil
from ned.flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned.ned_utilities import ned_controller


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
print vehicle.location.global_relative_frame.lon


################################################################################################
# CLEAR COMMANDS
################################################################################################
def clearAllCommands():
    cmds = vehicle.commands
    cmds.clear()
    cmds.wait_ready()
    cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                   0, 0, 0, 0, vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt)
    cmds.add(cmd1)
    cmds.upload()


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

    #print("Distance (meters):" + str(distance) + " Target: " + str(locationB.lat) + ", " + str(locationB.lon))
    return distance


log1 = CoordinateLogger()
log2 = CoordinateLogger()


################################################################################################
# Fly to
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed, startTag, endTag,abortdistance):
    print "Flying from: " + str(vehicle.location.global_relative_frame.lat) + "," + str(vehicle.location.global_relative_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon)
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame)

    while vehicle.mode.name=="GUIDED":
        log2.add_data(vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat)
        remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_relative_frame)
        print remainingDistance
        if remainingDistance< abortdistance:
            #clearAllCommands()
            print "Aborted target waypoint"
            break;
        time.sleep(1)

if __name__ == '__main__':
    arm_and_takeoff(10)

    startingPosition = vehicle.location.global_relative_frame
    log1.add_data(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)
    log1.add_data(-86.24230,41.71500)
    log1.add_data(-86.24340,41.71412)


    ##############
    # Starting NED
    ##############
    log2 = CoordinateLogger()


    ########################################################
    # 1st NED flight
    ########################################################
    print("Starting NED flight")
    nedcontroller = ned_controller()
    target = LocationGlobalRelative(41.71500, -86.24230, 20)
    ned = nedcontroller.setNed(vehicle.location.global_relative_frame, target)
    #closestDistance = 1000000000
    currentLocation = Location(vehicle.location.global_relative_frame.lat,
                               vehicle.location.global_relative_frame.lon)
    distance_to_target = get_distance_meters(currentLocation, target)
    while distance_to_target > 1:

        print("UAV speed: " + str(vehicle.groundspeed))
        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        distance_to_target = get_distance_meters(currentLocation, target)

        if distance_to_target <=1:  # Prevent unwanted fly-by
            break

        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        ned = nedcontroller.setNed(currentLocation, target)
        nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, vehicle)
        print("Distance: " + str(distance_to_target) + " Target: " + str(target.lat) + ", " + str(
            target.lon) + " From: " + str(currentLocation.lat) + ", " + str(currentLocation.lon) + " NED: " + str(ned.east) + "," + str(ned.north))

        # Log data
        log2.add_data(currentLocation.lon, currentLocation.lat)



    ###########################################################
    # Flight # 2 with new target
    ###########################################################
    print("\n Aborting and switching to new target")
    breakpoint = vehicle.location.global_relative_frame

    target = LocationGlobalRelative(41.71412, -86.24340, 20)
    ned = nedcontroller.setNed(vehicle.location.global_relative_frame, target)

    currentLocation = Location(vehicle.location.global_relative_frame.lat,
                               vehicle.location.global_relative_frame.lon)
    distance_to_target = get_distance_meters(currentLocation, target)
    while distance_to_target > 1:

        print("UAV speed: " + str(vehicle.groundspeed))
        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        distance_to_target = get_distance_meters(currentLocation, target)

        if distance_to_target <= 1:  # Prevent unwanted fly-by
            break


        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        ned = nedcontroller.setNed(currentLocation, target)
        nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, vehicle)
        print("Distance: " + str(distance_to_target) + " Target: " + str(target.lat) + ", " + str(
            target.lon) + " From: " + str(currentLocation.lat) + ", " + str(currentLocation.lon) + " NED: " + str(ned.east) + "," + str(ned.north))

        # Log data
        log2.add_data(currentLocation.lon, currentLocation.lat)

    nedcontroller.send_ned_stop(vehicle)

    vehicle.groundspeed=20
    print 'Returning to Launch'
    vehicle.mode = VehicleMode("RTL")
    # Close vehicle object before exiting script
    print
    "Close vehicle object"
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

    ###########################################################################
    # Plot graph
    ###########################################################################
    plotter = GraphPlotter(log1.lat_array, log1.lon_array, log2.lat_array, log2.lon_array, "Longitude", "Latitude",
                           "NED vs. target Coordinates")
    plotter.add_marker(breakpoint.lon, breakpoint.lat)
    plotter.scatter_plot()
