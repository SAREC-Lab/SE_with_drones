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
from pymavlink import mavutil
import math

lat_array = []
lon_array = []

class Location:
    def __init__(self,lat=0.0,lon=0.0):
        """ Create a new point at the origin """
        self.lat = lat
        self.lon = lon

def point_on_circle(radius, angle_indegrees, latitude, longitude):
    #Convert from degrees to radians
    lon = (radius*math.cos(angle_indegrees * math.pi / 180)) + longitude
    #print("lat: %f", lon)
    lat = (radius*math.sin(angle_indegrees * math.pi / 180)) + latitude
    #print("lon: %f", lat)
    #logfile.write(str(lat)+","+str(lon)+"\n")

    return Location(lat,lon)


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
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude


    while True:
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95: 
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

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = (R * c) * 1000

    print("Distance (meters):", distance)
    return distance

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        print(str(vehicle.location.global_relative_frame.lat) + "," + str(vehicle.location.global_relative_frame.lon) + "," + str(vehicle.location.global_relative_frame.alt))

        time.sleep(1)


################################################################################################
# Fly to
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed, startTag, endTag):
    print("Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon))
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame)

    while vehicle.mode.name=="GUIDED":
        remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame)
        print(remainingDistance)
        if remainingDistance< 1:
            print("Reached target")
            break;
        time.sleep(1)

def ned_values(self,north,east,down):
    self.d = dict();
    self.d['NORTH'] = north
    self.d['EAST'] = east
    self.d['DOWN'] = down

def setNED(currentLocation,targetLocation):
    ned = ned_values()
    ned.north = currentLocation.lat - targetLocation.lat
    ned.east = currentLocation.lon - targetLocation.lon
    return ned

if __name__ == '__main__':
    arm_and_takeoff(10)

    # Current location of vehicle.
    center = Location(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon) #83.456453, 43.987633)
    radius = .00001
    angle = 0
    while angle <= 360:
        point = (point_on_circle(radius, angle, center.lat, center.lon))
        print("Next target:", angle, point.lat, point.lon)
        angle = angle + 1


        # JCH: To do.  For each target lat and lon -- fly to the velocity vector.



    #print("Fly to waypoint 1")
    #fly_to(vehicle, LocationGlobalRelative(41.71500, -86.24230, 20), 10, "GOTO-1-START", "GOTO-1-END")

    # Set up velocity mappings
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend

    print("flying a NED")
    SOUTH = -2
    UP = -0.5  # NOTE: up is negative!
    # Fly south and up.
    send_ned_velocity(SOUTH, 0, UP, 5)


    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()
