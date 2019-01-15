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
import re
import numpy as np
import matplotlib.pyplot as plt

lat_array = []
lon_array = []

class Location:
    def __init__(self,lat=0.0,lon=0.0):
        """ Create a new point at the origin """
        self.lat = lat
        self.lon = lon

class Nedvalues:
    def __init__(self,north=0.0,east=0.0,down=0.0):
        """ Create a new point at the origin """
        self.north = north
        self.east = east
        self.down = down


def scatter_plot(x,y,x2,y2,x_label,y_label,title):
    plt.plot(x,y)
    plt.plot(x2,y2)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.show()

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

    #print("Distance (meters):", distance)
    return distance

def send_reverseThrust(ned,vehicle): #current NED
    print("Reverse thrust")
    if (vehicle.groundspeed)> 1:
        ned.north = ned.north *-1
        ned.east = ned.east*-1
        ned.down = ned.down*-1
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame BODY_OFFSET_NED
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        ned.north,ned.east,ned.down,  # x, y, z velocity in m/s Applies reverse thrust on X wrt drone body (not sure if it works if yaw isn't working yet!
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    time.sleep(1)

    while vehicle.groundspeed > 1:
        vehicle.send_mavlink(msg)
        print("Slowing down? {0}".format(str(vehicle.groundspeed)))
        time.sleep(1)
    return ned

def send_reverseThrust2(vehicle): #current NED
    print("Reverse thrust")

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame BODY_OFFSET_NED
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        -1, 0, 0,  # x, y, z velocity in m/s Applies reverse thrust on X wrt drone body (not sure if it works if yaw isn't working yet!
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    for x in range(0,50):
        vehicle.send_mavlink(msg)
    time.sleep(1)

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

    for x in range (0,duration):
        vehicle.send_mavlink(msg)
    # send command to vehicle on 1 Hz cycle
    #remainingdistance = get_distance_meters()
    #for x in range(0,duration):
    #    vehicle.send_mavlink(msg)
    #    print(str(vehicle.location.global_relative_frame.lat) + "," + str(vehicle.location.global_relative_frame.lon) + "," + str(vehicle.location.global_relative_frame.alt))

    time.sleep(1)


################################################################################################
# Fly to
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed):
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

#Have not dealt with meridian and equator!!
def setNED(currentLocation,targetLocation):
    ned = Nedvalues()
    ned.north = (targetLocation.lat - currentLocation.lat)
    ned.east = (targetLocation.lon - currentLocation.lon)
    #Needs scaling up!
    scaling_factor = 1
    if (abs(ned.north) < abs(ned.east)):
        if ned.north != 0:
            scaling_factor = 1/abs(ned.north)
        elif ned.east != 0:
            scaling_factor = 1/abs(ned.east)
    else:
        if ned.east != 0:
            scaling_factor = 1/abs(ned.east)
        elif ned.north != 0:
            scaling_factor = 1/abs(ned.north)

    ned.north = (ned.north*scaling_factor)/10
    ned.east = (ned.east*scaling_factor)/10
    return ned

def flyByNed(targetLocation):
    currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    distance_to_target = get_distance_meters(currentLocation, targetLocation)
    print ("Distance to target for new stretch: " + str(distance_to_target))
    closestDistance = distance_to_target
    ned = setNED(currentLocation, targetLocation)
    while distance_to_target > 1:
        print("Ground speed: " + str(vehicle.groundspeed))
        if(distance_to_target < 10):
            vehicle.groundspeed=distance_to_target

        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        distance_to_target = get_distance_meters(currentLocation, targetLocation)

        closestDistance = distance_to_target
        print ("Current Lat: " + str(currentLocation.lat) + " Current lon: " + str(currentLocation.lon))
        print ("Target  Lat: " + str(targetLocation.lat) + " Target  lon: " + str(targetLocation.lon))
        print (" Dist: " + str(distance_to_target) + "  NED: " + str(ned.north) + " " + str(
            ned.east) + " " + str(ned.down))
        currentLocation = Location(vehicle.location.global_relative_frame.lat,
                                   vehicle.location.global_relative_frame.lon)
        ned = setNED(currentLocation, targetLocation)
        send_ned_velocity(ned.north, ned.east, ned.down, 1)  # changed from 10 to 1!
        lat_array.append([])
        lon_array.append([])
        lat_array[-1].append(currentLocation.lat)
        lon_array[-1].append(currentLocation.lon)

    print ("Breaking at: " + str(distance_to_target) + " Velocity: " + str(vehicle.groundspeed) + "\n")
    #if vehicle.velocity > 1:
    #    print("Trying to brake: " + str(vehicle.groundspeed))
    #   ned = send_reverseThrust(ned,vehicle)

arm_and_takeoff(10)
# Current location of vehicle.
center = Location(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon) #83.456453, 43.987633)
radius = .0001
currentLocation = center
point1 = point_on_circle(radius,0, center.lat, center.lon)
location1 = LocationGlobalRelative(point1.lat, point1.lon, 10)
point2 = point_on_circle(radius,30, center.lat, center.lon)
location2 = LocationGlobalRelative(point2.lat, point2.lon, 10)
flyByNed(location1)
flyByNed(location2)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

###############
#Create the perfect plot
print ("Perfect plot")
lat2_array = []
lon2_array = []
lat2_array.append([])
lon2_array.append([])
lat2_array[-1].append(center.lat)
lon2_array[-1].append(center.lon)

lat2_array.append([])
lon2_array.append([])
lat2_array[-1].append(location1.lat)
lon2_array[-1].append(location1.lon)

lat2_array.append([])
lon2_array.append([])
lat2_array[-1].append(location2.lat)
lon2_array[-1].append(location2.lon)

scatter_plot(lon_array,lat_array,lon2_array,lat2_array,"Longitude","Latitude","")
