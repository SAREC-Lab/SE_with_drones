import dronekit_sitl
import dronekit
import json
import argparse
import os
import threading
import time
import signal
import util
import logging
import simplegcs
from dronekit import connect, VehicleMode, LocationGlobalRelative

# make sure you change this so that it's correct for your system
ARDUPATH = os.path.join('/', 'win', 'Work', 'git', 'ardupilot')


def main(ardupath=None):
    if ardupath is not None:
        global ARDUPATH
        ARDUPATH = ardupath


    connect()
    print("Registering Drones...")
    vehicle1=addDrone([41.715446209367,-86.242847096132,0],"Frank")

    print("Taking off...")
    vehicle1.simple_takeoff(10)
    time.sleep(10)
    print("Going somewhere...")

    gotoWaypoint(vehicle1,41.515446209367,-86.342847096132, 40,3)

    # point1 = LocationGlobalRelative(41.515446209367,-86.342847096132, 40)
    # vehicle1.simple_goto(point1)

    # point2 = LocationGlobalRelative(41.515446209367, -86.342847096132, 40)
    # vehicle2.simple_goto(point2)

def connect():
    print("Connecting to Dronology...")
    print("SITL path: "+ARDUPATH)
    global GCS
    GCS = simplegcs.SimpleGCS(ARDUPATH,"simplegcs")
    GCS.connect()
    time.sleep(10)

def gotoWaypoint(vehicle,xcoord,ycoord,zcoord, airspeed=10):
    vehicle.airspeed = airspeed
    point = LocationGlobalRelative(xcoord,ycoord,zcoord)
    vehicle.simple_goto(point)

def addDrone(home, name=None):
    vehicle = GCS.registerDrone(home,name)
    time.sleep(5)
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(3)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    return vehicle

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    #ap.add_argument('path_to_config', type=str, help='the path to the drone configuration file.')
    ap.add_argument('--ardupath', type=str, default=ARDUPATH)
    args = ap.parse_args()
    main(ardupath=args.ardupath)
