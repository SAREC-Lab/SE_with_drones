import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import UtilFunctions as util

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Waiting for vehicle to initialise!")
    while not vehicle.is_armable:
        sys.stdout.write('.')
        time.sleep(2)

    print("home: " + str(vehicle.location.global_relative_frame.lat))

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    print("Waiting for arming...")
    while not vehicle.armed:
        sys.stdout.write('.')
        time.sleep(2)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        # Break and return from function just below target altitude.
        print("Target altitude: " + str(vehicle.location.global_relative_frame.alt))
        time.sleep(1)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .90:
            print("Reached target altitude")
            break
    time.sleep(1)

def start_flight(vehicle, waypoint, groundspeed):
    targetLocation = LocationGlobalRelative(waypoint[0],waypoint[1],waypoint[2])
    print("Start flying")
    if (targetLocation.lat < 41.713799 or targetLocation.lat >41.715593):
        print("ERROR when assigning location! - Latitude outside range!")
        return
    if (targetLocation.lon < -86.244579 or targetLocation.lon > -86.236527):
        print("ERROR when assigning location! - Longitude outside range!")
        return

    print("Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(
        vehicle.location.global_relative_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon))
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)


def fly_to(vehicle, targetLocation, groundspeed):
    print("Trying to fly")
    if (targetLocation.lat < 41.713799 or targetLocation.lat >41.715593):
        print("ERROR when assigning location! - Latitude outside range!")
        return
    if (targetLocation.lon < -86.244579 or targetLocation.lon > -86.236527):
        print("ERROR when assigning location! - Longitude outside range!")
        return

    print("Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(
        vehicle.location.global_relative_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon))
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = util.get_distance_meters(currentTargetLocation, vehicle.location.global_relative_frame)
        # print("Distance to target: "+str(remainingDistance))
        if remainingDistance < 1:
            print("Reached target "+ str(remainingDistance))
            break
        time.sleep(1)

def get_location(vehicle):
    return vehicle.location.global_relative_frame

def return_to_launch(vehicle):
    vehicle.mode = VehicleMode("RTL")

