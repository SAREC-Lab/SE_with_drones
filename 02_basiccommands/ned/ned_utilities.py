#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
NED Utilities
"""
from pymavlink import mavutil
import time
import numpy as np
import nvector as nv
from nvector import rad, deg
from math import sin, cos, atan2, radians, sqrt
from ned.flight_plotter import Location

wgs84 = nv.FrameE(name='WGS84')


class Nedvalues:
    def __init__(self, north=0.0, east=0.0, down=0.0):
        """ Create a new point at the origin """
        self.north = north
        self.east = east
        self.down = down


class ned_controller:

    ################################################################################################
    # function:    Get distance in meters
    # parameters:  Two global relative locations
    # returns:     Distance in meters
    ################################################################################################
    def get_distance_meters(self, locationA, locationB):
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

    # Sends velocity vector message to UAV vehicle
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration, vehicle):
        """
        Move vehicle in a direction based on specified velocity vectors.
        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        for x in range(0, duration):
            vehicle.send_mavlink(msg)

        time.sleep(0.1)

        # Sends velocity vector message to UAV vehicle

    def send_ned_stop(self, vehicle):
        count = 1
        outerloopcounter = 1

        currentVelocity = vehicle.velocity
        north = currentVelocity[0]
        south = currentVelocity[1]
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            -north*100,-south*100,0,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        while True:
            if vehicle.groundspeed >  1 and outerloopcounter <= 100:
                if count == 100:
                    count = 1
                    print(vehicle.groundspeed)
                    outerloopcounter = outerloopcounter + 1
                else:
                    count = count + 1

                vehicle.send_mavlink(msg)

            if outerloopcounter == 100:
                break

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
         mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
         0, 0, 0,  # x, y, z positions (not used)
         0,0, 0,  # x, y, z velocity in m/s
         0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
         0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        time.sleep(0.1)

    # Sets NED given a current and target location
    def setNed(self, current, target):
        lat_C, lon_C = rad(current.lat), rad(current.lon)
        lat_T, lon_T = rad(target.lat), rad(target.lon)
        # create an n-vector for current and target
        nvecC = nv.lat_lon2n_E(lat_C, lon_C)
        nvecT = nv.lat_lon2n_E(lat_T, lon_T)
        # create a p-vec from C to T in the Earth's frame
        # the zeros are for the depth (depth = -1 * altitude)
        p_CT_E = nv.n_EA_E_and_n_EB_E2p_AB_E(nvecC, nvecT, 0, 0)
        # create a rotation matrix
        # this rotates points from the NED frame to the Earth's frame
        R_EN = nv.n_E2R_EN(nvecC)
        # rotate p_CT_E so it lines up with current's NED frame
        # we use the transpose so we can go from the Earth's frame to the NED frame
        n, e, d = np.dot(R_EN.T, p_CT_E).ravel()

        #Scale Nedvalues
        if n > e:
            scaleFactor = abs(1.00000/n)
        else:
            scaleFactor = abs(1.00000/e)

        return Nedvalues(n*scaleFactor, e*scaleFactor, d)
        #return Nedvalues(n,e,d)

