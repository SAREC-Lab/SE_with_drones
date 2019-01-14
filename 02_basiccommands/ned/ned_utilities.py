#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
NED Utilities
"""
from pymavlink import mavutil
import time

class Nedvalues:
    def __init__(self,north=0.0,east=0.0,down=0.0):
        """ Create a new point at the origin """
        self.north = north
        self.east = east
        self.down = down

class ned_controller:
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration,vehicle):
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

        time.sleep(1)

    def setNED(self,currentLocation,targetLocation):
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


