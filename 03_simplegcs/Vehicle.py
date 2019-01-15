import os
import sys
import threading

import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from Logger import Logger as Log
import UtilFunctions as util
import simplegcs as simplegcs


LOG_TIME=0.5

class VehicleManager(object):
    def initializeVehicle(self, ardupath, connection_string, home):
        raise NotImplementedError

    def arm_and_takeoff(self,aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        print("Waiting for vehicle to initialise!")
        while not self.vehicle.is_armable:
            sys.stdout.write('.')
            time.sleep(2)

        print("home: " + str(self.vehicle.location.global_relative_frame.lat))

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        print("Waiting for arming...")
        while not self.vehicle.armed:
            sys.stdout.write('.')
            time.sleep(2)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        Log.logmessage(self.vehicle, "TKOF-1-STRT", "")
        while True:
            # print " Altitude: ", vehicle.location.global_relative_frame.alt
            Log.logmessage(self.vehicle, "ASCENDING  ", "")
            # Break and return from function just below target altitude.
            print("Target altitude: " + str(self.vehicle.location.global_relative_frame.alt))
            time.sleep(1)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * .90:
                print("Reached target altitude")
                break

        time.sleep(1)
        Log.logmessage(self.vehicle, "TKOF-1-END ", "")

    def altitude_to(self, targetAltitude, startTag, endTag):
        targetLocation = LocationGlobalRelative(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,targetAltitude);
        self.currentTargetLocation = targetLocation
        remainingAltitude= targetAltitude - self.vehicle.location.global_relative_frame.alt
        Log.logmessage(self.vehicle, startTag, str(remainingAltitude))
        self.vehicle.simple_goto(self.currentTargetLocation)
        self.logActive = True
        while self.vehicle.mode.name == "GUIDED":
            remainingAltitude = targetAltitude - self.vehicle.location.global_relative_frame.alt
            print("Altitude: " + str(remainingAltitude))
            currAlt = self.vehicle.location.global_relative_frame.alt;
            if  currAlt >= targetAltitude * .95 and currAlt< targetAltitude * 1.05:
                print("Reached target altitude "+ str(startTag))
                self.logActive = False
                break;
            time.sleep(1)
        Log.logmessage(self.vehicle, endTag, "")

    def fly_to(self, targetLocation, groundspeed, startTag, endTag, breakout_point=None):
        if breakout_point is None:
            breakout_point = 1
        if (targetLocation.lat < 41.713799 or targetLocation.lat >41.715593):
            print("ERROR when assigning location! - Latitude outside range!")
            return
        if (targetLocation.lon < -86.244579 or targetLocation.lon > -86.236527):
            print("ERROR when assigning location! - Longitude outside range!")
            return

        print("Flying from: " + str(self.vehicle.location.global_frame.lat) + "," + str(
            self.vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon)+"  --  "+str(startTag))
        self.vehicle.groundspeed = groundspeed
        self.currentTargetLocation = targetLocation
        self.vehicle.simple_goto(self.currentTargetLocation)
        remainingDistance = util.get_distance_meters(self.currentTargetLocation, self.vehicle.location.global_frame)

        Log.logmessage(self.vehicle, startTag, str(remainingDistance))
        self.logActive=True
        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = util.get_distance_meters(self.currentTargetLocation, self.vehicle.location.global_frame)
            # print("Distance to target: "+str(remainingDistance))
            # Log.logmessage(self.vehicle, "continue", str(remainingDistance));
            if remainingDistance < breakout_point:
                print("Reached target "+ str(remainingDistance))
                self.logActive = False
                break;
            time.sleep(1)
        Log.logmessage(self.vehicle, endTag, "")

    def fly_to_closest_point(self, points,speed):
        # get closest point
        close = sys.maxint
        targetPoint = self.vehicle.location.global_frame
        i = found = 0
        for point in points:
            distanceToPoint = util.get_distance_meters(point, self.vehicle.location.global_frame)
            if (close > distanceToPoint):
                close = distanceToPoint
                found = i
                targetPoint = LocationGlobalRelative(point.lat, point.lon, point.alt)
            i += 1
            # 41.714473, -86.24252, 10)
        print("Closest point found")
        self.fly_to(targetPoint, speed, "STRT_TOPOINT", "END_TOPOINT")
        return found

    def get_vehicle_location(self):
        return self.vehicle.location.global_relative_frame

    def logperiodic(self):
        threading.Timer(LOG_TIME, self.logperiodic).start()
        if(self.logActive):
            remainingDistance = format(util.get_distance_meters(self.currentTargetLocation, self.vehicle.location.global_frame),'.3f')

            print("Distance to target: " + str(remainingDistance))
            Log.logmessage(self.vehicle, "FLYING     ", str(remainingDistance));

    def return_to_launch(self):
        Log.logmessage(self.vehicle, "RTL     ", "")
        self.vehicle.mode = VehicleMode("RTL")

class SimpleVehicleInitializer(VehicleManager):
    def initializeVehicle(self, ardupath, connection_string, home):
        self.logActive=False
        super(SimpleVehicleInitializer, self).logperiodic()
        home = home +",221,0"
        if not connection_string:
            sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
            sitl_args = ['-I{}'.format(0), '--home', home, '--model', '+', '--defaults', sitl_defaults]
            sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
            sitl.launch(sitl_args, await_ready=True)

            tcp, ip, port = sitl.connection_string().split(':')
            port = str(int(port) + 0 * 10)
            connection_string = ':'.join([tcp, ip, port])

        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=False)
        self.vehicle.wait_ready(timeout=500)
        return self.vehicle


class GCSVehicleInitializer(VehicleManager):
    def initializeVehicle(self, ardupath, connection_string, home):
        self.connect(ardupath)

        #### physical...
        if connection_string is not None and connection_string.startswith( '/dev/tty'):
            print("Activating Physical Drone...." +str(connection_string))
            vehicle = self.addDrone(connection_string,False)
        else:
            print("Activating Virtual Drone...." + str(home))
            vehicle = self.addDrone(home.split(","),True)

        #### --> connec
        self.logActive = False
        super(GCSVehicleInitializer, self).logperiodic()
        return vehicle


    def addDrone(self,home, virtual, name=None):
        self.vehicle = GCS.registerDrone(home, name, virtual)
        time.sleep(5)
        while not self.vehicle.is_armable:
            sys.stdout.write("...")
            time.sleep(3)
        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            sys.stdout.write("...")
            time.sleep(1)
        return self.vehicle

    def connect(self,ardupath):
        print("Connecting to Dronology...")
        print("SITL path: " + ardupath)
        global GCS
        GCS = simplegcs.SimpleGCS(ardupath, "simplegcs")
        GCS.connect()
        time.sleep(10)

