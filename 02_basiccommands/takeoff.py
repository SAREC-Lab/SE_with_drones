import os
import sys
import threading
import time
import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative

class VehicleWrapper():
    def initializeVehicle(self, ardupath, connection_string, home):
        self.logActive=False
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
        self.vehicle = connect(connection_string, wait_ready=True)
        self.vehicle.wait_ready(timeout=500)
        return self.vehicle

    def arm_and_takeoff(self,aTargetAltitude):
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

    def land(self):
        print("LANDING")
        self.vehicle.mode = VehicleMode("LAND")


ARDUPATH = os.path.join('/', 'home', 'uav', 'git', 'ardupilot')
#CONNECTION_STRING = "/dev/ttyUSB0,57600"
CONNECTION_STRING = None
if __name__ == '__main__':
     home = "41.714521, -86.241855"
     wrapper= VehicleWrapper()
     wrapper.initializeVehicle(ARDUPATH,CONNECTION_STRING,home)
     wrapper.arm_and_takeoff(5)
     time.sleep(10)
     wrapper.land()
     time.sleep(10)
