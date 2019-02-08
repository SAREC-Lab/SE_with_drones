
from connection import MessageReceiver, MessageSender
import sys
import threading
import time
import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
import util
import os
import json




MESSAGE_FREQUENCY=2

class Vehicle():

    vehicles = {}

    def initializeVehicle(self, ardupath, connection_string, home,name):
        self.logActive=False
        home = home +",221,0"
        if not connection_string:
            sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
            sitl_args = ['-I{}'.format(len(Vehicle.vehicles)), '--home', home, '--model', '+', '--defaults', sitl_defaults]
            sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))

            sitl.launch(sitl_args, await_ready=True)

            tcp, ip, port = sitl.connection_string().split(':')
            port = str(int(port) + len(Vehicle.vehicles) * 10)
            connection_string = ':'.join([tcp, ip, port])

        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=True)
        Vehicle.vehicles[name]=self.vehicle
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


class LeadVehicle(Vehicle):

    def start(self, ardupath, connection_string, home, name,address,port):
        # Vehicle.vehicles["xxx"]=None
        self.initializeVehicle(ardupath, connection_string, home, name)

        sender = MessageSender(name)
        sender.connect(address, port)

        threading.Thread(target=self.work, args=(sender,)).start()

    def work(self,sender):
        time.sleep(5)
        while True:
                state = util.message_from_vehicle(self.vehicle)
                sender.sendMessage(json.dumps(state))
                time.sleep(MESSAGE_FREQUENCY)


class FollowVehicle(Vehicle):

    def start(self, ardupath, connection_string, home, name,port):
        self.initializeVehicle(ardupath, connection_string, home, name)

        receiver = MessageReceiver("def",self)
        receiver.connect(port)


    status="ON_GROUND"

    def handleMessage(self,message):
        msg = json.loads(message)


