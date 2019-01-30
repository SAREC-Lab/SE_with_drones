# Import DroneKit-Python
from dronekit import connect, VehicleMode, time
import dronekit_sitl
import os
import thread

#Setup option parsing to get connection string
import argparse

class UAV_Copter:

    ################################################################################################
    # function:    Initialize Drone
    # parameters:  Vehicle ID (integer), Connection_string for physical drones
    # returns:     n/a
    ################################################################################################
    def initialize_drone(self, vehicle_num, home="41.7144367,-86.2417136,221,0",connection_string="", ):

        self.vehicle_id = "UAV-"+str(vehicle_num)
        print ("\nInitializing drone: " + self.vehicle_id)

        parser = argparse.ArgumentParser(description='Print out vehicle state information')
        parser.add_argument('--connect',help="vehicle connection target string. If not specified, SITL automatically started and used")
        args=parser.parse_args()

        connection_string = args.connect
        sitl = None

        #Start SITL if no connection string specified
        if not connection_string:
            import dronekit_sitl
            # connection_string = sitl.connection_string()

            ardupath = "/home/uav/git/ardupilot"
            self.home = home  # In this example, all UAVs start on top of each other!
            sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
            sitl_args = ['-I{}'.format(vehicle_num), '--home', home, '--model', '+', '--defaults', sitl_defaults]
            sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
            sitl.launch(sitl_args, await_ready=True)

            tcp, ip, port = sitl.connection_string().split(':')
            print (port + " " + str(vehicle_num))
            port = str(int(port) + vehicle_num * 10)
            connection_string = ':'.join([tcp, ip, port])


        ################################################################################################
        # Connect to the Vehicle
        ################################################################################################
        print 'Connecting to vehicle on: %s' % connection_string
        self.vehicle = connect(connection_string, wait_ready=True)
        self.vehicle.wait_ready(timeout=120)
        return self.vehicle, sitl
        time.sleep(10)

    def print_drone_state(self):

        print("\nDrone State for : " + self.vehicle_id )
        #Get some vehicle attributes (state)
        print "Autopilot_version: %s" % self.vehicle.version
        print "Get some vehicle attribute values:"
        print "GPS: %s" % self.vehicle.gps_0
        print "Battery: %s" % self.vehicle.battery
        print "Last Heartbeat: %s" % self.vehicle.last_heartbeat
        print "Is Armable?: %s" % self.vehicle.system_status.state
        print "Mode: %s" % self.vehicle.mode.name # settable
        print self.vehicle.location.global_relative_frame.lat
        print self.vehicle.location.global_relative_frame.lon
        # Close vehicle object before exiting script

    def close_vehicle(self):
        self.vehicle.close()
        time.sleep(5)
        print("Vehicle " + self.vehicle_id + " is closed")

