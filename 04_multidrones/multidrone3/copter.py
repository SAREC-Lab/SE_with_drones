# Import DroneKit-Python
import dronekit_sitl
import dronekit
import os
import thread

ARDUPATH = "/home/uav/git/ardupilot"

class UAV_Copter:

    def setvalues(self,sitl,vehicle,waypoints,vehicle_id):
        print("\n Adding: " + vehicle_id)
        self.sitl = sitl
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.vehicle_id = vehicle_id

    def connect_vehicle(self,instance, home):
        home_ = tuple(home) + (0,)
        home_ = ','.join(map(str, home_))
        sitl_defaults = os.path.join(ARDUPATH, 'Tools', 'autotest', 'default_params', 'copter.parm')
        sitl_args = ['-I{}'.format(instance), '--home', home_, '--speedup', '2','--model', '+', '--defaults', sitl_defaults]
        sitl = dronekit_sitl.SITL(path=os.path.join(ARDUPATH, 'build', 'sitl', 'bin', 'arducopter'))
        sitl.launch(sitl_args, await_ready=True)

        tcp, ip, port = sitl.connection_string().split(':')
        port = str(int(port) + instance * 10)
        conn_string = ':'.join([tcp, ip, port])

        vehicle = dronekit.connect(conn_string)
        vehicle.wait_ready(timeout=120)

        return vehicle, sitl

    def print_drone_state(self):
        # print("\nDrone State for : " + self.vehicle.id)
        # Get some self.vehicle attributes (state)
        print "Autopilot_version: %s" % self.vehicle.version
        print "Get some self.vehicle attribute values:"
        print "GPS: %s" % self.vehicle.gps_0
        print "Battery: %s" % self.vehicle.battery
        print "Last Heartbeat: %s" % self.vehicle.last_heartbeat
        print "Is Armable?: %s" % self.vehicle.system_status.state
        print "Mode: %s" % self.vehicle.mode.name  # settable
        print self.vehicle.location.global_relative_frame.lat
        print self.vehicle.location.global_relative_frame.lon
        # Close self.vehicle object before exiting script

    def getid(self):
        return self.vehicle_id