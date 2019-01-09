from dronekit import connect, VehicleMode, time
import dronekit_sitl
import dronekit
import os

ardupath ="/home/uav/git/ardupilot"
home="41.715446209367,-86.242847096132"
sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
sitl_args = ['-I{}'.format(0), '--home', home, '--model', '+', '--defaults', sitl_defaults]
sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
sitl.launch(sitl_args, await_ready=True)

tcp, ip, port = sitl.connection_string().split(':')
port = str(int(port) + 0 * 10)
conn_string = ':'.join([tcp, ip, port])

vehicle = dronekit.connect(conn_string)
vehicle.wait_ready(timeout=120)

while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(3)
print("Arming motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True