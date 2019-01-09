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

_LOG = logging.getLogger(__name__)
_LOG.setLevel(logging.INFO)
if not os.path.exists('log'):
    os.makedirs('log')



fh = logging.FileHandler('log/gcs-main.log', mode='w')
fh.setLevel(logging.INFO)
formatter = logging.Formatter('| %(levelname)6s | %(funcName)8s:%(lineno)2d | %(message)s |')
fh.setFormatter(formatter)
_LOG.addHandler(fh)

DO_CONT = False
MESSAGE_FREQUENCY=1.0

class SimpleGCS:
    sitls = []
    vehicles = {}

    def __init__(self,ardupath,g_id="default_groundstation"):
        self.g_id=g_id
        self.ardupath=ardupath

    def registerDrone(self, home,name,virtual=True):
        if name is  None:
            name = get_vehicle_id(len(self.vehicles))

        if virtual:
            vehicle, sitl = self.connect_virtual_vehicle(len(self.vehicles), home)
            self.sitls.append(sitl)
        else:
            vehicle = self.connect_physical_vehicle(home)
        handshake = util.DroneHandshakeMessage.from_vehicle(vehicle, self.dronology._g_id,name)

        self.vehicles[name]=vehicle
        self.dronology.send(str(handshake))
        print("New drone registered.."+handshake.__str__())
        return vehicle

    def connect(self):
        self.dronology = util.Connection(None, "localhost", 1234,self.g_id)
        self.dronology.start()
        global DO_CONT
        DO_CONT = True
        w0 = threading.Thread(target=state_out_work, args=(self.dronology, self.vehicles))
        w0.start()

    def connect_virtual_vehicle(self,instance, home):
        home_ = tuple(home) + (0,) + (0,)
        home_ = ','.join(map(str, home_))
        print(home_)
        sitl_defaults = os.path.join(self.ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
        sitl_args = ['-I{}'.format(instance), '--home', home_, '--model', '+', '--defaults', sitl_defaults]
        sitl = dronekit_sitl.SITL(path=os.path.join(self.ardupath, 'build', 'sitl', 'bin', 'arducopter'))
        sitl.launch(sitl_args, await_ready=True)

        tcp, ip, port = sitl.connection_string().split(':')
        port = str(int(port) + instance * 10)
        conn_string = ':'.join([tcp, ip, port])

        vehicle = dronekit.connect(conn_string)
        vehicle.wait_ready(timeout=120)

        return vehicle, sitl

    def connect_physical_vehicle(self, home):

        vehicle = dronekit.connect(home, wait_ready=True)
        vehicle.wait_ready(timeout=120)

        return vehicle


def get_vehicle_id(i):
    return 'drone{}'.format(i)


def state_out_work(dronology, vehicles):
    while DO_CONT:
       # for i, v in enumerate(vehicles):
        for name, v in vehicles.iteritems():
            state = util.StateMessage.from_vehicle(v,dronology._g_id,name)
            state_str = str(state)
            _LOG.info(state_str)
            dronology.send(state_str)

        time.sleep(MESSAGE_FREQUENCY)
