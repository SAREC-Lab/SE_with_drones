import argparse
import json
import os

from dronekit import LocationGlobalRelative

import Logger as Logger
import Vehicle as Vehicle

########### Configuration Properties ###########

SIMPLE_GCS = False
ARDUPATH = os.path.join('/', 'home', 'uav', 'git', 'ardupilot')
LOG_FILE_NAME = "experiment.log"
Vehicle.LOG_TIME = 0.5
#CONNECTION_STRING="/dev/ttyUSB0,56700"
CONNECTION_STRING = None
########### Configuration Properties ###########


if SIMPLE_GCS:
    from Vehicle import GCSVehicleInitializer  as VehicleManager
else:
    from Vehicle import SimpleVehicleInitializer as VehicleManager






def main():
    home="41.714521,-86.241855"
    vehicleManager=VehicleManager()
    vehicle=vehicleManager.initializeVehicle(ARDUPATH,CONNECTION_STRING,home)





if __name__ == '__main__':
    main()


