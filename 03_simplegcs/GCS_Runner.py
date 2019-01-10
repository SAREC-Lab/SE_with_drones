import argparse
import json
import os

from dronekit import LocationGlobalRelative

import experimentrunner.Logger as Logger
import experimentrunner.Vehicle as Vehicle
from experimentrunner import ExperimentConfig as config
c
########### Configuration Properties ###########

SIMPLE_GCS = False
ARDUPATH = os.path.join('/', 'home', 'uav', 'git', 'ardupilot')
LOG_FILE_NAME = "experiment.log"
Vehicle.LOG_TIME = 0.5

########### Configuration Properties ###########


if SIMPLE_GCS:
    from experimentrunner.Vehicle import GCSVehicleInitializer  as VehicleManager
else:
    from experimentrunner.Vehicle import SimpleVehicleInitializer as VehicleManager


def runmission():
    config.getMission().run()


def checkCommand(command):
    if (command['id'] == 'TAKE-OFF'):
        pass
    elif (command['id'] == 'GOTO'):
        pass
    elif (command['id'] == 'RTL'):
        pass
    else:
        raise NotImplementedError("Unknown command: " + command['id'])


def executeCommand(command, vehiclehandler):
    if (command['id'] == 'TAKE-OFF'):
        altitude = command['altitude']
        print("TAKING OFF TO: " + str(altitude))
        vehiclehandler.arm_and_takeoff(altitude)
    elif (command['id'] == 'GOTO'):
        lat = float(command['latitude'])
        lon = float(command['longitude'])
        alt = float(command['altitude'])
        speed = float(command['speed'])
        print("GOING: " + str(speed))
        vehiclehandler.fly_to(LocationGlobalRelative(lat, lon, alt), speed, command['tag-start'], command['tag-end'])
    elif (command['id'] == 'RTL'):
        vehiclehandler.return_to_launch()


def loadmission(filename):
    with open(filename) as f:
        data = json.load(f)
    for command in data['commands']:
        checkCommand(command)

    Logger.initLogger(LOG_FILE_NAME)
    Logger.startLogging("1", "ASD", "VRTL", "EX1");

    home = str(data['meta-data']['home'])
    CONNECTION_STRING = "/dev/ttyUSB0,56700"
    vehicleManager = VehicleManager();
    vehicle = vehicleManager.initializeVehicle(ARDUPATH, CONNECTION_STRING, home)

    for command in data['commands']:
        executeCommand(command, vehicleManager)

    # pprint(data)


def main():
    parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    parser.add_argument('--missionfile', help="json-file with mission steps")
    args = parser.parse_args()

    connection_string = args.connect
    filename = args.missionfile

    if filename is None:
        runmission();
    else:
        loadmission(filename)


if __name__ == '__main__':
    main()


