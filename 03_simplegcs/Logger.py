import datetime
import logging
import os
import threading
import time
from math import sqrt
from shutil import copyfile



FORMATTER = logging.Formatter(fmt='%(asctime)s.%(msecs)03d%(message)s',datefmt='%H:%M:%S') #datefmt="%Y-%m-%d %H:%M:%S.%s.%03d")
LOG_FOLDER="log"
BACKUP_FOLDER="log-backup"

class Logger():

    @staticmethod
    def initLogger(filename):
        full_file = os.path.join(LOG_FOLDER,filename)
        Logger.logger = logging.getLogger("ExperimentLogger")
        Logger.logger.setLevel(logging.INFO)
        if not os.path.exists(LOG_FOLDER):
            os.makedirs(LOG_FOLDER)

        if os.path.isfile(full_file):
            backup(filename)
            os.remove(full_file)
        fh = logging.FileHandler(full_file)
        fh.setFormatter(FORMATTER)
        Logger.logger.addHandler(fh)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setFormatter(FORMATTER)
        #Logger.logger.addHandler(consoleHandler)
        print("Setting up log file")

    @staticmethod
    def startLogging(experimentNumber, experimentDescription,droneType,experimenter_initials ):
        experimentNumber = experimentNumber
        experimentDescription=experimentDescription
        droneType= droneType
        logging.getLogger("ExperimentLogger").info(
            ",METADATA," + "DroneType: " + droneType + " (" + experimenter_initials + "): " + datetime.datetime.now().strftime(
                "%I:%M%p, %B %d %Y") + " Exp #: " + str(experimentNumber) + ": " + experimentDescription)


# Do not change this one

# function:    log message
# parameters:  Vehicle, command issued, optional text
    @staticmethod
    def logmessage(vehicle,command,extra):
        #cmds = vehicle.commands
        #cmds.download()
        #cmds.wait_ready()
        xv = vehicle.velocity[0]
        xy = vehicle.velocity[1]
        xz = vehicle.velocity[2]
        altitude =format('%06.3f' % vehicle.location.global_relative_frame.alt)
        battery=vehicle.battery.level

        lat = format(vehicle.location.global_relative_frame.lat, '.7f')
        lon = format(vehicle.location.global_relative_frame.lon, '.7f')

        speed = format(sqrt((xv*xv)+(xy*xy)+(xz*xz)),'.3f')
        #print "X velocity " + str(xv)
        message = "," + command + "," +  str(lat) + "," + str(lon) + "," + str(altitude) + "," + str(speed) + "," + str(time.time()) + "," + extra + "," + str(vehicle.battery.voltage) + "," + str(battery) + ","
        logging.getLogger("ExperimentLogger").info(message)



def backup(filename):
    if not os.path.exists(BACKUP_FOLDER):
        os.makedirs(BACKUP_FOLDER)
    i=1
    tocheck = os.path.join(BACKUP_FOLDER, filename+"_"+str(i))
    while(os.path.isfile(tocheck)):
        i+=1
        tocheck = str(BACKUP_FOLDER + "/" + filename + "_" + str(i))
    full_file = os.path.join(LOG_FOLDER, filename)
    copyfile(full_file,tocheck)