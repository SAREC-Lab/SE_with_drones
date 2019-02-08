import os
import time

from vehicle import FollowVehicle, LeadVehicle

ARDUPATH = os.path.join('/', 'home', 'uav', 'git', 'ardupilot')
CONNECTION_STRING = None


ADDRESS = "localhost"

if __name__ == '__main__':
     home = "41.714521, -86.241855"
     print("Starting Lead Vehicle...")
     time.sleep(5)


     leadVehicle= LeadVehicle()
     leadVehicle.start(ARDUPATH,CONNECTION_STRING,home,"Leader",ADDRESS,1234)

     leadVehicle.arm_and_takeoff(15)

     time.sleep(60)
     leadVehicle.land()
     time.sleep(60)