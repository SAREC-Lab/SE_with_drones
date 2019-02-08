import os
import time

from vehicle import FollowVehicle, LeadVehicle

ARDUPATH = os.path.join('/', 'home', 'uav', 'git', 'ardupilot')
CONNECTION_STRING = None



if __name__ == '__main__':
     home = "41.714521, -86.241855"
     print("Starting Follow Vehicle...")


     time.sleep(5)

     followVehicle = FollowVehicle()
     followVehicle.start(ARDUPATH, CONNECTION_STRING, home, "Follower", 1234)


     time.sleep(60)