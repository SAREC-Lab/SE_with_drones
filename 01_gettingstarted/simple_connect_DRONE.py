from dronekit import connect, VehicleMode, time
import dronekit
import os


vehicle = dronekit.connect("/dev/ttyUSB0",wait_ready=True)
vehicle.wait_ready(timeout=120)

while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(3)
print("Arming motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True