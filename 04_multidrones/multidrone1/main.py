#Calling program for demonstrating multiple SITL drones
from copter import UAV_Copter
import json

drones = list()
copter_counter = 0

# Create N copter vehicles (1)
for d in range(3):
    print("\nAdding drone: " + str(copter_counter))
    copter = UAV_Copter()  # Create instance of uav
    copter.initialize_drone(copter_counter)  # Initialize
    copter_counter= copter_counter+1  # Used to ensure unique ports for each SITL instance
    drones.append(copter)

# Show state
for d in drones:
    d.print_drone_state()

# Close all drones
for d in drones:
    d.close_vehicle()









