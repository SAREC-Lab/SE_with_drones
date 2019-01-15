import math
import os
import re
import numpy as np
import matplotlib.pyplot as plt


#logfile = open("results.log","w")
lat_array = []
lon_array = []


class Location:
    def __init__(self,lat=0.0,lon=0.0):
        """ Create a new point at the origin """
        self.lat = lat
        self.lon = lon

def point_on_circle(radius, angle_indegrees, latitude, longitude):
    #Convert from degrees to radians
    lon = (radius*math.cos(angle_indegrees * math.pi / 180)) + longitude
    #print("lat: %f", lon)
    lat = (radius*math.sin(angle_indegrees * math.pi / 180)) + latitude
    #print("lon: %f", lat)
    #logfile.write(str(lat)+","+str(lon)+"\n")

    return Location(lat,lon)

def scatter_plot(x,y,x_label,y_label,title):
    plt.plot(x,y)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.show()


center = Location(41.7144367,-86.2417136)
radius = .00001
angle = 0
while angle <=45:
    point = (point_on_circle(radius,angle,center.lat, center.lon))
    print("Position for angle:",angle, point.lat, point.lon)
    lat_array.append([])
    lon_array.append([])
    lat_array[-1].append(point.lat)
    lon_array[-1].append(point.lon)
    angle = angle +1

#logfile.close()
scatter_plot(lon_array,lat_array,"Longitude","Latitude","")