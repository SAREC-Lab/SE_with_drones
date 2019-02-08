import json
import os
import threading
import socket
import time
from boltons import socketutils



def message_from_vehicle(vehicle):
    lla = vehicle.location.global_relative_frame
    att = vehicle.attitude
    vel = vehicle.velocity
    battery = {
        "voltage": vehicle.battery.voltage,
        "current": vehicle.battery.current,
        "level": vehicle.battery.level,

    }
    data = {
        "location": {"x": lla.lat, "y": lla.lon, "z": lla.alt},
        "status": vehicle.system_status.state,
        "heading": vehicle.heading,
        "groundspeed": vehicle.airspeed,
        "mode": vehicle.mode.name
    }

    return data