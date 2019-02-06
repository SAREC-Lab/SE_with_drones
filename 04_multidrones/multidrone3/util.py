import json
import os
import threading
import socket
import time
from boltons import socketutils


class DronologyMessage(object):
    def __init__(self, m_type,g_id, uav_id, data):
        self.m_type = m_type
        self.g_id=g_id
        self.uav_id = uav_id
        self.data = data

    def __str__(self):
        return json.dumps({'type': self.m_type,
                           'sendtimestamp': long(round(time.time() * 1000)),
                           'uavid': str(self.uav_id),
                           'groundstationid': str(self.g_id),
                           'data': self.data})

    def __repr__(self):
        return str(self)


class DroneHandshakeMessage(DronologyMessage):
    def __init__(self, g_id,uav_id, data, p2sac='../cfg/sac.json'):
        super(DroneHandshakeMessage, self).__init__('handshake',g_id, uav_id, data)
        self.p2sac = p2sac

    @classmethod
    def from_vehicle(cls, vehicle,g_id, v_id, p2sac='../cfg/sac.json'):
        battery = {
            'voltage': vehicle.battery.voltage,
            'current': vehicle.battery.current,
            'level': vehicle.battery.level,
        }

        lla = vehicle.location.global_relative_frame
        data = {
            'home': {'x': lla.lat,
                     'y': lla.lon,
                     'z': lla.alt},
            'safetycase': json.dumps({})}
        return cls(g_id,v_id, data)


class StateMessage(DronologyMessage):
    def __init__(self, g_id, uav_id, data):
        super(StateMessage, self).__init__('state',g_id, uav_id, data)

    @classmethod
    def from_vehicle(cls, vehicle,g_id, v_id, **kwargs):
        lla = vehicle.location.global_relative_frame
        att = vehicle.attitude
        vel = vehicle.velocity
        battery = {
            'voltage': vehicle.battery.voltage,
            'current': vehicle.battery.current,
            'level': vehicle.battery.level,
        }
        data = {
            'location': {'x': lla.lat, 'y': lla.lon, 'z': lla.alt},
            'attitude': {'x': att.roll, 'y': att.pitch, 'z': att.yaw},
            'velocity': {'x': vel[0], 'y': vel[1], 'z': vel[2]},
            'status': vehicle.system_status.state,
            'heading': vehicle.heading,
            'armable': vehicle.is_armable,
            'airspeed': vehicle.airspeed,
            'groundspeed': vehicle.airspeed,
            'armed': vehicle.armed,
            'mode': vehicle.mode.name,
            'batterystatus': battery
        }

        return cls(g_id,v_id, data)

class Connection:
    _WAITING = 1
    _CONNECTED = 2
    _DEAD = -1

    def __init__(self, msg_queue=None, addr='localhost', port=1234, g_id='default_groundstation'):
        self._g_id = g_id
        self._msgs = msg_queue
        self._addr = addr
        self._port = port
        self._sock = None
        self._conn_lock = threading.Lock()
        self._status = Connection._WAITING
        self._status_lock = threading.Lock()
        self._msg_buffer = ''

    def get_status(self):
        with self._status_lock:
            return self._status

    def set_status(self, status):
        with self._status_lock:
            self._status = status

    def is_connected(self):
        return self.get_status() == Connection._CONNECTED

    def start(self):
        threading.Thread(target=self._work).start()

    def stop(self):
        self.set_status(Connection._DEAD)

    def send(self, msg):
        success = False
        with self._conn_lock:
            if self._status == Connection._CONNECTED:
                try:
                    self._sock.send(msg)
                    self._sock.send(os.linesep)
                    success = True
                except Exception as e:
                    print('failed to send message! ({})'.format(e))

        return success

    def _work(self):
        """
        Main loop.
            1. Wait for a connection
            2. Once connected, wait for commands from dronology
            3. If connection interrupted, wait for another connection again.
            4. Shut down when status is set to DEAD
        :return:
        """
        cont = True
        while cont:

            status = self.get_status()
            if status == Connection._DEAD:
                # Shut down
                cont = False
            elif status == Connection._WAITING:
                # Try to connect, timeout after 10 seconds.
                try:
                    sock = socket.create_connection((self._addr, self._port), timeout=5.0)
                    self._sock = socketutils.BufferedSocket(sock)
                    handshake = json.dumps({'type': 'connect', 'groundstationid': self._g_id})
                    self._sock.send(handshake)
                    self._sock.send(os.linesep)
                    self.set_status(Connection._CONNECTED)
                except socket.error as e:
                    print('Socket error ({})'.format(e))
                    time.sleep(10.0)
            else:
                # Receive messages
                try:
                    msg = self._sock.recv_until(os.linesep, timeout=0.1)
                    if self._msgs is not None:
                        self._msgs.append(msg)
                        print ("Message received")
                        print (msg)
                except socket.timeout:
                    pass
                except socket.error as e:
                    print('connection interrupted! ({})'.format(e))
                    self._sock.shutdown(socket.SHUT_RDWR)
                    self._sock.close()
                    self._sock = None
                    self.set_status(Connection._WAITING)
                    time.sleep(20.0)

        if self._sock is not None:
            print('Shutting down socket.')
            self._sock.shutdown(socket.SHUT_WR)
            print('Closing socket.')
            self._sock.close()
            return