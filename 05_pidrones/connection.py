import socket
import threading
import time
from boltons import socketutils
import json
import os
class MessageSender():
    def __init__(self, id, ):
        self.id=id


    _WAITING = 1
    _CONNECTED = 2
    _DEAD = -1

    status = _WAITING

    def connect(self,address,port):
        threading.Thread(target=self.connectTo,args=(address,port,)).start()

    def connectTo(self,address,port):
        try:
            sock = socket.create_connection((address, port), timeout=5.0)
            self._sock = socketutils.BufferedSocket(sock)
            print(">>> Connected to lead vehicle "+ address + " on port  "+ str(port))
        except socket.error as e:
            print('-Socket error ({})'.format(e))
            print(e)
            time.sleep(10.0)

    def sendMessage(self,message):
        print("SEND:"+message)
        self._sock.send(message)
        self._sock.send(os.linesep)


class MessageReceiver():
    def __init__(self, id,callback ):
        self.id = id
        self.callback=callback

    _WAITING = 1
    _CONNECTED = 2
    _DEAD = -1

    status = _WAITING

    def connect(self,port):
        threading.Thread(target=self.createConnection, args=(port,)).start()

    def createConnection(self, port):
        try:
            serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            serv.bind(('0.0.0.0', port))
            print(">>> Opening connection on port  " + str(port))
            serv.listen(5)
            conn, addr = serv.accept()
            print(">>> Connection established")
            self._sock = socketutils.BufferedSocket(conn)
            while True:
                msg = self._sock.recv_until(os.linesep, timeout=50)
                print("RECEIVED: "+msg)
                self.callback.handleMessage(msg)
        except socket.error as e:
            print('>Socket error ({})'.format(e))
            time.sleep(10.0)



