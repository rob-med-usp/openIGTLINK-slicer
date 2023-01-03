import socket
import threading
import time

class SimpleConnection:
    def __init__(self, port):

        # Create a TCP/IP socket
        self.port = port
        self.ip = self.get_ip()
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # self.conn.connect((self.ip, self.port))

        # Create a thread to try to connect to the server
        self.connected = False
        self.connect_thread = threading.Thread(target=self.try_connect)
        # self.connect_thread.start()
    
    def try_connect(self):
        
        while True:
            time.sleep(1)
            try:
                if self.connected and self.is_connected():
                    time.sleep(1)
                    continue
                else:
                    self.connected = False

                print("Trying to connect to the server...")
                self.conn.connect((self.ip, self.port))
                
                print(f'Connected to server at {self.ip}:{self.port}')
                self.connected = True
            
            except Exception:
                self.connected = False
    
    # Check if the connection is established
    def is_connected(self):
        if self.conn.sendall(' '.encode()) == 0:
            self.connected = False
        else:
            self.connected = True

    # Get IP address of the computer
    def get_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            # doesn't even have to be reachable
            s.connect(('10.254.254.254', 1))
            IP = s.getsockname()[0]
        except Exception:
            IP = '127.0.0.1'
        finally:
            s.close()
        return IP
    
    # Send the eletrode control points to the server
    def sendElectrode(self, points_arr):
        # connect to server
        try:
            self.conn.connect((self.ip, self.port))
            self.connected = True
        except Exception:
            print('Server not reachable.')
            self.connected = False
            return
        
        # build message from poin array
        msg = ' '.join(str(coord) for coord in points_arr.ravel())

        # input a \n char on the end of the message
        msg += '\n'

        # send message
        if self.connected:
            self.conn.send(msg.encode())
            print(f'Sent: {msg}')
            self.conn.close()
            self.connected = False

        return