#!/usr/bin/python3

import threading
import time
import socket
import select

class TCPServer:
    def __init__(self):
        self.RECV_BUFFER = 16
        self.HOST = "192.168.4.1"
        self.PORT = 5000
  
        self.dataIn = []
        self.dataOut = []
        self.TCP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TCP_server_socket.bind((self.HOST, self.PORT))
        self.TCP_server_socket.listen(1)
        self.TCP_connection = None
        self.TCP_client_address = None

    def serverLoop(self):
        while True:
            try:        
                print("Waiting for client...")
                self.TCP_connection, self.TCP_client_address = self.TCP_server_socket.accept()
                print("Connection from ", self.TCP_client_address)
                while True:            
                    self.dataIn = self.TCP_connection.recv(self.RECV_BUFFER)
                    if not self.dataIn:
                        self.TCP_server_socket.close()
                        break
                    else:                    
                        
                        self.TCP_connection.sendall(bytes(self.dataOut))

            except KeyboardInterrupt:
                self.TCP_server_socket.close()

            finally:
                self.dataIn = [0x1C, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                


