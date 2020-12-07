#!/usr/bin/python3

from Motor import Motor
from MyPS4Controller import MyPS4Controller, MyPS4EventDefinition
import threading
import time
import os

class RobotPiController:
    def __init__(self):        
        self.ps4ControllerThread = ThreadControllerClass("PS4ControllerThread")
        self.tcpServerThread = ThreadControllerClass("TCPServer")
        self.ps4ControllerThread.start()
        self.tcpServerThread.start()
        self.ps4ControllerThread.join()
        self.tcpServerThread.join()

        
    
    

class ThreadControllerClass(threading.Thread):
    def __init__(self, threadName):
        threading.Thread.__init__(self)
        self.threadName = threadName

        
    def run(self):
        if self.threadName == "PS4ControllerThread":
            #ide kellene bejátszani a Motor class member tagjait!!!!!!!!!!!!!
            controller = MyPS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False, event_definition=MyPS4EventDefinition)
            while os.system("sudo bluetoothctl -- connect DC:AF:68:A5:86:0F") != 0:
                print("Retry to connect the PS4 controller!")    
            controller.listen()
            
        if self.threadName == "TCPServer":
            print ("Starting dummy TCP server thread")
            while True:
                print("Dummy message!")
                time.sleep(1)
            print("End TCP server")

        

						
