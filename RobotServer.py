#!/usr/bin/python3

import RPi.GPIO as GPIO
import socket
import select
import time
import string
import sys
import math
import os
import serial
from PCA9685 import PCA9685
import ADS1118
import typek

#Motor Controll Board Pins
MA1 = 20
MA2 = 21
MB1 = 6
MB2 = 13
motorPWM1Pin = 26
motorPWM2Pin = 12

p1=None
p2=None

actualLeftMotorPWM = 0
actualRightMotorPWM = 0

#TCP server dataIn
##################################################################################################################################################################################################################################################
#      0            1           2           3           4               5               6           7               8               9                   10              11          12          13          14          15
# Motors dir    Left motor  Right motor  Arm1 servo   Arm2 servo    Arm3 servo      Arm4 servo   Cam H servo    Cam V servo    Distance On/Off   Left Lamp PWM    Right Lamp PWM   Reserve    Reserve      Reserve     Reserve
##################################################################################################################################################################################################################################################
#TCP server dataOut
############################################################################################################################################################################################################################################################
#      0            1           2           3           4               5               6           7               8               9               10                      11              12                  13                  14                  15
# Motors dir    Left motor  Right motor  Arm1 servo   Arm2 servo    Arm3 servo      Arm4 servo   Cam H servo    Cam V servo    Distance value  Left Lamp status    Right Lamp status   ADC Current Value H  ADC Current Value L    ADC 12V Value H     ADC 12V Value L
############################################################################################################################################################################################################################################################


RECV_BUFFER = 16
HOST = "192.168.4.1"
PORT = 5000
  
dataIn=[]
dataOut = [0x1C, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
TCP_server_socket=None
TCP_connection=None
TCP_client_address=None

#Servo default 
servo = None
adc = None

Arm_Port1 = 12
Arm_Port2 = 13
Arm_Port3 = 14
Arm_Port4 = 15

Cam_Horizontal_Port = 0
Cam_Vertical_Port = 1

Left_Lamp_Port = 2
Right_Lamp_Port = 3

Init_Arm_Pos0 = 1500    #set 0 degree on the servo  
Init_Arm_Pos1 = 1500 
Init_Arm_Pos2 = 1500 
Init_Arm_Pos3 = 1500

Init_Cam_Horizontal_Pos = 1500
Init_Cam_Vertical_Pos = 500

GPIO_TRIGGER_PIN = 18
GPIO_ECHO_PIN = 24

def initServoControl():
    global servo
    global Arm_Port1
    global Arm_Port2
    global Arm_Port3
    global Arm_Port4
    global Cam_Horizontal_Port
    global Cam_Vertical_Port
    global Init_Arm_Pos0
    global Init_Arm_Pos1
    global Init_Arm_Pos2
    global Init_Arm_Pos3
    global Init_Cam_Horizontal_Pos
    global Init_Cam_Vertical_Pos

    servo = PCA9685(0x40)
    servo.setPWMFreq(50)

    servo.setServoPulse(Arm_Port1, Init_Arm_Pos0)
    servo.setServoPulse(Arm_Port2, Init_Arm_Pos1)
    servo.setServoPulse(Arm_Port3, Init_Arm_Pos2)
    servo.setServoPulse(Arm_Port4, Init_Arm_Pos3)

    servo.setServoPulse(Cam_Horizontal_Port, Init_Cam_Horizontal_Pos)
    servo.setServoPulse(Cam_Vertical_Port, Init_Cam_Vertical_Pos)

    servo.setPWM(Left_Lamp_Port, 0, 0)
    servo.setPWM(Right_Lamp_Port, 0, 0)

def initServerSocket():
    global TCP_server_socket
    global HOST
    global PORT
    
    TCP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    TCP_server_socket.bind((HOST, PORT))
    TCP_server_socket.listen(1)

def initMotorsPins():
    global p1
    global p2
    global MA1
    global MA2
    global MB1
    global MB2
    global motorPWM1
    global motorPWM2
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)    
    GPIO.setup(MA1,GPIO.OUT)
    GPIO.setup(MA2,GPIO.OUT)
    GPIO.setup(MB1,GPIO.OUT)
    GPIO.setup(MB2,GPIO.OUT)
    GPIO.setup(motorPWM1Pin,GPIO.OUT)
    GPIO.setup(motorPWM2Pin,GPIO.OUT)
    p1 = GPIO.PWM(motorPWM1Pin,500)
    p2 = GPIO.PWM(motorPWM2Pin,500)
    stop()
    p1.start(50)
    p2.start(50)

#todo finish it!!!!!!!
def initADC():
    pass
    #global adc

    #int_temp = ADS1118.encode(single_shot=True, temp_sensor=True, data_rate=5) # internal temperature

    #adc = ADS1118.ADS1118(SCLK=17, DIN=18, DOUT=27) # set the GPIO pins
    

def adcControll():
    dataOut[12] = 0x64
    dataOut[13] = 0x65
    dataOut[14] = 0x66
    dataOut[15] = 0x67

def initSR04():
    global GPIO_ECHO_PIN
    global GPIO_TRIGGER_PIN

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_TRIGGER_PIN, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_PIN, GPIO.IN)

def distanceControll():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2 
    dataOut[9] = byte(distance)
     

def set_motor(A1,A2,B1,B2):
    GPIO.output(MA1,A1)
    GPIO.output(MA2,A2)
    GPIO.output(MB1,B1)
    GPIO.output(MB2,B2)

def reverse():
    set_motor(1,0,1,0)    

def stop():
    set_motor(0,0,0,0)

def forward():
    set_motor(0,1,0,1)

def left():
    set_motor(1,0,0,0)
def right():
    set_motor(0,0,1,0)
    
def motorControll():
    global actualLeftMotorPWM
    global actualRightMotorPWM
    global p1
    global p2
    global dataIn    
    global dataOut

    if dataIn[0]==0x18:
        forward()        
        print("Forward")
    if dataIn[0]==0x08:
        left()        
        print("Left")
    if dataIn[0]==0x1c:
        stop()
        dataOut[0] = 0x1c
        dataOut[1] = 0x00
        dataOut[2] = 0x00
        actualLeftMotorPWM = 0
        actualRightMotorPWM = 0
        print("Stop")
    if dataIn[0]==0x5a:
        right()        
        print("Right")
    if dataIn[0]==0x52:
        reverse()        
        print("Reverse")
        
    if (dataIn[1] < 101 and dataIn[1] >= 0 and dataIn[0] != 0x1c):
        actualLeftMotorPWM = dataIn[1]
    if (dataIn[2] < 101 and dataIn[2] >= 0  and dataIn[0] != 0x1c):
        actualRightMotorPWM = dataIn[2]
            
    p1.ChangeDutyCycle(actualLeftMotorPWM)
    p2.ChangeDutyCycle(actualRightMotorPWM)

    dataOut[0] = dataIn[0]
    dataOut[1] = dataIn[1]
    dataOut[2] = dataIn[2]

def servoControll():
    global servo
    global Arm_Port1
    global Arm_Port2
    global Arm_Port3
    global Arm_Port4
    global Cam_Horizontal_Port
    global Cam_Vertical_Port

    global dataIn
    global dataOut
    
    dataOut[3] = dataIn[3]
    dataOut[4] = dataIn[4]
    dataOut[5] = dataIn[5]
    dataOut[6] = dataIn[6]
    dataOut[7] = dataIn[7]
    dataOut[8] = dataIn[8]

    #convert to raw value rawPos0
    rawArmPos0 = int((dataIn[3] * 11.11) + 500)
    rawArmPos1 = int((dataIn[4] * 11.11) + 500)
    rawArmPos2 = int((dataIn[5] * 11.11) + 500)
    rawArmPos3 = int((dataIn[6] * 11.11) + 500)

    rawCamHorPos = int((dataIn[7] * 11.11) + 500)

    rawCamVerPos = int((dataIn[8] * 11.11) + 500)

    servo.setServoPulse(Arm_Port1,rawArmPos0)
    servo.setServoPulse(Arm_Port2,rawArmPos1)
    servo.setServoPulse(Arm_Port3,rawArmPos2)
    servo.setServoPulse(Arm_Port4,rawArmPos3)

    servo.setServoPulse(Cam_Horizontal_Port, rawCamHorPos)
    servo.setServoPulse(Cam_Vertical_Port, rawCamVerPos)

def lampControll():
    global servo
    global Left_Lamp_Port
    global Right_Lamp_Port
 
    dataOut[10] = dataIn[10]
    dataOut[11] = dataIn[11]

    leftLamp = int(40.95 * dataIn[10])
    rightLamp = int(40.95 * dataIn[11])

    servo.setPWM(Left_Lamp_Port, leftLamp, 0)
    servo.setPWM(Right_Lamp_Port, rightLamp, 0)



def main():
    global RECV_BUFFER
    global TCP_server_socket   
    global TCP_client_address
    global TCP_connection 
    global dataIn
    global dataOut    
        
    print("Robot Server started...")
    print("Initialization Motor pins")
    initMotorsPins()
    print("Initialization Server Socket")
    initServerSocket()    
    print("Initialization Servo pins")
    initServoControl()
    print("Initialization ADC")
    initADC()
    print("Initialization HC-SR04")
    initSR04()
    
    #infinity cycle :(
    while True:
        try:        
            print("Waiting for client...")
            TCP_connection, TCP_client_address = TCP_server_socket.accept()
            print("Connection from ", TCP_client_address)
            while True:            
                dataIn=TCP_connection.recv(RECV_BUFFER)
                if not dataIn:
                    TCP_server_socket.close()
                    break
                else:                    
                    print("Run motor controll...")
                    motorControll() 
                    print("Run servo controll...")          
                    servoControll()    
                    print("Run lamp controll...")
                    lampControll()                    
                    if dataIn[9] == 0xFF:
                        print("Distance controll...")
                        distanceControll()
                    print("Run ADC controll...")
                    adcControll()                        
                    print("Send data to the client...")
                    TCP_connection.sendall(bytes(dataOut))

        except KeyboardInterrupt:
            GPIO.cleanup()
            TCP_server_socket.close()

        finally:
            dataIn=[0x1C, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            motorControll()
            servoControll()
            lampControll()


if __name__ == "__main__":    
    main()
    GPIO.cleanup()
    TCP_server_socket.close()
