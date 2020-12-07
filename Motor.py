#!/usr/bin/python3

import RPi.GPIO as GPIO
import threading
import time

class Motor:
    def __init__(self, MA1 = 20, MA2 = 21, MB1 = 6, MB2 = 13, motorPWM1Pin = 26, motorPWM2Pin = 12):
        self.MA1 = MA1
        self.MA2 = MA2
        self.MB1 = MB1
        self.MB2 = MB2
        self.motorPWM1Pin = motorPWM1Pin
        self.motorPWM2Pin = motorPWM2Pin
        self.leftMotorPWMController = None
        self.rightMotorPWMController = None
        self.actualLeftMotorPWM = 0
        self.actualRightMotorPWM = 0
        
    def InitMotorHw(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)    
        GPIO.setup(self.MA1,GPIO.OUT)
        GPIO.setup(self.MA2,GPIO.OUT)
        GPIO.setup(self.MB1,GPIO.OUT)
        GPIO.setup(self.MB2,GPIO.OUT)
        GPIO.setup(self.motorPWM1Pin,GPIO.OUT)
        GPIO.setup(self.motorPWM2Pin,GPIO.OUT)
        self.leftMotorPWMController = GPIO.PWM(self.motorPWM1Pin,500)
        self.rightMotorPWMController = GPIO.PWM(self.motorPWM2Pin,500)
        self.stop()
        self.leftMotorPWMController.start(50)
        self.rightMotorPWMController.start(50)

    def set_motor(self,A1,A2,B1,B2):
        GPIO.output(self.MA1,A1)
        GPIO.output(self.MA2,A2)
        GPIO.output(self.MB1,B1)
        GPIO.output(self.MB2,B2)

    def reverse(self):
        self.set_motor(1,0,1,0)    

    def stop(self):
        self.set_motor(0,0,0,0)
        self.actualLeftMotorPWM = 0
        self.actualRightMotorPWM = 0

    def forward(self):
        self.set_motor(0,1,0,1)

    def setLeftMotorPWM(self, leftPWM):
        self.actualLeftMotorPWM = leftPWM
        self.leftMotorPWMController.ChangeDutyCycle(self.actualLeftMotorPWM)

    def setRightMotorPWM(self, rightPWM):
        self.actualRightMotorPWM = rightPWM
        self.rightMotorPWMController.ChangeDutyCycle(self.actualRightMotorPWM)
    
    #Optional function - not nessesery because of it can be implemented by pwm controls of the motors
    def left(self):
        self.set_motor(1,0,0,0)
    
    #Optional function - not nessesery because of it can be implemented by pwm controls of the motors
    def right(self):
        self.set_motor(0,0,1,0)
    
        
   
    
