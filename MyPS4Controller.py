#!/usr/bin/python3

from pyPS4Controller.controller import Controller, Event
from Motor import Motor
import RPi.GPIO as GPIO
import math

class MyPS4Controller(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.L3X = 0
        self.L3Y = 0
        self.leftMotorPWM = 0
        self.rightMotorPWM = 0
        self.motor = Motor()
        self.motor.InitMotorHw()
        
    def on_x_press(self):
       print("Hello world")

    def on_x_release(self):
       print("Goodbye world")
       
    def on_L3_up(self, value):
        print("L3 up értéke: {}".format(value))
        self.L3Y = value
        self.controll()

    def on_L3_down(self, value):
        print("L3 down értéke: {}".format(value))
        self.L3Y = value
        self.controll()

    def on_L3_left(self, value):
        print("L3 left értéke: {}".format(value))
        self.L3X = value
        self.controll()

    def on_L3_right(self, value):
        print("L3 right értéke: {}".format(value))
        self.L3X = value
        self.controll()
        
    def on_L3_y_at_rest(self):   
        self.L3Y = 0
        self.controll()

    def on_L3_x_at_rest(self):
        self.L3X = 0
        self.controll()

    def controll(self):
        #stop case
        if self.L3Y == 0 and self.L3X == 0:
            self.motor.stop()

        #forward case
        if self.L3Y < 0 and self.L3X == 0:
            self.leftMotorPWM = self.rightMotorPWM = int((abs(self.L3Y) / 32767) * 100)
            self.motor.forward()
            self.motor.setLeftMotorPWM(self.leftMotorPWM)
            self.motor.setRightMotorPWM(self.rightMotorPWM)

        #backward case
        if self.L3Y > 0 and self.L3X == 0:
            self.leftMotorPWM = self.rightMotorPWM = int((abs(self.L3Y) / 32767) * 100)
            self.motor.reverse()
            self.motor.setLeftMotorPWM(self.leftMotorPWM)
            self.motor.setRightMotorPWM(self.rightMotorPWM)       


        #first quater
        if self.L3Y < 0 and self.L3X < 0:           
            baseSpeedVector = math.sqrt((self.L3X * self.L3X) + (self.L3Y * self.L3Y))
            rightMotor = baseSpeedVector * (abs(self.L3X) / 32767)
            leftMotor = rightMotor * ((32767 - abs(self.L3X)) / 32768)
            
            self.leftMotorPWM = int((leftMotor / 32767) * 100)
            self.rightMotorPWM = int((rightMotor/32767) * 100)            
            
            if self.leftMotorPWM > 100 or self.rightMotorPWM > 100:
                print("----------------------------------------")
                print("DANGER IN FIRST QUATER")
                print("Left motor PWM: {}".format(self.leftMotorPWM))
                print("Right motor PWM: {}".format(self.rightMotorPWM))
                print("----------------------------------------")
            else :
                self.motor.forward()
                self.motor.setLeftMotorPWM(self.leftMotorPWM)
                self.motor.setRightMotorPWM(self.rightMotorPWM)

        #second quater
        if self.L3Y < 0 and self.L3X > 0:           
            baseSpeedVector = math.sqrt((self.L3X * self.L3X) + (self.L3Y * self.L3Y))
            leftMotor = baseSpeedVector * (abs(self.L3X) / 32767)
            rightMotor = leftMotor * ((32767 - abs(self.L3X)) / 32768)
            
            self.leftMotorPWM = int((leftMotor / 32767) * 100)
            self.rightMotorPWM = int((rightMotor/32767) * 100)            
            
            if self.leftMotorPWM > 100 or self.rightMotorPWM > 100:
                print("----------------------------------------")
                print("DANGER IN SECOND QUATER")
                print("Left motor PWM: {}".format(self.leftMotorPWM))
                print("Right motor PWM: {}".format(self.rightMotorPWM))
                print("----------------------------------------")
            else:
                self.motor.forward()
                self.motor.setLeftMotorPWM(self.leftMotorPWM)
                self.motor.setRightMotorPWM(self.rightMotorPWM)

        #third quater
        if self.L3Y > 0 and self.L3X > 0:           
            baseSpeedVector = math.sqrt((self.L3X * self.L3X) + (self.L3Y * self.L3Y))
            rightMotor = baseSpeedVector * (abs(self.L3X) / 32767)
            leftMotor = rightMotor * ((32767 - abs(self.L3X)) / 32768)
            
            self.leftMotorPWM = int((leftMotor / 32767) * 100)
            self.rightMotorPWM = int((rightMotor/32767) * 100)         
            
            if self.leftMotorPWM > 100 or self.rightMotorPWM > 100:
                print("----------------------------------------")
                print("DANGER IN THIRD QUATER")
                print("Left motor PWM: {}".format(self.leftMotorPWM))
                print("Right motor PWM: {}".format(self.rightMotorPWM))
                print("----------------------------------------")
            else :
                self.motor.reverse()
                self.motor.setLeftMotorPWM(self.leftMotorPWM)
                self.motor.setRightMotorPWM(self.rightMotorPWM)

        #fourth quater
        if self.L3Y > 0 and self.L3X < 0:           
            baseSpeedVector = math.sqrt((self.L3X * self.L3X) + (self.L3Y * self.L3Y))
            leftMotor = baseSpeedVector * (abs(self.L3X) / 32767)
            rightMotor = leftMotor * ((32767 - abs(self.L3X)) / 32768)
            
            self.leftMotorPWM = int((leftMotor / 32767) * 100)
            self.rightMotorPWM = int((rightMotor/32767) * 100)            
            
            if self.leftMotorPWM > 100 or self.rightMotorPWM > 100:
                print("----------------------------------------")
                print("DANGER IN FOURTH QUATER")
                print("Left motor PWM: {}".format(self.leftMotorPWM))
                print("Right motor PWM: {}".format(self.rightMotorPWM))
                print("----------------------------------------")
            else:            
                self.motor.reverse()
                self.motor.setLeftMotorPWM(self.leftMotorPWM)
                self.motor.setRightMotorPWM(self.rightMotorPWM)

    
        

class MyPS4EventDefinition(Event):

    def __init__(self, **kwargs):
        Event.__init__(self, **kwargs)

    def circle_pressed(self):
        return self.button_id == 1 and self.button_type == 1 and self.value == 1

    def circle_released(self):
        return self.button_id == 1 and self.button_type == 1 and self.value == 0

    def x_pressed(self):
        return self.button_id == 0 and self.button_type == 1 and self.value == 1

    def x_released(self):
        return self.button_id == 0 and self.button_type == 1 and self.value == 0

    def triangle_pressed(self):
        return self.button_id == 2 and self.button_type == 1 and self.value == 1

    def triangle_released(self):
        return self.button_id == 2 and self.button_type == 1 and self.value == 0

    def square_pressed(self):
        return self.button_id == 3 and self.button_type == 1 and self.value == 1

    def square_released(self):
        return self.button_id == 3 and self.button_type == 1 and self.value == 0
    
    def L3_y_at_rest(self):
        return self.button_id in [1] and self.value == 0

    def L3_x_at_rest(self):
        return self.button_id in [0] and self.value == 0

    def L3_up(self):
        return self.button_id == 1 and self.value < 0

    def L3_down(self):
        return self.button_id == 1 and self.value > 0

    def L3_left(self):
        return self.button_id == 0 and self.value < 0

    def L3_right(self):
        return self.button_id == 0 and self.value > 0