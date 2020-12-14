#!/usr/bin/python3

from pyPS4Controller.controller import Controller, Event
from Motor import Motor
from Servo import Servo
import RPi.GPIO as GPIO
import math
import time
from datetime import datetime

class MyPS4Controller(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.L3X = 0
        self.L3Y = 0
        self.R3X = 0
        self.R3Y = 0
        self.leftMotorPWM = 0
        self.rightMotorPWM = 0
        self.arrow = "None"
        self.motor = Motor()
        self.servo = Servo()
        self.motor.InitMotorHw()
        self.servo.InitServoHw()
        self.BASESTEPSIZE = 5      
       
    def on_L3_up(self, value):
        #print("L3 up értéke: {}".format(value))
        self.L3Y = value
        self.motorControll()

    def on_L3_down(self, value):
        #print("L3 down értéke: {}".format(value))
        self.L3Y = value
        self.motorControll()

    def on_L3_left(self, value):
        #print("L3 left értéke: {}".format(value))
        self.L3X = value
        self.motorControll()

    def on_L3_right(self, value):
        #print("L3 right értéke: {}".format(value))
        self.L3X = value
        self.motorControll()
        
    def on_L3_y_at_rest(self):   
        self.L3Y = 0
        self.motorControll()

    def on_L3_x_at_rest(self):
        self.L3X = 0
        self.motorControll()

    def motorControll(self):
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

        #left case
        if self.L3Y == 0 and self.L3X < 0:
            self.rightMotorPWM = int((abs(self.L3X) / 32767) * 100)
            self.leftMotorPWM = 0
            self.motor.setLeftMotorPWM(self.leftMotorPWM)
            self.motor.setRightMotorPWM(self.rightMotorPWM)

        #right case
        if self.L3Y == 0 and self.L3X > 0:
            self.leftMotorPWM = int((abs(self.L3X) / 32767) * 100)
            self.rightMotorPWM = 0
            self.motor.setLeftMotorPWM(self.leftMotorPWM)
            self.motor.setRightMotorPWM(self.rightMotorPWM)


        #first quater
        if self.L3Y < 0 and self.L3X < 0:           
            baseSpeedVector = math.sqrt((self.L3X * self.L3X) + (self.L3Y * self.L3Y))
            rightMotor = baseSpeedVector * (abs(self.L3Y) / 32767)
            leftMotor = rightMotor * ((32767 - abs(self.L3Y)) / 32768)
            
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
            leftMotor = baseSpeedVector * (abs(self.L3Y) / 32767)
            rightMotor = leftMotor * ((32767 - abs(self.L3Y)) / 32768)
            
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
            rightMotor = baseSpeedVector * (abs(self.L3Y) / 32767)
            leftMotor = rightMotor * ((32767 - abs(self.L3Y)) / 32768)
            
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
            leftMotor = baseSpeedVector * (abs(self.L3Y) / 32767)
            rightMotor = leftMotor * ((32767 - abs(self.L3Y)) / 32768)
            
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

    def on_up_arrow_press(self):
        #print("Up lenyomva")
        self.arrow = "Up"
        self.servoControll()

    def on_up_down_arrow_release(self):
        self.arrow = "None"
        self.servoControll()

    def on_down_arrow_press(self):
        #print("Down lenyomva")
        self.arrow = "Down"
        self.servoControll()

    def on_left_arrow_press(self):
        #print("Left lenyomva")
        self.arrow = "Left"
        self.servoControll()

    def on_left_right_arrow_release(self):
        self.arrow = "None"
        self.servoControll()

    def on_right_arrow_press(self):
        #print("Right lenyomva")
        self.arrow = "Right"
        self.servoControll()

    def on_R3_up(self, value):
        #print("R3 up értéke: {}".format(value))
        self.R3Y = value        
        self.servoControll()

    def on_R3_down(self, value):
        #print("R3 down értéke: {}".format(value))
        self.R3Y = value
        self.servoControll()

    def on_R3_left(self, value):
        #print("R3 left értéke: {}".format(value))
        self.R3X = value
        self.servoControll()

    def on_R3_right(self, value):
        #print("R3 right értéke: {}".format(value))
        self.R3X = value
        self.servoControll()
        
    def on_R3_y_at_rest(self):   
        self.R3Y = 0
        self.servoControll()

    def on_R3_x_at_rest(self):
        self.R3X = 0
        self.servoControll()


    def servoControll(self):
        #arm controll condition 
        if self.arrow != "None" :
            self.ArmServoControl()
        else:
            self.CameraServoControl()

    def ArmServoControl(self):
        if self.arrow == "Down":
            oldPosition = self.servo.getArm0Position()
            if self.R3Y > 0:                
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition + stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm0Position(newPosition)
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now))
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm0Position()))
                    """
            if self.R3Y < 0:
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition - stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm0Position(newPosition)
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now)) 
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm0Position()))
                    """
        if self.arrow == "Right":
            oldPosition = self.servo.getArm1Position()
            if self.R3Y > 0:                
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition + stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm1Position(newPosition)
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now))
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm1Position()))
                    """
            if self.R3Y < 0:
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition - stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm1Position(newPosition)
                    """                    
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now)) 
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm1Position()))
                    """
        if self.arrow == "Up":
            oldPosition = self.servo.getArm2Position()
            if self.R3Y > 0:                
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition + stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm2Position(newPosition)
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now))
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm2Position()))
                    """
            if self.R3Y < 0:
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition - stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm2Position(newPosition)                    
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now)) 
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm2Position()))
                    """
        if self.arrow == "Left":
            oldPosition = self.servo.getArm3Position()
            if self.R3Y > 0:                
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition + stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm3Position(newPosition)
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now))
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm3Position()))
                    """
            if self.R3Y < 0:
                stepSize = int(abs(self.R3Y) / (32767 / 5)) # 1 - 7 the range
                newPosition = oldPosition - stepSize
                if (oldPosition != newPosition) and (abs(oldPosition - newPosition) <= 3):
                    self.servo.setArm3Position(newPosition)
                    """
                    now = datetime.now()
                    print(stepSize)
                    print("Time: {}".format(now)) 
                    print("old Position: {}".format(oldPosition))
                    print("new position: {}".format(self.servo.getArm3Position()))
                    """
    def CameraServoControl(self):
        #ToDo I have to implemention
        pass


    
        

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
    
    def R3_event(self):
        return self.button_type == 2 and self.button_id in [4, 3]
        
    def R3_y_at_rest(self):
        return self.button_id in [4] and self.value == 0
        
    def R3_x_at_rest(self):
        return self.button_id in [3] and self.value == 0
        
    def R3_up(self):
        return self.button_id == 4 and self.value < 0
        
    def R3_down(self):
        return self.button_id == 4 and self.value > 0
        
    def R3_left(self):
        return self.button_id == 3 and self.value < 0
        
    def R3_right(self):
        return self.button_id == 3 and self.value > 0
    """            
    def up_arrow_pressed(self):
        return self.button_id == 7 and self.button_type == 2 and self.value == 1
        

    def down_arrow_pressed(self):
        return self.button_id == 7 and self.button_type == 2 and self.value == 1
        

    def up_down_arrow_released(self):
        return self.button_id == 7 and self.button_type == 2 and self.value == 0
        

    # left / right arrows #
    def left_arrow_pressed(self):
        return self.button_id == 6 and self.button_type == 2 and self.value == 1

    def right_arrow_pressed(self):
        return self.button_id == 6 and self.button_type == 2 and self.value == 1
    

    def left_right_arrow_released(self):
        return self.button_id == 6 and self.button_type == 2 and self.value == 0
        
"""