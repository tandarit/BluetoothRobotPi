#!/usr/bin/python3

import RPi.GPIO as GPIO
from PCA9685 import PCA9685

class Servo:
    def __init__(self, I2C_Address = 0x40, pwmFreq = 50, Arm_Port1 = 12, Arm_Port2 = 13, Arm_Port3 = 14, Arm_Port4 = 15, Cam_Horizontal_Port = 0, Cam_Vertical_Port = 1):
        self.I2C_Address = I2C_Address
        self. pwmFreq =pwmFreq
        self.Arm_Port1 = Arm_Port1
        self.Arm_Port2 = Arm_Port2
        self.Arm_Port3 = Arm_Port3
        self.Arm_Port4 = Arm_Port4
        self.Cam_Horizontal_Port = Cam_Horizontal_Port
        self.Cam_Vertical_Port = Cam_Vertical_Port
        
        self.rawArmPos0 = 1500
        self.rawArmPos1 = 1500
        self.rawArmPos2 = 1500
        self.rawArmPos3 = 1500

        self.rawCamHorPos = 1500
        self.rawCamVerPos = 500

    def InitServoHw(self):
        self.servo = PCA9685(self.I2C_Address)
        self.servo.setPWMFreq(self.pwmFreq)

        self.servo.setServoPulse(self.Arm_Port1, self.rawArmPos0)
        self.servo.setServoPulse(self.Arm_Port2, self.rawArmPos1)
        self.servo.setServoPulse(self.Arm_Port3, self.rawArmPos2)
        self.servo.setServoPulse(self.Arm_Port4, self.rawArmPos3)

        self.servo.setServoPulse(self.Cam_Horizontal_Port, self.rawCamHorPos)
        self.servo.setServoPulse(self.Cam_Vertical_Port, self.rawCamVerPos)

        #self.servo.setPWM(Left_Lamp_Port, 0, 0)
        #self.servo.setPWM(Right_Lamp_Port, 0, 0)

    def getArm0Position(self):
        return int((self.rawArmPos0 - 500) / 11.11)

    def setArm0Position(self, value):
        #check value 0-180 just for safety
        if value>=0 and value <= 90:
            self.rawArmPos0 = int((value * 11.11) + 500)
            self.servo.setServoPulse(self.Arm_Port1, self.rawArmPos0)

    def getArm1Position(self):
        return int((self.rawArmPos1 - 500) / 11.11)

    def setArm1Position(self, value):
        #check value 0-180 just for safety
        if value>=0 and value <= 180:
            self.rawArmPos1 = int((value * 11.11) + 500)
            self.servo.setServoPulse(self.Arm_Port2, self.rawArmPos1)

    def getArm2Position(self):
        return int((self.rawArmPos2 - 500) / 11.11)

    def setArm2Position(self, value):
        #check value 0-180 just for safety
        if value>=0 and value <= 180:
            self.rawArmPos2 = int((value * 11.11) + 500)
            self.servo.setServoPulse(self.Arm_Port3, self.rawArmPos2)

    def getArm3Position(self):
        return int((self.rawArmPos3 - 500) / 11.11)

    def setArm3Position(self, value):
        #check value 0-180 just for safety
        if value>=90 and value <= 165:
            self.rawArmPos3 = int((value * 11.11) + 500)
            self.servo.setServoPulse(self.Arm_Port4, self.rawArmPos3)

    def getCamHorPosition(self):
        return int((self.rawCamHorPos - 500) / 11.11)

    def setCamHorPosition(self, value):
        #check value 0-180 just for safety
        if value>=0 and value <= 165:
            self.rawCamHorPos = int((value * 11.11) + 500)
            self.servo.setServoPulse(self.Cam_Horizontal_Port, self.rawCamHorPos)
    
    def getCamVerPosition(self):
        return int((self.rawCamVerPos - 500) / 11.11)

    def setCamVerPosition(self, value):
        #check value 0-180 just for safety
        if value>=0 and value <= 180:
            self.rawCamVerPos = int((value * 11.11) + 500)
            self.servo.setServoPulse(self.Cam_Vertical_Port, self.rawCamVerPos)
