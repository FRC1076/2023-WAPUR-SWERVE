import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
import ctre

from logger import Logger

class Elevator:
    def __init__(self, currentHeight, targetPosition, kP, kI, kD):
        self.logger = Logger.getLogger()
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(config["RIGHT_MOTOR_ID"], motor_type) # elevator up-down
        self.left_motor = rev.CANSparkMax(config["LEFT_MOTOR_ID"], motor_type) # elevator up-down

        self.right_encoder = self.right_motor.getEncoder() # measure elevator height
        self.left_encoder = self.left_motor.getEncoder() # ""
        self.right_encoder.setPosition(0)
        self.left_encoder.setPosition(0)

    def setHeight(self,height):
        #4 different heights for shelves, change values to actual shelf measurements
        rackOne = 0
        rackTwo = 0
        rackThree = 0
        rackFour = 0

        #get button input and select specific shelf height
        if buttonInput()=="X":
            self.height = rackOne
        elif buttonInput()=="Y":
            self.height = rackTwo
        elif buttonInput()=="A":
            self.height = rackThree
        elif buttonInput()=="B":
            self.height = rackFour
        return self.height

    def update(self):
        self.moveToPos(self.targetPosition)
        return
    
    def extend(self, targetSpeed):  # controls length of the elevator 
            
        if targetSpeed > 1:
            targetSpeed = 1
        if targetSpeed < -1:
            targetSpeed = -1
        
        if targetSpeed > 0:
            targetSpeed *= 0.5

        return targetSpeed
    
    def manualRaise(self):
        self.right_motor.set(1)
        self.left_motor.set(1)
        return
    
    def manualLower(self):
        self.right_motor.set(-1)
        self.left_motor.set(-1)
        return
    
    def motors_off(self):
        self.right_motor.set(0)
        self.left_motor.set(0)

    # Move elevator and reset target to where you end up.
    def move(self, targetSpeed):
        self.extend(targetSpeed)
        self.targetPosition = self.getEncoderPosition()
    
    def resetEncoders(self):
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)
        self.targetPosition = self.getEncoderPosition()

    def getEncoderPosition(self):
        return self.right_encoder.getPosition()
    
     #def log(self, *dataToLog):
        #self.logger.log(DASH_PREFIX, dataToLog)
