import wpilib
import wpilib.drive
import wpimath.controller

import rev


#from logger import Logger

class Elevator:
    def __init__(self, config):
        #self.currentHeight = config["currentHeight"]
        #self.kP = config["kP"]
        #self.kI = config["kI"]
        #self.kD = config["kD"]
        self.shelfHeightA = config["SHELF_HEIGHT_A"] # Lowest shelf
        self.shelfHeightB = config["SHELF_HEIGHT_B"]
        self.shelfHeightC = config["SHELF_HEIGHT_C"]
        self.shelfHeightD = config["SHELF_HEIGHT_D"]
        #self.logger = Logger.getLogger()
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(config["RIGHT_MOTOR_ID"], motor_type) # elevator up-down
        self.left_motor = rev.CANSparkMax(config["LEFT_MOTOR_ID"], motor_type) # elevator up-down

        self.right_encoder = self.right_motor.getEncoder() # measure elevator height
        self.left_encoder = self.left_motor.getEncoder() # ""
        self.right_encoder.setPosition(0)
        self.left_encoder.setPosition(0)

    def extend(self, targetSpeed):  # controls length of the elevator 
            
        if targetSpeed > 1:
            targetSpeed = 1
        if targetSpeed < -1:
            targetSpeed = -1
        
        if targetSpeed > 0:
            targetSpeed *= 0.5

        return targetSpeed
    def moveToHeight(self, shelfLabel):
        targetHeight = 0
        if shelfLabel == "A":
            targetHeight = self.shelfHeightA
        elif shelfLabel == "B":
             targetHeight = self.shelfHeightB
        elif shelfLabel == "C":
             targetHeight = self.shelfHeightC
        else:
            targetHeight = self.shelfHeightD  

    def manualRaise(self):
        self.right_motor.set(1)
        self.left_motor.set(1)
        return
    
    def manualLower(self):
        self.right_motor.set(-1)
        self.left_motor.set(-1)
        return
    
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
