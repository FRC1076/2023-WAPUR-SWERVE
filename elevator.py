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
        motorType = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.rightMotor = rev.CANSparkMax(config["RIGHT_MOTOR_ID"], motorType) # elevator up-down
        self.leftMotor = rev.CANSparkMax(config["LEFT_MOTOR_ID"], motorType) # elevator up-down

        self.rightEncoder = self.rightMotor.getEncoder() # measure elevator height
        self.leftEncoder = self.leftMotor.getEncoder() # ""
        self.rightEncoder.setPosition(0)
        self.leftEncoder.setPosition(0)

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
            print(f"shelf-A [{targetHeight}]")
        elif shelfLabel == "B":
             targetHeight = self.shelfHeightB
             print(f"shelf-B [{targetHeight}]")
        elif shelfLabel == "C":
             targetHeight = self.shelfHeightC
             print(f"shelf-C [{targetHeight}]")
        else:
            targetHeight = self.shelfHeightD  
            print(f"shelf-D [{targetHeight}]")

    def manualRaise(self):
        self.rightMotor.set(1)
        self.leftMotor.set(1)
        #print(f"Raisingelevator [{self.getEncoderPosition()}]")
        return
    
    def manualLower(self):
        self.rightMotor.set(-1)
        self.leftMotor.set(-1)
        #print(f"Lowering elevator [{self.getEncoderPosition()}]")
        
        return
    
    # Move elevator and reset target to where you end up.
    def move(self, targetSpeed):
        self.extend(targetSpeed)
        self.targetPosition = self.getEncoderPosition()
    
    def resetEncoders(self):
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)
        self.targetPosition = self.getEncoderPosition()

    def getEncoderPosition(self):
        return self.rightEncoder.getPosition()
    
     #def log(self, *dataToLog):
        #self.logger.log(DASH_PREFIX, dataToLog)
