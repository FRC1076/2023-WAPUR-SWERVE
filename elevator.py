import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
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
        self.lowerSafety = config['LOWER_SAFETY']
        self.upperSafety = config['UPPER_SAFETY']
        kP = config['KP'] 
        kI = config['KI']
        kD = config['KD']
        
        #self.logger = Logger.getLogger()
        motorType = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.rightMotor = rev.CANSparkMax(config["RIGHT_MOTOR_ID"], motorType) # elevator up-down
        self.leftMotor = rev.CANSparkMax(config["LEFT_MOTOR_ID"], motorType) # elevator up-down

        self.pidController = PIDController(kP, kI, kD)
        self.pidController.setTolerance(0.3, 0.01)

        self.rightEncoder = self.rightMotor.getEncoder() # measure elevator height
        self.leftEncoder = self.leftMotor.getEncoder() # ""
        self.rightEncoder.setPosition(0)
        self.leftEncoder.setPosition(0)

    def extend(self, targetSpeed):  # controls length of the elevator 
            
        if targetSpeed > 1:
            targetSpeed = 1
        if targetSpeed < -1:
            targetSpeed = -1

        #make sure arm doesn't go past limit
        if self.getEncoderPosition() > self.upperSafety and targetSpeed < 0:
            self.rightMotor.set(0)
            self.leftMotor.set(0)
            return
        if self.getEncoderPosition() < self.lowerSafety and targetSpeed > 0:
            self.rightMotor.set(0)
            self.leftMotor.set(0)
            return
        
        # the motors are running backwards, invert targetSpeed.
        self.rightMotor.set(-targetSpeed)
        self.leftMotor.set(-targetSpeed)

        return
    
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

        extendSpeed = self.pidController.calculate(self.getEncoderPosition(), targetHeight)
        slowedExtendSpeed = extendSpeed * 0.1125
        print("Elevator: moveToPos: ", self.pidController.getSetpoint(), " actual position: ", self.getEncoderPosition(),"Extend speed:", slowedExtendSpeed)
        self.extend(slowedExtendSpeed)
    
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
