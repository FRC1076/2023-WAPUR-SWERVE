import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
import ctre

class Elevator:
    def __init__(self, config):
        #self.logger = Logger.getLogger()
        
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(config["RIGHT_MOTOR_ID"], motor_type) # elevator up-down
        self.left_motor = rev.CANSparkMax(config["LEFT_MOTOR_ID"], motor_type) # elevator up-down