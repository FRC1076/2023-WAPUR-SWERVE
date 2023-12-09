import math
import time
import sys
import os
from datetime import datetime

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
import ctre
from navx import AHRS
from networktables import NetworkTables

from robotconfig import robotconfig, MODULE_NAMES
from controller import Controller
from swervedrive import SwerveDrive
"""
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig

from swervedrive import BalanceConfig
from swervedrive import TargetConfig
from swervedrive import BearingConfig
from swervedrive import VisionDriveConfig

from swervometer import FieldConfig
from swervometer import RobotPropertyConfig
from swervometer import Swervometer

from elevator import Elevator
from grabber import Grabber
from vision import Vision
from logger import Logger
from dashboard import Dashboard

from tester import Tester
"""

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        controllers = self.initControllers(robotconfig["CONTROLLERS"])
        self.driver = controllers[0]
        self.operator = controllers[1]

        self.drivetrain = self.initDrivetrain(robotconfig["DRIVETRAIN"])
    
    def initLogger(self, dir):
        return #Logger.getLogger(dir)
    
    def initControllers(self, config):
        ctrls = []
        self.log(config)
        for ctrlConfig in config.values():
            self.log(ctrlConfig)
            ctrlID = ctrlConfig['ID']
            ctrl = wpilib.XboxController(ctrlID)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls.append(Controller(ctrl, dz, lta, rta))
        return ctrls
    
    def initVision(self, config):
        return
    
    def initElevator(self, config):
        return
    
    def initGrabber(self, config):
        return
    
    def initDrivetrain(self, config):
        gyro = AHRS.create_spi(wpilib._wpilib.SPI.Port.kMXP, 500000, 50)
        return SwerveDrive(config, gyro)
    
    def initAuton(self, config):
        return
    
    def initTeleop(self):
        return

    def robotPeriodic(self):
        return True
    
    def teleopPeriodic(self):
        self.teleopDrivetrain()
        return
    
    def teleopDrivetrain(self):

        #Get the joystick inputs for swerve
        fwd = self.deadzoneCorrection(self.driver.xboxController.getLeftY(), self.driver.deadzone)
        strafe = self.deadzoneCorrection(self.driver.xboxController.getLeftX(), self.driver.deadzone)
        rcw = self.deadzoneCorrection(self.driver.xboxController.getRightX(), self.driver.deadzone)

        """controller_at_180_to_bot = -1
        fwd *= controller_at_180_to_bot
        strafe *= controller_at_180_to_bot"""
        print(fwd, strafe, rcw)
        #Call move, which will move the swerve drive
        if fwd != 0 or strafe != 0 or rcw != 0:
            self.drivetrain.move(-fwd, strafe, rcw, self.drivetrain.getBearing())
        #move will automatically call execute
        #self.drivetrain.execute()
        
        return
        
    
    def teleopElevatorGrabber(self):
        return
    
    def autonomousInit(self): #this or initAuton?
        return
    
    def autonomousPeriodic(self):
        return
    
    def teleopManeuver(self):
        return
    
    def deadzoneCorrection(self, val, deadzone):
        """
        Given the deadzone value x, the deadzone both eliminates all
        values between -x and x, and scales the remaining values from
        -1 to 1, to (-1 + x) to (1 - x)
        """
        if abs(val) < deadzone:
            return 0
        elif val < 0:
            x = (abs(val) - deadzone) / (1 - deadzone)
            return -x
        else:
            x = (val - deadzone) / (1 - deadzone)
            return x


    def log(self, *dataToLog):
        return

if __name__ == "__main__":
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)