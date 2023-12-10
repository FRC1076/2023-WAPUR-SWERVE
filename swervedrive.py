from navx import AHRS
import math

#from magicbot import magiccomponent
import swervemodule
#from dashboard import Dashboard

#import ntcore
from networktables import NetworkTables
from networktables.util import ntproperty
from collections import namedtuple
from wpimath.controller import PIDController
#from swervometer import Swervometer
#from logger import Logger
#from robotconfig import MODULE_NAMES
from swervemodule import SwerveModule
import wpilib

#DASH_PREFIX = MODULE_NAMES.SWERVEDRIVE

class SwerveDrive:

    # Get some config options from the dashboard.
    # I'm pretty sure these don't get written anywhere else, unless via Shuffleboard
    # These are static class variables but they're accessed using self. later -- should be fixed
    # Also, it's weird to use ntproperty here when we do otherwise elsewhere
    lowerInputThresh = ntproperty('/SmartDashboard/drive/drive/lowerInputThresh', 0.001)
    rotationMultiplier = ntproperty('/SmartDashboard/drive/drive/rotationMultiplier', 0.5)
    xyMultiplier = ntproperty('/SmartDashboard/drive/drive/xyMultiplier', 0.65)
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', True) # Turn to true to run it in verbose mode.

    def __init__(self, config, gyro):
        
        ##self.logger = Logger.getLogger()
        self.frontLeftModule = SwerveModule(config["FRONT_LEFT_MODULE"])
        self.frontRightModule = SwerveModule(config["FRONT_RIGHT_MODULE"])
        self.rearLeftModule = SwerveModule(config["REAR_LEFT_MODULE"])
        self.rearRightModule = SwerveModule(config["REAR_RIGHT_MODULE"])

        # Put all the modules into a dictionary
        self.modules = {
            'front_left': self.frontLeftModule,
            'front_right': self.frontRightModule,
            'rear_left': self.rearLeftModule,
            'rear_right': self.rearRightModule
        }

        #self.swervometer = _swervometer
        #self.vision = _vision
        self.gyro = gyro
        self.gyroAngleZero = 0.0
        #assuming balanced at initialization
        #self.gyroBalanceZero = self.getGyroRoll()
        self.gyroBalanceZero = 0.0

        # Get Smart Dashboard
        #self.sd = NetworkTables.getTable('SmartDashboard')
        #self.dashboard = Dashboard.getDashboard()

        # should do this here rather than above
        # self.lowerInputThresh = ntproperty('/SmartDashboard/drive/drive/lowerInputThresh', 0.001)
        # self.rotationMultiplier = ntproperty('/SmartDashboard/drive/drive/rotationMultiplier', 0.5)
        # self.xyMultiplier = ntproperty('/SmartDashboard/drive/drive/xyMultiplier', 0.65)
        # self.debugging = ntproperty('/SmartDashboard/drive/drive/debugging', True) # Turn to true to run it in verbose mode.

        # Set all inputs to zero
        self.requestedVectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.requestedAngles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.requestedSpeeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        """self.pose_target_x = 666 # impossible values, default for logging
        self.pose_target_y = 666 
        self.pose_target_bearing = 666 """
        
        # Variables that allow enabling and disabling of features in code
        self.squaredInputs = False
        self.thresholdInputVectors = True

        #self.width = (30 / 12) / 2 # (Inch / 12 = Foot) / 2
        #self.length = (30 / 12) / 2 # (Inch / 12 = Foot) / 2

        self.wheelLock = False
        """
        self.balance_config = _balance_cfg
        self.balance_pitch_kP = self.balance_config.balance_pitch_kP
        self.balance_pitch_kI = self.balance_config.balance_pitch_kI
        self.balance_pitch_kD = self.balance_config.balance_pitch_kD

        self.balance_yaw_kP = self.balance_config.balance_yaw_kP
        self.balance_yaw_kI = self.balance_config.balance_yaw_kI
        self.balance_yaw_kD = self.balance_config.balance_yaw_kD

        self.balance_pitch_pid_controller = PIDController(self.balance_config.balance_pitch_kP, self.balance_config.balance_pitch_kI, self.balance_config.balance_pitch_kD)
        self.balance_pitch_pid_controller.enableContinuousInput(-180, 180)
        self.balance_pitch_pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.balance_yaw_pid_controller = PIDController(self.balance_config.balance_yaw_kP, self.balance_config.balance_yaw_kI, self.balance_config.balance_yaw_kD)
        self.balance_yaw_pid_controller.enableContinuousInput(0, 360)
        self.balance_yaw_pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.target_config = _target_cfg
        self.target_kP = self.target_config.target_kP
        self.target_kI = self.target_config.target_kI
        self.target_kD = self.target_config.target_kD
        
        self.target_x_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_x_pid_controller.setTolerance(0.5, 0.5)
        self.target_y_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_y_pid_controller.setTolerance(0.5, 0.5)
        # self.target_rcw_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        # self.target_rcw_pid_controller.setTolerance(0.5, 0.5)
        # self.target_rcw_pid_controller.enableContinuousInput(0, 360)
        """
        bearingkP = config["BEARING_kP"]
        bearingkI = config["BEARING_kI"]
        bearingkD = config["BEARING_kD"]
        self.bearingPIDController = PIDController(bearingkP, bearingkI, bearingkD)

        self.bearing = self.getGyroAngle()
        self.updateBearing = False

        self.frameDimensionX = config["ROBOT_SWERVE_MODULE_OFFSET_X"]
        self.frameDimensionY = config["ROBOT_SWERVE_MODULE_OFFSET_Y"]
        # TODO: 
        # - tune PID values
        """
        self.visionDrive_config = _visionDrive_cfg
        self.visionDrive_x_pid_controller = PIDController(self.visionDrive_config.x_visionDrive_kP, self.visionDrive_config.x_visionDrive_kP, self.visionDrive_config.x_visionDrive_kP)
        self.visionDrive_x_pid_controller.setTolerance(0.5, 0.5)
        self.visionDrive_y_pid_controller = PIDController(self.visionDrive_config.y_visionDrive_kP, self.visionDrive_config.y_visionDrive_kP, self.visionDrive_config.y_visionDrive_kP)
        self.visionDrive_y_pid_controller.setTolerance(0.01, 0.01)
        self.reflectiveTargetOffsetX = self.visionDrive_config.target_offsetX_reflective
        self.reflectiveTargetTargetSize = self.visionDrive_config.target_target_size_reflective
        self.aprilTargetOffsetX = self.visionDrive_config.target_offsetX_april
        self.aprilTargetTargetSize = self.visionDrive_config.target_target_size_april
        self.max_target_offset_x = self.visionDrive_config.max_target_offset_x
        self.min_target_size = self.visionDrive_config.min_target_size"""

        self.inAuton = True
        self.autonSteerStraight = config["AUTON_STEER_STRAIGHT"]
        self.teleopSteerStraight = config["TELEOP_STEER_STRAIGHT"]

    def setInAuton(self, state):
        self.inAuton = state
        return

    def shouldSteerStraight(self):
        if self.inAuton:
            return self.autonSteerStraight
        else:
            return self.teleopSteerStraight

    def getBearing(self):      
        return self.bearing

    def setBearing(self, _bearing):      
        self.bearing = _bearing
        self.updateBearing = False
    """
    def reset(self):
        #self.log("SWERVETRIVE reset")

        # Set all inputs to zero
        self.requestedVectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.requestedAngles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.requestedSpeeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }
        # Variables that allow enabling and disabling of features in code
        self.squaredInputs = False
        self.thresholdInputVectors = True

        self.wheelLock = False
        
        for key in self.modules:
            self.modules[key].reset()

        self.resetGyro()
        self.bearing = self.getGyroAngle()
        self.updateBearing = False
    """

    @staticmethod
    def square_input(input):
        return math.copysign(input * input, input) # Return magnitude of x but the sign of y. (x, y)
    
    @staticmethod
    def normalize(data):
        """
        Get the maximum value in the data. If the max is more than 1,
        divide each data by that max.
        :param data: The data to be normalized
        :returns: The normalized data
        """
        maxMagnitude = max(abs(x) for x in data)

        if maxMagnitude > 1.0:
            for i in range(len(data)):
                data[i] = data[i] / maxMagnitude
        
        return data

    @staticmethod
    def normalizeDictionary(data):
        """
        Get the maximum value in the data. If the max is more than 1,
        divide each data by that max.
        :param data: The dictionary with the data to be normalized
        :returns: The normalized dictionary with the data
        """
        maxMagnitude = max(abs(x) for x in data.values())

        if maxMagnitude > 1.0:
            for key in data:
                data[key] = data[key] / maxMagnitude
        
        return data
    
    @staticmethod
    def clamp(value : float, lower : float = -1.0, upper : float = 1.0):
        if (value > upper):
            return upper

        if (value < lower):
            return lower
        
        return value

    #angle off of gyro zero
    def getGyroAngle(self):
        angle = (self.gyro.getAngle() - self.gyroAngleZero % 360)#+ self.swervometer.getTeamGyroAdjustment()) % 360
        #print ("Gyro Adjustment", self.swervometer.getTeamGyroAdjustment())
        return angle
        
    def getGyroBalance(self):
        balance = (self.gyro.getPitch() - self.gyroBalanceZero)

        return balance

    #raw level side to side
    def getGyroPitch(self):
        pitch = self.gyro.getPitch()

        return pitch

    #raw angle
    def getGyroYaw(self):
        yaw = self.gyro.getYaw()

        return yaw

    #raw level front to back
    def getGyroRoll(self):
        roll = self.gyro.getRoll()

        return roll

    #def printGyro(self):
        #self.log("Angle: ", self.getGyroAngle(), ", Pitch: ", self.getGyroPitch(), ", Yaw: ", self.getGyroYaw(), ", Roll: ", self.getGyroRoll())

    def resetGyro(self):
        #self.log("SWERVEDRIVE resetGyro Angle: ", self.getGyroAngle(), ", Pitch: ", self.getGyroPitch(), ", Yaw: ", self.getGyroYaw(), ", Roll: ", self.getGyroRoll())
        if self.gyro:
            self.gyro.reset()
            self.bearing = self.getGyroAngle()

    """
    def flush(self):
        
        This method should be called to reset all requested values of the drive system.
        It will also flush each module individually.
        
        self.requestedVectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.requestedAngles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.requestedSpeeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        for module in self.modules.values():
            module.flush()
    """

    def set_raw_fwd(self, fwd):
        """
        Sets the raw fwd value to prevent it from being passed through any filters
        :param fwd: A value from -1 to 1
        """
        self.requestedVectors['fwd'] = fwd

    def set_raw_strafe(self, strafe):
        """
        Sets the raw strafe value to prevent it from being passed through any filters
        :param strafe: A value from -1 to 1
        """
        self.requestedVectors['strafe'] = strafe
    
    def set_raw_rcw(self, rcw):
        """
        Sets the raw rcw value to prevent it from being passed through any filters
        :param rcw: A value from -1 to 1
        """
        self.requestedVectors['rcw'] = rcw

    def set_fwd(self, fwd):
        """
        Individually sets the fwd value. (passed through filters)
        :param fwd: A value from -1 to 1
        """
        if self.squaredInputs:
            fwd = self.square_input(fwd)

        fwd *= self.xyMultiplier

        self.requestedVectors['fwd'] = fwd

    def set_strafe(self, strafe):
        """
        Individually sets the strafe value. (passed through filters)
        :param strafe: A value from -1 to 1
        """
        if self.squaredInputs:
            strafe = self.square_input(strafe)

        strafe *= self.xyMultiplier

        self.requestedVectors['strafe'] = strafe

    def set_rcw(self, rcw):
        """
        Individually sets the rcw value. (passed through filters)
        :param rcw: A value from -1 to 1
        """
        if self.squaredInputs:
            rcw = self.square_input(rcw)

        rcw *= self.rotationMultiplier

        self.requestedVectors['rcw'] = rcw
    """
    def balance(self):
        
        #self.log("Balance starting")

        #self.printGyro()

        yawSign = -1

        if(self.getGyroYaw() >= -90 and self.getGyroYaw() <= 90):
            BALANCED_YAW = 0.0
            yawSign = +1
        else:
            BALANCED_YAW = 180.0
            yawSign = -1
        BALANCED_PITCH = 0.0

        #self.log("Balance: Yaw = ", self.getGyroYaw(), " BALANCED_YAW = ", BALANCED_YAW, " BALANCED_PITCH = ", BALANCED_PITCH)
        #self.log("Balance: pitch:", self.getGyroBalance())
        pitch_error = self.balance_pitch_pid_controller.calculate(self.getGyroBalance(), BALANCED_PITCH) 
        yaw_error = self.balance_yaw_pid_controller.calculate(self.getGyroYaw(), BALANCED_YAW)
        #self.log("Balance: pitch_error:", pitch_error, ", yaw_error: ", yaw_error)

        # Set the output to 0 if at setpoint or to a value between (-1, 1)
        if self.balance_pitch_pid_controller.atSetpoint():
            pitch_output = 0
        else:
            #pitch_output = clamp(pitch_error)
            pitch_output = -pitch_error # Deliberately flipping sign

        if self.balance_yaw_pid_controller.atSetpoint():
            yaw_output = 0
        else:
            yaw_output = clamp(yaw_error)
        
        #self.log("Balance: Pitch setpoint: ", self.balance_pitch_pid_controller.getSetpoint(), "pitch output: ", pitch_output, " pitch error: ", pitch_error)
        #self.log("Balance: Yaw setpoint: ", self.balance_yaw_pid_controller.getSetpoint(), "yaw output: ", yaw_output, " yaw error: ", yaw_error)

        # Put the output to the dashboard
        #self.log('Balance pitch output', pitch_output)
        #self.log('Balance yaw output', yaw_output)
        self.move(yawSign * pitch_output, 0.0, yaw_output, self.bearing)
        
        self.update_smartdash()

        self.execute()

        if self.balance_pitch_pid_controller.atSetpoint() and self.balance_yaw_pid_controller.atSetpoint():
            #self.log("Balance: atSetpoint")
            return True
        else:
            #self.log("Balance: not atSetpoint")
            return False
    """
    def steerStraight(self, rcw, bearing):
        
        self.bearing = bearing
        current_angle = self.getGyroAngle()
        if rcw != 0:
            self.updateBearing = True
            #self.log("rcw (!=0): ", rcw, " bearing: ", self.bearing, " currentAngle: ", current_angle)
            return rcw
        else:
            self.updateBearing = False
            angle_diff = abs(current_angle - self.bearing)
            if (angle_diff) > 180:
                angle_diff = 360 - angle_diff
                if self.bearing < current_angle:
                    target_angle = current_angle + angle_diff
                else:
                    target_angle = current_angle - angle_diff
            else:
                if self.bearing < current_angle:
                    target_angle = current_angle - angle_diff
                else:
                    target_angle = current_angle + angle_diff

            rcw_error = self.bearing_pid_controller.calculate(self.getGyroAngle(), target_angle)
            #self.log("SWERVEDRIVE steerStraight rcw: ", rcw, " rcw_error: ", rcw_error, " current_angle: ", current_angle, " bearing: ", self.bearing, " target_angle: ", target_angle)
            return rcw_error

    def move(self, base_fwd, base_strafe, rcw, bearing):
        
        """
        Calulates the speed and angle for each wheel given the requested movement
        Positive fwd value = Forward robot movement\n
        Negative fwd value = Backward robot movement\n
        Positive strafe value = Left robot movement\n
        Negative strafe value = Right robot movement
        :param fwd: the requested movement in the X direction 2D plane
        :param strafe: the requested movement in the Y direction of the 2D plane
        :param rcw: the requestest magnitude of the rotational vector of a 2D plane
        """
        ##self.log("SWERVEDRIVE: MoveAdjustment: ", self.swervometer.getTeamMoveAdjustment())
        fwd = base_fwd #* self.swervometer.getTeamMoveAdjustment()
        strafe = base_strafe #* self.swervometer.getTeamMoveAdjustment()

        #self.log("SWERVEDRIVE Moving:", fwd, strafe, rcw, bearing)

        #Convert field-oriented translate to chassis-oriented translate
        
        current_angle = self.getGyroAngle() % 360
        desired_angle = (math.degrees(math.atan2(strafe, fwd))) % 360
        chassis_angle = (desired_angle - current_angle) % 360
        magnitude = self.clamp(math.hypot(strafe, fwd), 0, 1)
        
        chassis_fwd = magnitude * math.sin(math.radians(chassis_angle))
        chassis_strafe = magnitude * math.cos(math.radians(chassis_angle))

        ##self.log("modified strafe: " + str(chassis_strafe) + ", modified fwd: " + str(chassis_fwd))
        # self.dashboard.putNumber("Current Gyro Angle", self.getGyroAngle())

        self.set_fwd(chassis_fwd)
        self.set_strafe(chassis_strafe)

        # self.set_fwd(fwd)
        # self.set_strafe(strafe)
        
        #self.log("Drivetrain: Move: shouldSteerStraight:", self.shouldSteerStraight())

        if self.shouldSteerStraight():
            self.set_rcw(self.steerStraight(rcw, bearing))
        else:
            self.set_rcw(rcw)
    """
    def goToOffsetAndTargetSize(self, targetOffsetX, targetTargetSize):
        if self.vision:
            
            #self.log("goToOffsetAndTargetSize: targetOffsetX: ", targetOffsetX, " targetTargetSize: ", targetTargetSize)
            YAW = 0.0
            if(self.getGyroYaw() >= -90 and self.getGyroYaw() <= 90):
                YAW = 0.0
            else:
                YAW = 180.0

            #self.log("goToOffsetAndTargetSize: YAW: ", YAW)

            offsetX = self.vision.getTargetOffsetHorizontalReflective() 
            targetSize = self.vision.getTargetSizeReflective()

            #self.log("goToOffsetAndTargetSize: offsetX: ", offsetX, " targetSize: ", targetSize)

            if abs(offsetX) > self.max_target_offset_x or targetSize < self.min_target_size: # impossible values, there's no target
                #self.log('Aborting goToReflectiveTapeCentered() cuz no targets')
                #self.log('Target offset X: ', abs(offsetX), ", Target area: ", targetSize)
                return False

            x_error = self.visionDrive_x_pid_controller.calculate(offsetX, targetOffsetX)
            x_error = -x_error
            x_error = 0
            y_error = -self.visionDrive_y_pid_controller.calculate(targetSize, targetTargetSize)
            
            #self.log("goToOffsetAndTargetSize: x_error: ", x_error, " y_error: ", y_error)
            
            if self.visionDrive_x_pid_controller.atSetpoint() and  \
                self.visionDrive_y_pid_controller.atSetpoint():
                self.update_smartdash()
                return True
            else:
                #self.move(x_error, y_error, 0, YAW)
                self.move(x_error, y_error, 0, self.bearing)
                self.execute()
                self.update_smartdash()
                return False
    """
    """
    def goToAprilTagCentered(self):
        self.vision.setToAprilTagPipeline()
        return self.goToOffsetAndTargetSize(self.aprilTagTargetOffsetX,
                                            self.aprilTagTargetTargetSize)
    """
    """
    def goToReflectiveTapeCentered(self):
        self.vision.setToReflectivePipeline()
        return self.goToOffsetAndTargetSize(self.reflectiveTargetOffsetX,
                                            self.reflectiveTargetTargetSize)
    """
    """
    def goToBalance(self, x, y, bearing, tolerance):
        #self.log("SWERVEDRIVE Going to balance:", x, y, bearing, tolerance)

        if abs(self.getGyroBalance()) > tolerance:
            return True
        else:
            return self.goToPose(x, y, bearing)

    def goToPose(self, x, y, bearing):

        #self.log("SWERVEDRIVE Going to pose:", x, y, bearing)

        # for telemetry
        self.pose_target_x = x
        self.pose_target_y = y
        self.pose_target_bearing = bearing

        currentX, currentY, currentRCW = self.swervometer.getCOF()
        x_error = self.target_x_pid_controller.calculate(currentX, x)
        y_error = -self.target_y_pid_controller.calculate(currentY, y)
        #rcw_error = self.target_rcw_pid_controller.calculate(currentRCW, rcw)
        ##self.log("hello: x: ", self.target_x_pid_controller.getSetpoint(), " y: ", self.target_y_pid_controller.getSetpoint())
        #if self.target_x_pid_controller.atSetpoint():
            #self.log("X at set point")
        #if self.target_y_pid_controller.atSetpoint():
            #self.log("Y at set point")
        #if self.target_rcw_pid_controller.atSetpoint():
        #    #self.log("RCW at set point")
        
        #if self.target_x_pid_controller.atSetpoint() and self.target_y_pid_controller.atSetpoint() and self.target_rcw_pid_controller.atSetPoint(): 
        # Get current pose                    
        currentX, currentY, currentBearing = self.swervometer.getCOF()
        
        # Debugging               
        #if self.target_x_pid_controller.atSetpoint():
        #    print("X at set point")
        #if self.target_y_pid_controller.atSetpoint():
        #    print("Y at set point")

        if self.target_x_pid_controller.atSetpoint() and self.target_y_pid_controller.atSetpoint(): 
            self.update_smartdash()
            return True
        else:
            self.move(x_error, y_error, 0, bearing)
            
            self.update_smartdash()
            self.execute()
            # #self.log("xPositionError: ", self.target_x_pid_controller.getPositionError(), "yPositionError: ", self.target_y_pid_controller.getPositionError(), "rcwPositionError: ", self.target_rcw_pid_controller.getPositionError())
            # #self.log("xPositionTolerance: ", self.target_x_pid_controller.getPositionTolerance(), "yPositionTolerance: ", self.target_y_pid_controller.getPositionTolerance(), "rcwPositionTolerance: ", self.target_rcw_pid_controller.getPositionTolerance())
            # #self.log("currentX: ", currentX, " targetX: ", x, "x_error: ", x_error, " currentY: ", currentY, " targetY: ", y, " y_error: ", y_error, " currentBearing: ", currentRCW, " self.bearing: ", self.bearing, " target bearing: ", bearing)
            return False
    """
    def calculateVectors(self):
        """
        Calculate the requested speed and angle of each modules from self.requestedVectors and store them in
        self.requestedSpeeds and self.requestedAngles dictionaries.
        """
        self.requestedVectors['fwd'], self.requestedVectors['strafe'], self.requestedVectors['rcw'] = self.normalize([self.requestedVectors['fwd'], self.requestedVectors['strafe'], self.requestedVectors['rcw']])

        # Does nothing if the values are lower than the input thresh
        if self.thresholdInputVectors:
            ##self.log("checking thresholds: fwd: ", self.requestedVectors['fwd'], "strafe: ", self.requestedVectors['strafe'], "rcw: ", self.requestedVectors['rcw'])
            if abs(self.requestedVectors['fwd']) < self.lowerInputThresh:
                ##self.log("forward = 0")
                self.requestedVectors['fwd'] = 0

            if abs(self.requestedVectors['strafe']) < self.lowerInputThresh:
                ##self.log("strafe = 0")
                self.requestedVectors['strafe'] = 0

            if abs(self.requestedVectors['rcw']) < self.lowerInputThresh:
                ##self.log("rcw = 0")
                self.requestedVectors['rcw'] = 0

            if self.requestedVectors['rcw'] == 0 and self.requestedVectors['strafe'] == 0 and self.requestedVectors['fwd'] == 0:  # Prevents a useless loop.
                ##self.log("all three zero")
                self.requestedSpeeds = dict.fromkeys(self.requestedSpeeds, 0) # Do NOT reset the wheel angles.

                if self.wheelLock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defense)

                    self.requestedAngles['front_left'] = -45
                    self.requestedAngles['front_right'] = 45
                    self.requestedAngles['rear_left'] = 45
                    self.requestedAngles['rear_right'] = -45

                    #self.wheelLock = False
                    ##self.log("testing wheel lock")
                return
        
        ratio = math.hypot(self.frameDimensionX, self.frameDimensionY)

        #theta = self.getGyroAngle()
        #if (theta > 45 and theta < 135) or (theta > 225 and theta < 315):
        #    speedSign = -1
        #else:
        #    speedSign = 1

        # Old velocities per quadrant
        rightY = self.requestedVectors['fwd'] + (self.requestedVectors['rcw'] * (frame_dimension_y / ratio))
        leftY = self.requestedVectors['fwd'] - (self.requestedVectors['rcw'] * (frame_dimension_y / ratio))
        rearX = self.requestedVectors['strafe'] + (self.requestedVectors['rcw'] * (frame_dimension_x / ratio))
        frontX = self.requestedVectors['strafe'] - (self.requestedVectors['rcw'] * (frame_dimension_x / ratio))
        
        # Velocities per quadrant
        #rightY = (self.requestedVectors['strafe'] * speedSign) + (self.requestedVectors['rcw'] * (frame_dimension_y / ratio))
        #leftY = (self.requestedVectors['strafe'] * speedSign) - (self.requestedVectors['rcw'] * (frame_dimension_y / ratio))
        #rearX = (self.requestedVectors['fwd'] * speedSign) + (self.requestedVectors['rcw'] * (frame_dimension_x / ratio))
        #frontX = (self.requestedVectors['fwd'] * speedSign) - (self.requestedVectors['rcw'] * (frame_dimension_x / ratio))

        # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        rearLeft_speed = math.hypot(frontX, rightY)
        rearLeft_angle = math.degrees(math.atan2(frontX, rightY))

        frontLeft_speed = math.hypot(frontX, leftY)
        frontLeft_angle = math.degrees(math.atan2(frontX, leftY))

        rearRight_speed = math.hypot(rearX, rightY)
        rearRight_angle = math.degrees(math.atan2(rearX, rightY))

        frontRight_speed = math.hypot(rearX, leftY)
        frontRight_angle = math.degrees(math.atan2(rearX, leftY))

        self.requestedSpeeds['front_left'] = frontLeft_speed
        self.requestedSpeeds['front_right'] = frontRight_speed
        self.requestedSpeeds['rear_left'] = rearLeft_speed
        self.requestedSpeeds['rear_right'] = rearRight_speed

        self.requestedAngles['front_left'] = frontLeft_angle
        self.requestedAngles['front_right'] = frontRight_angle
        self.requestedAngles['rear_left'] = rearLeft_angle
        self.requestedAngles['rear_right'] = rearRight_angle

        self.requestedSpeeds = self.normalizeDictionary(self.requestedSpeeds)

        # Zero request vectors for saftey reasons
        self.requestedVectors['fwd'] = 0.0
        self.requestedVectors['strafe'] = 0.0
        self.requestedVectors['rcw'] = 0.0

    def setWheelLock(self, isLocked):
        ##self.log("is locked", isLocked)
        self.wheelLock = isLocked
    
    def getWheelLock(self):
        return self.wheelLock
    """
    def setRampRates(self, openLoopRampRate, closedLoopRampRate):
        for key in self.modules:
            self.modules[key].setRampRate(openLoopRampRate, closedLoopRampRate)
        
    def debug(self, debug_modules=False):
        
        Prints debugging information to log
        
        if debug_modules:
            for key in self.modules:
                self.modules[key].debug()
        
        #self.log('Requested values: ', self.requestedVectors, '\n')
        #self.log('Requested angles: ', self.requestedAngles, '\n')
        #self.log('Requested speeds: ', self.requestedSpeeds, '\n')
    """
    def execute(self):
        """
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        """
        #self.update_smartdash()

        # Calculate each vector
        self.calculateVectors()

        # Set the speed and angle for each module

        # Calculate normalized speeds with lever arm adjustment
        for key in self.modules:
            ##self.log("Execute: key: ", key, " base speed: ", self.requestedSpeeds[key], " COMmult: ", self.swervometer.getCOMmult(key), " adjusted speed: ", (self.requestedSpeeds[key] * self.swervometer.getCOMmult(key)), self.requestedSpeeds[key] * self.swervometer.getCOMmult(key))
            self.requestedSpeeds[key] = self.requestedSpeeds[key] #* self.swervometer.getCOMmult(key)
        
        self.requestedSpeeds = self.normalizeDictionary(self.requestedSpeeds)

        for key in self.modules:
            self.modules[key].move(self.requestedSpeeds[key], self.requestedAngles[key])
        
        # Reset the speed back to zero
        self.requestedSpeeds = dict.fromkeys(self.requestedSpeeds, 0)

        # Execute each module
        first_module = True
        """for key in self.modules:
            #self.log("Module: Key: ", key)
            self.modules[key].execute()"""

        """"COFX, COFY, COFAngle = self.swervometer.calculateCOFPose(self.modules, self.getGyroAngle())"""
        """
        if self.vision:
            #self.log("Vision started")
            if self.vision.canUpdatePose():
                #self.log("Vision: canupdatepose")
                pose = self.vision.getPose()
                orientation = self.vision.getOrientation()
                if self.vision.shouldUpdatePose():
                    if pose[0] != -1:
                        self.swervometer.setCOF(pose[0], pose[1], orientation[2])
                        #self.log("Vision updated position: (" + str(pose[0]) + ", " + str(pose[1]) + ") with rotation of " + str(orientation[2]) + " degrees.")
                    #else:
                        #self.log("Vision should have updated position, but pose was empty.")
                #else:
                    #self.log("Vision reports position: (" + str(pose[0]) + ", " + str(pose[1]) + ") with rotation of " + str(orientation[2]) + " degrees.")
                #self.log("AFTER COMMENTS")

        #self.log("COFX: ", COFX, ", COFY: ", COFY, ", COF Angle: ", COFAngle)

        if(self.updateBearing):
            #self.log("Old Bearing: ", self.bearing)
            self.bearing = self.getGyroAngle()
            #self.log("New Bearing: ", self.bearing)
            self.updateBearing = False
        """
    def idle(self):
        for key in self.modules:
            self.modules[key].idle()
    """     
    def update_smartdash(self):
        
        Log current state for telemetry

        self.dashboard.putNumber(DASH_PREFIX, '/front_left_req_ang', self.requestedAngles['front_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/front_right_req_ang', self.requestedAngles['front_right'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_left_req_ang', self.requestedAngles['rear_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_right_req_ang', self.requestedAngles['rear_right'])
        
        self.dashboard.putNumber(DASH_PREFIX, '/front_left_req_spd', self.requestedSpeeds['front_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/front_right_req_ang', self.requestedSpeeds['front_right'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_left_req_ang', self.requestedSpeeds['rear_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_right_req_ang', self.requestedSpeeds['rear_right'])
        
        # Zero request vectors for saftey reasons
        self.dashboard.putNumber(DASH_PREFIX, '/req_vector_fwd', self.requestedVectors['fwd'])
        self.dashboard.putNumber(DASH_PREFIX, '/req_vector_strafe', self.requestedVectors['strafe'])
        self.dashboard.putNumber(DASH_PREFIX, '/req_vector_rcw', self.requestedVectors['rcw'])
        self.dashboard.putBoolean(DASH_PREFIX, '/wheelLock', self.wheelLock)
        
        self.dashboard.putNumber(DASH_PREFIX, '/pose_target_x', self.pose_target_x)
        self.dashboard.putNumber(DASH_PREFIX, '/pose_target_y', self.pose_target_y)
        self.dashboard.putNumber(DASH_PREFIX, '/pose_target_bearing', self.pose_target_bearing)
        
        self.dashboard.putNumber(DASH_PREFIX, '/Bearing', self.getBearing())
        self.dashboard.putNumber(DASH_PREFIX, '/Gyro Angle', self.getGyroAngle())
        self.dashboard.putNumber(DASH_PREFIX, '/Gyro Balance', self.getGyroBalance())
        self.dashboard.putNumber(DASH_PREFIX, '/Gyro Pitch', self.getGyroPitch())
        self.dashboard.putNumber(DASH_PREFIX, '/Bearing', self.getGyroRoll())
        self.dashboard.putNumber(DASH_PREFIX, '/Bearing', self.getGyroYaw())
        
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Pitch kP', self.balance_pitch_pid_controller.getP())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Pitch kI', self.balance_pitch_pid_controller.getI())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Pitch kD', self.balance_pitch_pid_controller.getD())

        self.dashboard.putNumber(DASH_PREFIX, '/Balance Yaw kP', self.balance_yaw_pid_controller.getP())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Yaw kI', self.balance_yaw_pid_controller.getI())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Yaw kD', self.balance_yaw_pid_controller.getD())

        for key in self.requestedAngles:
            self.dashboard.putNumber(DASH_PREFIX, '/%s_angle' % key, self.requestedAngles[key])
            self.dashboard.putNumber(DASH_PREFIX, '/%s_speed' % key, self.requestedSpeeds[key])
  
    #def log(self, *dataToLog):
        #self.logger.log(DASH_PREFIX, dataToLog)  
    """