from swervemodule import SwerveModule
import math
from util import clamp


class SwerveDrive:

    def __init__(self,  config, gyro):

        self.frontLeftModule = SwerveModule(config["FRONT_LEFT_MODULE"])
        self.frontRightModule = SwerveModule(config["FRONT_RIGHT_MODULE"])
        self.rearLeftModule = SwerveModule(config["REAR_LEFT_MODULE"])
        self.realRightModule = SwerveModule(config["REAR_RIGHT_MODULE"])

        self.gyro = gyro
        self.gyroAngleZero = 0.0
        self.bearing = self.getGyroAngle()

        self.requestedVectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.autonSteerStraight = config["AUTON_STEER_STRAIGHT"]
        self.teleopSteerStraight = config["TELEOP_STEER_STRAIGHT"]
        self.inAuton = False #change later

        return
    
    def move(self, baseFwd, baseStrafe, rcw, bearing):
        
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
        #self.log("SWERVEDRIVE: MoveAdjustment: ", self.swervometer.getTeamMoveAdjustment())
        fwd = baseFwd #* self.swervometer.getTeamMoveAdjustment()
        strafe = baseStrafe #* self.swervometer.getTeamMoveAdjustment()

        #self.log("SWERVEDRIVE Moving:", fwd, strafe, rcw, bearing)

        #Convert field-oriented translate to chassis-oriented translate
        
        currentAngle = self.getGyroAngle() % 360
        desiredAngle = (math.degrees(math.atan2(strafe, fwd))) % 360
        chassisAngle = (desiredAngle - currentAngle) % 360
        magnitude = clamp(math.hypot(strafe, fwd), 0, 1)
        
        chassisFwd = magnitude * math.sin(math.radians(chassisAngle))
        chassisStrafe = magnitude * math.cos(math.radians(chassisAngle))

        #self.log("modified strafe: " + str(chassisStrafe) + ", modified fwd: " + str(chassisFwd))
        # self.dashboard.putNumber("Current Gyro Angle", self.getGyroAngle())

        self.setFWD(chassisFwd)
        self.setStrafe(chassisStrafe)

        # self.setFwd(fwd)
        # self.setStrafe(strafe)
        
        # self.log("Drivetrain: Move: shouldSteerStraight:", self.shouldSteerStraight())

        if self.shouldSteerStraight():
            self.setRCW(self.steerStraight(rcw, bearing))
        else:
            self.setRCW(rcw)
    
    def calculateVectors(self):
        self.requestedVectors['fwd'], self.requestedVectors['strafe'], self.requestedVectors['rcw'] = self.normalize([self.requestedVectors['fwd'], self.requestedVectors['strafe'], self.requestedVectors['rcw']])
        frameDimensionX, frameDimensionY = [0, 0]   #self.swerveometer.getFrameDimensions()
        ratio = math.hypot(frameDimensionX, frameDimensionY)
        return
    
    def execute(self):
        return
    
    def getGyroAngle(self):
        #angle = (self.gyro.getAngle() - self.gyroAngleZero + self.swervometer.getTeamGyroAdjustment()) % 360
        angle = (self.gyro.getAngle() - self.gyroAngleZero) % 360
        return angle
    
    def getBearing(self):
        return self.bearing

    def setFWD(self, fwd):
        #fwd *= self.xymultiplier
        self.requestedVectors['fwd'] = fwd

    def setStrafe(self, strafe):
        #strafe *= self.xymultiplier
        self.requestedVectors['strafe'] = strafe
    
    def setRCW(self, rcw):
        #rcw *= self.xymultiplier
        self.requestedVectors['rcw'] = rcw
        return
    
    def shouldSteerStraight(self):
        if self.inAuton:
            return self.autonSteerStraight
        else:
            return self.teleopSteerStraight
        
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

