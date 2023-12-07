from swervemodule import SwerveModule
import math
from wpimath.controller import PIDController
#from util import clamp


class SwerveDrive:

    def __init__(self, config, gyro):

        self.frontLeftModule = SwerveModule(config["FRONT_LEFT_MODULE"])
        self.frontRightModule = SwerveModule(config["FRONT_RIGHT_MODULE"])
        self.rearLeftModule = SwerveModule(config["REAR_LEFT_MODULE"])
        self.rearRightModule = SwerveModule(config["REAR_RIGHT_MODULE"])
        self.modules = {
            'front_left': self.frontLeftModule,
            'front_right': self.frontRightModule,
            'rear_left': self.rearLeftModule,
            'rear_right': self.rearRightModule
        }

        self.gyro = gyro
        self.gyroAngleZero = 0.0 #Not sure why we need this, but keep this for right now it was in last year's code
        self.bearing = self.getGyroAngle()
        self.updateBearing = False
        self.bearingPIDController = PIDController(config["BEARING_kP"], config["BEARING_kI"], config["BEARING_kD"])

        self.requestedVectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.requestedSpeeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.requestedAngles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.autonSteerStraight = config["AUTON_STEER_STRAIGHT"]
        self.teleopSteerStraight = config["TELEOP_STEER_STRAIGHT"]
        self.inAuton = False #change later

        self.lower_input_thresh = 0.001 #Set this into auton later
        self.xy_multiplier = 0.65
        self.rotation_multiplier = 0.5
    
        self.frameDimensionX = 13.75 #move to config file
        self.frameDimensionY = 9.75

    def move(self, fwd, strafe, rcw, bearing):
        
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

        #fwd = baseFwd #* self.swervometer.getTeamMoveAdjustment()
        #strafe = baseStrafe #* self.swervometer.getTeamMoveAdjustment()

        #self.log("SWERVEDRIVE Moving:", fwd, strafe, rcw, bearing)

        #Convert field-oriented translate to chassis-oriented translate
        
        currentAngle = self.getGyroAngle() % 360
        desiredAngle = (math.degrees(math.atan2(strafe, fwd))) % 360
        chassisAngle = (desiredAngle - currentAngle) % 360
        #magnitude = clamp(math.hypot(strafe, fwd), 0, 1)
        magnitude = math.hypot(strafe, fwd)
        if magnitude < 0:
            magnitude = 0
        if magnitude > 1:
            magnitude = 1
        
        chassisFwd = magnitude * math.sin(math.radians(chassisAngle))
        chassisStrafe = magnitude * math.cos(math.radians(chassisAngle))

        #self.log("modified strafe: " + str(chassisStrafe) + ", modified fwd: " + str(chassisFwd))
        # self.dashboard.putNumber("Current Gyro Angle", self.getGyroAngle())

        self.requestedVectors['fwd'] = chassisFwd * self.xy_multiplier
        self.requestedVectors['strafe'] = chassisStrafe * self.xy_multiplier

        # self.setFwd(fwd)
        # self.setStrafe(strafe)
        
        # self.log("Drivetrain: Move: shouldSteerStraight:", self.shouldSteerStraight())

        if self.shouldSteerStraight():
            self.requestedVectors['rcw'] = self.steerStraight(rcw, bearing) * self.rotation_multiplier
        else:
            self.requestedVectors['rcw'] = rcw * self.rotation_multiplier
    
        self.calculateVectors() #this will then in turn call execute

    
    def calculateVectors(self):
        
        self.requestedVectors = self.normalizeDictionary(self.requestedVectors)

        if self.threshold_input_vectors:
            #self.log("checking thresholds: fwd: ", self._requested_vectors['fwd'], "strafe: ", self._requested_vectors['strafe'], "rcw: ", self._requested_vectors['rcw'])
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
                #self.log("forward = 0")
                self._requested_vectors['fwd'] = 0

            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                #self.log("strafe = 0")
                self._requested_vectors['strafe'] = 0

            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                #self.log("rcw = 0")
                self._requested_vectors['rcw'] = 0

            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0:  # Prevents a useless loop.
                #self.log("all three zero")
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0) # Do NOT reset the wheel angles.

                if self.wheel_lock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defense)

                    self._requested_angles['front_left'] = -45
                    self._requested_angles['front_right'] = 45
                    self._requested_angles['rear_left'] = 45
                    self._requested_angles['rear_right'] = -45

                    #self.wheel_lock = False
                    #self.log("testing wheel lock")
                return
            
        ratio = math.hypot(self.frameDimensionX, self.frameDimensionY)

        # Old velocities per quadrant
        rightY = self.requestedVectors['fwd'] + (self.requestedVectors['rcw'] * (self.frameDimensionY / ratio))
        leftY = self.requestedVectors['fwd'] - (self.requestedVectors['rcw'] * (self.frameDimensionY / ratio))
        rearX = self.requestedVectors['strafe'] + (self.requestedVectors['rcw'] * (self.frameDimensionX / ratio))
        frontX = self.requestedVectors['strafe'] - (self.requestedVectors['rcw'] * (self.frameDimensionX / ratio))
        
        # Velocities per quadrant
        #rightY = (self.requestedVectors['strafe'] * speedSign) + (self.requestedVectors['rcw'] * (frameDimensionY / ratio))
        #leftY = (self.requestedVectors['strafe'] * speedSign) - (self.requestedVectors['rcw'] * (frameDimensionY / ratio))
        #rearX = (self.requestedVectors['fwd'] * speedSign) + (self.requestedVectors['rcw'] * (frameDimensionX / ratio))
        #frontX = (self.requestedVectors['fwd'] * speedSign) - (self.requestedVectors['rcw'] * (frameDimensionX / ratio))

        # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        rearLeftSpeed = math.hypot(frontX, rightY)
        rearLeftAngle = math.degrees(math.atan2(frontX, rightY))

        frontLeftSpeed = math.hypot(frontX, leftY)
        frontLeftAngle = math.degrees(math.atan2(frontX, leftY))

        rearRightSpeed = math.hypot(rearX, rightY)
        rearRightAngle = math.degrees(math.atan2(rearX, rightY))

        frontRightSpeed = math.hypot(rearX, leftY)
        frontRightAngle = math.degrees(math.atan2(rearX, leftY))

        self.requestedSpeeds['front_left'] = frontLeftSpeed
        self.requestedSpeeds['front_right'] = frontRightSpeed
        self.requestedSpeeds['rear_left'] = rearLeftSpeed
        self.requestedSpeeds['rear_right'] = rearRightSpeed

        self.requestedAngles['front_left'] = frontLeftAngle
        self.requestedAngles['front_right'] = frontRightAngle
        self.requestedAngles['rear_left'] = rearLeftAngle
        self.requestedAngles['rear_right'] = rearRightAngle

        self.requestedSpeeds = self.normalizeDictionary(self.requestedSpeeds)

        # Zero request vectors for saftey reasons
        self.requestedVectors['fwd'] = 0.0
        self.requestedVectors['strafe'] = 0.0
        self.requestedVectors['rcw'] = 0.0

        return
    
    def execute(self):

        """
        for key in self.modules:
            self._requested_speeds[key] = self._requested_speeds[key] * self.swervometer.getCOMmult(key)
        """

        for key in self.modules:
            self.modules[key].move(self.requestedSpeeds[key], self.requestedAngles[key])

        self.requestedSpeeds = dict.fromkeys(self.requestedSpeeds, 0)

        if(self.updateBearing):
            self.bearing = self.getGyroAngle()
            self.updateBearing = False

    
    def getGyroAngle(self):
        #angle = (self.gyro.getAngle() - self.gyroAngleZero + self.swervometer.getTeamGyroAdjustment()) % 360
        angle = (self.gyro.getAngle() - self.gyroAngleZero) % 360
        return angle
    
    def getBearing(self):
        return self.bearing
    
    def shouldSteerStraight(self):
        if self.inAuton:
            return self.autonSteerStraight
        else:
            return self.teleopSteerStraight
    
    def steerStraight(self, rcw, bearing):
        
        self.bearing = bearing
        currentAngle = self.getGyroAngle()
        if rcw != 0:
            self.updateBearing = True
            return rcw
        else:
            self.updateBearing = False
            angleDiff = abs(currentAngle - self.bearing)
            if angleDiff > 180:
                angleDiff = 360 - angleDiff
                if self.bearing < currentAngle:
                    targetAngle = currentAngle + angleDiff
                else:
                    targetAngle = currentAngle - angleDiff
            else:
                if self.bearing < currentAngle:
                    targetAngle = currentAngle - angleDiff
                else:
                    targetAngle = currentAngle + angleDiff

            rcwError = self.bearingPIDController.calculate(self.getGyroAngle(), targetAngle)
            return rcwError
        
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

