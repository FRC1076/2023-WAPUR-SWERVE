import wpilib
import rev
import ctre

class SwerveModule:

    def __init__(self, config):

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        self.rotateMotor = rev.CANSparkMax(config["ROTATE_MOTOR"], motor_type)
        self.rotateEncoder = ctre.CANCoder(config["ROTATE_ENCODER"])
        self.driveMotor = rev.CANSparkMax(config["DRIVE_MOTOR"], motor_type)
        self.driveEncoder = self.driveMotor.getEncoder()

        angle = (self.rotateEncoder.getAbsolutePosition() % 360)
        if angle >= 90 and angle <= 270:
            self.positionSign = +1 
            self.moduleFlipped = False
        else:
            self.positionSign = -1
            self.moduleFlipped = True
        self.headingPIDController = self.rotateMotor.getPIDController()
        self.headingPIDController.setP(config["ROTATE_kP"])
        self.headingPIDController.setI(config["ROTATE_kI"])
        self.headingPIDController.setD(config["ROTATE_kD"])
        self.headingPIDController.enableContinuousInput(0, 360)
        self.headingPIDController.setTolerance(0.5, 0.5)
        return

    def move(self, speed, deg):
        """
        Set the requested speed and rotation of passed.
        :param speed: requested speed of wheel from -1 to 1
        :param deg: requested angle of wheel from 0 to 359 (Will wrap if over or under)
        """
        # deg %= 360 # mod 360, may want to change
        
        
        """
        If the difference between the requested degree and the current degree is
        more than 90 degrees, don't turn the wheel 180 degrees. Instead reverse the speed.
        """
        diff = abs(deg - self.getCurrentAngle())

        if (diff > 180):
            diff = 360 - diff

        if diff > 90: #make this with the new tick-degree methods
            self.moduleFlipped = not self.moduleFlipped
            self.positionSign *= -1

        if self.moduleFlipped:
            speed *= -1
            #deg += 180
            #deg %= 360
        
        #print("Module Flipped Test: flipped: ", self.moduleFlipped, " speed: ", speed, " positionSign: ", self.positionSign)
        angle = deg%360
        driveOutput = speed
        
        rotateOutput = self.headingPIDController.calculate(self.getCurrentAngle(), angle)

        self.rotateMotor.set(rotateOutput)
        self.driveMotor.set(driveOutput)


    def getCurrentAngle(self):
        angle = (self.rotateEncoder.getAbsolutePosition()) % 360
        
        if self.moduleFlipped:
            angle = (angle + 180) % 360
        
        return angle
    
    def getCurrentVelocity(self):
        velocity = self.driveEncoder.getVelocity()
        #multiply by ratio (inches / rotation)
        return velocity