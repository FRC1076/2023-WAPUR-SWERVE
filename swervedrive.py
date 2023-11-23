from swervemodule import SwerveModule

class SwerveDrive:
    def __init__(self,  config):
        self.frontLeftModule = SwerveModule(config["FRONT_LEFT_MODULE"])
        self.frontRightModule = SwerveModule(config["FRONT_RIGHT_MODULE"])
        self.rearLeftModule = SwerveModule(config["REAR_LEFT_MODULE"])
        self.realRightModule = SwerveModule(config["REAR_RIGHT_MODULE"])
        return