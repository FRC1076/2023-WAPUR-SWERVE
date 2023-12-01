from collections import namedtuple

DEADZONE = 0.1

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

controllerConfig = {
    "DRIVER": {
        "ID": 0,
        "DEADZONE": DEADZONE,
        "LEFT_TRIGGER_AXIS": 2,
        "RIGHT_TRIGGER_AXIS": 3,
    },
    "OPERATOR": {
        "ID": 1,
        "DEADZONE": DEADZONE,
        "LEFT_TRIGGER_AXIS": 2,
        "RIGHT_TRIGGER_AXIS": 3,
    }
}

swervometerConfig = {

}

drivetrainConfig = {
    "FRONT_LEFT_MODULE": {
        "ROTATE_MOTOR": 0,
        "ROTATE_ENCODER": 0,
        "DRIVE_MOTOR": 0,
    },
    "FRONT_RIGHT_MODULE": {
        "ROTATE_MOTOR": 0,
        "ROTATE_ENCODER": 0,
        "DRIVE_MOTOR": 0,
    },
    "REAR_LEFT_MODULE": {
        "ROTATE_MOTOR": 0,
        "ROTATE_ENCODER": 0,
        "DRIVE_MOTOR": 0,
    },
    "REAR_RIGHT_MODULE": {
        "ROTATE_MOTOR": 0,
        "ROTATE_ENCODER": 0,
        "DRIVE_MOTOR": 0,
    },
    "BEARING_kP": 0.025,
    "BEARING_kI": 0.00001,
    "BEARING_kD": 0.0001,
}

visionConfig = {

}

elevatorConfig = {

}

grabberConfig = {

}

autonConfig = {

}

MODULE_NAMES = namedtuple('MODULE_NAMES', [
])

loggingConfig = {

}

dashboardConfig = {

}

robotconfig = {
    "CONTROLLERS": controllerConfig,
    "DRIVETRAIN": drivetrainConfig,
}