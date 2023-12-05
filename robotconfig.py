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
        "ROTATE_ENCODER": 1,
        "DRIVE_MOTOR": 2,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "FRONT_RIGHT_MODULE": {
        "ROTATE_MOTOR": 3,
        "ROTATE_ENCODER": 4,
        "DRIVE_MOTOR": 5,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "REAR_LEFT_MODULE": {
        "ROTATE_MOTOR": 6,
        "ROTATE_ENCODER": 7,
        "DRIVE_MOTOR": 8,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "REAR_RIGHT_MODULE": {
        "ROTATE_MOTOR": 9,
        "ROTATE_ENCODER": 10,
        "DRIVE_MOTOR": 11,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "BEARING_kP": 0.025,
    "BEARING_kI": 0.00001,
    "BEARING_kD": 0.0001,
    "AUTON_STEER_STRAIGHT": False,
    "TELEOP_STEER_STRAIGHT": False,
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