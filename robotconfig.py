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
        "ROTATE_MOTOR": 11,
        "ROTATE_ENCODER": 21,
        "DRIVE_MOTOR": 1, #
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "FRONT_RIGHT_MODULE": {
        "ROTATE_MOTOR": 12,
        "ROTATE_ENCODER": 22,
        "DRIVE_MOTOR": 2,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "REAR_LEFT_MODULE": {
        "ROTATE_MOTOR": 14,
        "ROTATE_ENCODER": 24,
        "DRIVE_MOTOR": 4,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "REAR_RIGHT_MODULE": {
        "ROTATE_MOTOR": 13,
        "ROTATE_ENCODER": 23,
        "DRIVE_MOTOR": 3,
        "ROTATE_kP": 0.025,
        "ROTATE_kI": 0.00001,
        "ROTATE_kD": 0.0001,
    },
    "BEARING_kP": 0.025,
    "BEARING_kI": 0.00001,
    "BEARING_kD": 0.0001,
    "AUTON_STEER_STRAIGHT": False,
    "TELEOP_STEER_STRAIGHT": False,
    "ROBOT_SWERVE_MODULE_OFFSET_X": 13.75,
    "ROBOT_SWERVE_MODULE_OFFSET_Y": 9.75,
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