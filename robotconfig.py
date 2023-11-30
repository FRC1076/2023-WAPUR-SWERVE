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

}

visionConfig = {

}

elevatorConfig = {
    "LEFT_MOTOR_ID": 1,
    "RIGHT_MOTOR_ID": 2,
    "SHELF_HEIGHT_A": 4, # Lowest, in inches
    "SHELF_HEIGHT_B": 18,
    "SHELF_HEIGHT_C": 32,
    "SHELF_HEIGHT_D": 46,
    'LOWER_SAFETY': 1,
    'UPPER_SAFETY': 33,
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
    "ELEVATOR": elevatorConfig,
}

