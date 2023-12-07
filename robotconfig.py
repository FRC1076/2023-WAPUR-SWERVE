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

TICKS_PER_INCH = 2
elevatorConfig = {
    "LEFT_MOTOR_ID": 5,
    "RIGHT_MOTOR_ID": 6,
    "SHELF_HEIGHT_A": 4/TICKS_PER_INCH, # Lowest shelf, each tick is 2 inches
    "SHELF_HEIGHT_B": 18/TICKS_PER_INCH,
    "SHELF_HEIGHT_C": 32/TICKS_PER_INCH,
    "SHELF_HEIGHT_D": 46/TICKS_PER_INCH,
    "MARGIN": 1/TICKS_PER_INCH, # Extra height so the crate is slightly above shelf surface
    'LOWER_SAFETY': 1,
    'UPPER_SAFETY': 33,
    "KP": 0.2, 
    "KI": 1.0,
    "KD": 0,
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

