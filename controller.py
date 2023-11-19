class Controller:
    def __init__(self, xboxController, deadzone, 
                 leftTriggerAxis, rightTriggerAxis):
        self.xboxController = xboxController
        self.deadzone = deadzone
        self.leftTriggerAxis = leftTriggerAxis
        self.rightTriggerAxis = rightTriggerAxis