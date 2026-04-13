class ControlInput:
    def __init__(self, acc, steering):
        if acc > 0:
            self.accel = min(acc, 1.0)
            self.brake = 0.
        else:
            self.accel = 0.
            self.brake = min(-acc, 1.0)
        self.steering = steering
