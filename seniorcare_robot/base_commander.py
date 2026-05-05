from capstone_interfaces.msg import Master2Base


class BaseCommander:
    def __init__(self, publisher):
        self.publisher = publisher

    def _publish(self, stop=False, reset_encoders=False, left_speed=0.0, right_speed=0.0):
        msg = Master2Base()
        msg.stop = stop
        msg.reset_encoders = reset_encoders
        msg.left_speed = float(left_speed)
        msg.right_speed = float(right_speed)
        self.publisher.publish(msg)

    def stop(self):
        self._publish(stop=True, reset_encoders=False, left_speed=0.0, right_speed=0.0)

    def reset_encoders(self):
        self._publish(stop=False, reset_encoders=True, left_speed=0.0, right_speed=0.0)

    def set_speed(self, left_speed: float, right_speed: float):
        self._publish(stop=False, reset_encoders=False,
                      left_speed=left_speed, right_speed=right_speed)

    def go_forward(self, speed: float):
        self.set_speed(speed, speed)

    def go_backward(self, speed: float):
        self.set_speed(-speed, -speed)

    def turn_right(self, speed: float):
        self.set_speed(speed, -speed)

    def turn_left(self, speed: float):
        self.set_speed(-speed, speed)
