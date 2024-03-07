import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zomby import Zomby

class MotorControl(Node):
    def __init__(self):
        super().__init__(node_name="motor_control")

        self.zomby = Zomby(
            port="COM6",
            baud_rate=115200
        )

        self.sub_cmd_vel = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self.cmd_vel_cb,
            qos_profile=10
        )

    def cmd_vel_cb(self, msg: Twist):
        pass

def main():
    rclpy.init()

    motor_control = MotorControl()
    rclpy.spin(motor_control)

    rclpy.shutdown()

if __name__ == "__main__":
    main()