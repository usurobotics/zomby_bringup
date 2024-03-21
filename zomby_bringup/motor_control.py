import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zomby_bringup.lib.zomby import Zomby
import zomby_bringup.params.zomby_params as zomby_params

DEBUG = 1

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

        print("Motor Control node initialized.")

    def cmd_vel_cb(self, msg: Twist):
        
        if DEBUG:
            print("Received command:")
            print("\tlinear: " + str(msg.linear.x))
            print("\tangular: " + str(msg.angular.z))

        cmd = np.array([[msg.linear.x],
                        [msg.angular.z]])
        
        r = zomby_params.WHEEL_RADIUS
        l = zomby_params.WHEEL_DISTANCE

        m = np.array([[r/2, r/2],
                      [r/l, r/l]])
        
        m_inv = np.linalg.inv(m)

        wheel_veloc = m_inv@cmd

        right_speed = int(wheel_veloc.item(0))
        left_speed = int(wheel_veloc.item(1))

        if DEBUG:
            print("Sending speeds:")
            print("\tright: " + str(right_speed))
            print("\tleft: " + str(left_speed))

        self.zomby.setSpeed(
            right_speed=right_speed,
            left_speed=left_speed
        )

def main():
    rclpy.init()

    motor_control = MotorControl()
    rclpy.spin(motor_control)

    rclpy.shutdown()

if __name__ == "__main__":
    main()