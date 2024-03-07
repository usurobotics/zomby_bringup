import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zomby_bringup.lib.zomby import Zomby
import params.zomby_parameters as zomby_params

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
        cmd = np.array([[msg.linear.x],
                        [msg.angular.z]])
        m = (zomby_params.wheel_radius/2) * np.array([[1,1],
                                                      [1,-1]])
        
        m_inv = np.linalg.inv(m)

        wheel_veloc = m_inv@cmd

        right_speed = wheel_veloc.item(0)
        left_speed = wheel_veloc.item(1)

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