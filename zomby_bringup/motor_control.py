import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zomby_bringup.lib.zomby import Zomby
import zomby_bringup.params.zomby_params as zomby_params
from zomby_bringup.lib.tools import saturate, map_range


DEBUG = 1


class MotorControl(Node):
    def __init__(self):
        super().__init__(node_name="motor_control")

        # Initialize serial communication with zomby arduino
        self.zomby = Zomby(
            port="/dev/ttyACM1",
            baud_rate=115200
        )

        # Subscribe to cmd_vel topic
        self.sub_cmd_vel = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self.cmd_vel_cb,
            qos_profile=1
        )

        print("Motor Control node initialized.")

    def cmd_vel_cb(self, msg: Twist):
        """Callback function for cmd_vel subscriber."""
        
        if DEBUG:
            print("Received command:")
            print("\tlinear: " + str(msg.linear.x))
            print("\tangular: " + str(msg.angular.z))

        # Store received command in a vector
        cmd = np.array([[msg.linear.x],
                        [msg.angular.z]])

        # Convert linear and angular velocity to right and left motor speeds.
        r = zomby_params.WHEEL_RADIUS
        l = zomby_params.WHEEL_DISTANCE

        m = np.array([[r/2, r/2],
                      [r/l, -r/l]])
        
        m_inv = np.linalg.inv(m)

        wheel_veloc = m_inv@cmd
        right_speed = wheel_veloc.item(0)
        left_speed = wheel_veloc.item(1)

        if DEBUG:
            print("Calculated right velocity: " + str(right_speed))
            print("Calculated left velocity: " + str(left_speed))

        # Saturate to max velocities
        right_speed_sat = saturate(
            value=right_speed,
            max=zomby_params.MAX_WHEEL_ANGULAR_VELOCITY,
            min=-zomby_params.MAX_WHEEL_ANGULAR_VELOCITY
        )
        left_speed_sat = saturate(
            value=left_speed,
            max=zomby_params.MAX_WHEEL_ANGULAR_VELOCITY,
            min=-zomby_params.MAX_WHEEL_ANGULAR_VELOCITY
        )

        # map to value usable by motor controllers
        right_cmd = map_range(
            value=right_speed_sat,
            in_min=-zomby_params.MAX_WHEEL_ANGULAR_VELOCITY,
            in_max=zomby_params.MAX_WHEEL_ANGULAR_VELOCITY,
            out_min=0,
            out_max=128
        )
        left_cmd = map_range(
            value=left_speed_sat,
            in_min=-zomby_params.MAX_WHEEL_ANGULAR_VELOCITY,
            in_max=zomby_params.MAX_WHEEL_ANGULAR_VELOCITY,
            out_min=0,
            out_max=128
        )

        # cast to ints
        right_cmd_int = int(right_cmd)
        left_cmd_int = int(left_cmd)

        if DEBUG:
            print("Sending speeds:")
            print("\tright: " + str(right_cmd_int))
            print("\tleft: " + str(left_cmd_int))

        self.zomby.setSpeed(
            right_speed=right_cmd_int,
            left_speed=left_cmd_int
        )

def main():
    rclpy.init()

    motor_control = MotorControl()
    rclpy.spin(motor_control)

    rclpy.shutdown()

if __name__ == "__main__":
    main()