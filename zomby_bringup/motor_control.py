import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zomby_bringup.lib.zomby import Zomby
import zomby_bringup.params.zomby_params as zomby_params

DEBUG = 1

def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def saturate(value: float, max: float, min: float):
    sat_value: float
    if value > max:
        sat_value = max
    elif value < min:
        sat_value = min
    else:
        sat_value = value

    return sat_value

class MotorControl(Node):
    def __init__(self):
        super().__init__(node_name="motor_control")

        self.zomby = Zomby(
            port="/dev/ttyACM1",
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
                      [r/l, -r/l]])
        
        m_inv = np.linalg.inv(m)

        wheel_veloc = m_inv@cmd
        right_speed = wheel_veloc.item(0)
        left_speed = wheel_veloc.item(1)

        # Saturate to max velocities
        right_speed_sat = saturate(
            value=right_speed,
            max=zomby_params.MAX_SPEED,
            min=-zomby_params.MAX_SPEED
        )
        left_speed_sat = saturate(
            value=left_speed,
            max=zomby_params.MAX_SPEED,
            min=-zomby_params.MAX_SPEED
        )

        # map to value usable by motor controllers
        right_cmd = map_range(
            value=right_speed_sat,
            in_min=-zomby_params.MAX_SPEED,
            in_max=zomby_params.MAX_SPEED,
            out_min=0,
            out_max=128
        )
        left_cmd = map_range(
            value=left_speed_sat,
            in_min=-zomby_params.MAX_SPEED,
            in_max=zomby_params.MAX_SPEED,
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