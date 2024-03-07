import zomby_bringup.lib.zomby as zomby
import time as pytime

if __name__ == "__main__":
    robot = zomby.Zomby(port="COM9", baud_rate=115200)

    max_speed: int = 90

    min_speed: int = 40

    speed: int = 64 # start at stopped!

    step: int = 1

    increment: bool = True

    count: int = 0

    num_cycles: int = 4

    delay: float = 0.05

    run: bool = True


    while run:
        #robot.wait_for_arduino()
        robot.setSpeed(
            left_speed=speed, 
            right_speed=speed
        )

        if speed == max_speed:
            increment = False

        if not increment and speed == min_speed:
            increment = True
            count += 1

        if speed == 64 and count == num_cycles:
            run = False

        if increment:
            speed += step
        else:
            speed -= step

        pytime.sleep(delay)