"""Code for communicating to zomby's arduino via USB"""

import serial

DEBUG = 1

class Zomby:
    """Class for communicating to zomby's arduino via USB"""
    def wait_for_arduino(self):

        # waits to receive this character
        arduino_ready_signal = 'R'
        
        waiting = True
        
        print("Waiting for Arduino...")

        while waiting:
            if self.__serial_port.in_waiting > 0:
                char = self.__serial_port.read(1).decode('utf-8')
                if char == arduino_ready_signal:
                    print("Arduino is ready.")
                    waiting = False


    def __init__(self, port, baud_rate):
        # Initialize serial port
        self.__serial_port = serial.Serial(port, baud_rate)

        self.__ser_ID_right = b"r"
        self.__ser_ID_left = b"l"


        # Wait for arduino to be ready
        self.wait_for_arduino()

    def setSpeed(self, right_speed, left_speed):

        self.wait_for_arduino()

        if right_speed > 128:
            right_speed_sat = 128
        elif right_speed < 0:
            right_speed_sat = 0
        else:
            right_speed_sat = right_speed

        if left_speed > 128:
            left_speed_sat = 128
        elif left_speed < 0:
            left_speed_sat = 0
        else:
            left_speed_sat = left_speed

        if DEBUG:
            print("send speed ")
        # send right motor ID to arduino
        self.__serial_port.write(self.__ser_ID_right)

        # DEBUG
        if DEBUG:
            print(self.__ser_ID_right)

        # send right motor speed to arduino
        self.__serial_port.write(right_speed_sat.to_bytes(length=1, byteorder='big'))

        # DEBUG
        if DEBUG:
            print(right_speed_sat)

        # send left motor ID to arduino
        self.__serial_port.write(self.__ser_ID_left)

        # DEBUG
        if DEBUG:
            print(self.__ser_ID_left)

        # send left motor speed to arduino
        self.__serial_port.write(left_speed_sat.to_bytes(length=1, byteorder='big'))

        # DEBUG
        if DEBUG:
            print(left_speed_sat)