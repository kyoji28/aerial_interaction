#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rospy

from spinal.msg import (
    ServoStates,
    ServoTorqueStates,
    ServoControlCmd,
    ServoTorqueCmd,
)


class KeyboardServoTeleop:
    def __init__(self):
        # -------------------------
        # Parameters
        # -------------------------
        self.servo_index = rospy.get_param("~servo_index", 0)
        self.step = rospy.get_param("~step", 20)
        self.max_delta = rospy.get_param("~max_delta", 300)

        # -------------------------
        # Servo state
        # -------------------------
        self.current_position = None
        self.target_position = None
        self.reference_position = None
        self.torque_on = False

        # -------------------------
        # ROS I/O
        # -------------------------
        self.target_pub = rospy.Publisher(
            "/servo/target_states",
            ServoControlCmd,
            queue_size=1,
        )

        self.torque_pub = rospy.Publisher(
            "/servo/torque_enable",
            ServoTorqueCmd,
            queue_size=1,
        )

        rospy.Subscriber(
            "/servo/states",
            ServoStates,
            self.servo_state_callback,
        )

        rospy.Subscriber(
            "/servo/torque_states",
            ServoTorqueStates,
            self.torque_state_callback,
        )

        rospy.on_shutdown(self.shutdown)

    def servo_state_callback(self, msg):
        for servo in msg.servos:
            if servo.index == self.servo_index:
                self.current_position = int(servo.angle)
                break

    def torque_state_callback(self, msg):
        if self.servo_index < len(msg.torque_enable):
            self.torque_on = bool(msg.torque_enable[self.servo_index])

    def publish_torque(self, enable):
        msg = ServoTorqueCmd()
        msg.index = [self.servo_index]
        msg.torque_enable = [1 if enable else 0]
        self.torque_pub.publish(msg)

    def enable_torque(self):
        if self.current_position is None:
            print("Servo position has not been received yet.")
            return

        # Current position becomes the center of the permitted motion range.
        self.reference_position = self.current_position
        self.target_position = self.current_position

        print(
            f"Torque ON request: current={self.current_position}, "
            f"allowed range="
            f"[{self.reference_position - self.max_delta}, "
            f"{self.reference_position + self.max_delta}]"
        )

        self.publish_torque(True)

    def disable_torque(self):
        self.publish_torque(False)

        self.torque_on = False
        self.target_position = None
        self.reference_position = None

        print("Torque OFF")

    def move(self, delta):
        if not self.torque_on:
            print("Torque is OFF. Press 't' first.")
            return

        if self.current_position is None:
            print("Servo position is unavailable.")
            return

        if self.target_position is None:
            self.target_position = self.current_position

        if self.reference_position is None:
            self.reference_position = self.current_position

        new_target = self.target_position + delta

        # Software motion limit around the position where torque was enabled.
        lower = self.reference_position - self.max_delta
        upper = self.reference_position + self.max_delta

        if new_target < lower:
            new_target = lower
            print("Reached negative software limit.")

        if new_target > upper:
            new_target = upper
            print("Reached positive software limit.")

        # ServoControlCmd.angles is int16.
        new_target = max(-32768, min(32767, new_target))

        msg = ServoControlCmd()
        msg.index = [self.servo_index]
        msg.angles = [int(new_target)]

        self.target_pub.publish(msg)

        self.target_position = new_target

        print(
            f"target={self.target_position}, "
            f"current={self.current_position}, "
            f"step={self.step}"
        )

    def print_status(self):
        print("")
        print("----- Servo status -----")
        print(f"servo index : {self.servo_index}")
        print(f"current     : {self.current_position}")
        print(f"target      : {self.target_position}")
        print(f"torque      : {'ON' if self.torque_on else 'OFF'}")
        print(f"step        : {self.step}")
        print(f"max delta   : {self.max_delta}")
        print("------------------------")
        print("")

    def shutdown(self):
        # Try to release the servo when this node exits.
        try:
            self.publish_torque(False)
            rospy.sleep(0.1)
        except Exception:
            pass


def get_key(settings):
    tty.setraw(sys.stdin.fileno())

    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings,
    )

    return key


def print_help():
    print(
        """
========================================
 XC330 Keyboard Teleoperation
========================================

 t       : torque ON at current position
 SPACE   : torque OFF

 a       : move negative direction
 d       : move positive direction

 +       : increase step
 -       : decrease step

 p       : print status

 q       : torque OFF and quit

----------------------------------------
Always press 't' before using a/d.
========================================
"""
    )


def main():
    rospy.init_node("keyboard_servo_teleop")

    teleop = KeyboardServoTeleop()

    settings = termios.tcgetattr(sys.stdin)

    print("Waiting for /servo/states ...")

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and teleop.current_position is None:
        rate.sleep()

    if rospy.is_shutdown():
        return

    print(
        f"Servo detected: index={teleop.servo_index}, "
        f"position={teleop.current_position}"
    )

    print_help()

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == "t":
                teleop.enable_torque()

            elif key == " ":
                teleop.disable_torque()

            elif key == "a":
                teleop.move(-teleop.step)

            elif key == "d":
                teleop.move(+teleop.step)

            elif key in ["+", "="]:
                teleop.step += 10
                print(f"step = {teleop.step}")

            elif key == "-":
                teleop.step = max(10, teleop.step - 10)
                print(f"step = {teleop.step}")

            elif key == "p":
                teleop.print_status()

            elif key == "q":
                teleop.disable_torque()
                break

    finally:
        teleop.disable_torque()

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings,
        )


if __name__ == "__main__":
    main()
