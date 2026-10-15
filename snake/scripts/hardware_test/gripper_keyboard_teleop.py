#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rospy

from spinal.msg import (
    ServoStates,
    ServoControlCmd,
    ServoTorqueCmd,
)


class GripperKeyboardTeleop:
    # ================================================================
    # Servo configuration
    # ================================================================

    # DYNAMIXEL ID1 -> servo index 0
    # Lower jaw
    LOWER_JAW = 0

    # DYNAMIXEL ID2 -> servo index 1
    # Upper jaw
    UPPER_JAW = 1

    # ------------------------------------------------------------
    # Direction confirmed on the actual gripper
    #
    # Lower jaw:
    #   angle increases -> OPEN
    #   angle decreases -> CLOSE
    #
    # Upper jaw:
    #   angle decreases -> UP / OPEN
    #   angle increases -> DOWN / CLOSE
    # ------------------------------------------------------------

    LOWER_OPEN_SIGN = +1
    UPPER_UP_SIGN = -1

    # ================================================================
    # Initialization
    # ================================================================

    def __init__(self):
        # ------------------------------------------------------------
        # Motion parameters
        # ------------------------------------------------------------

        # Initial movement per key press [count]
        self.step = 20

        # ------------------------------------------------------------
        # Servo states
        # ------------------------------------------------------------

        self.current = {
            self.LOWER_JAW: None,
            self.UPPER_JAW: None,
        }

        self.target = {
            self.LOWER_JAW: None,
            self.UPPER_JAW: None,
        }

        self.errors = {
            self.LOWER_JAW: None,
            self.UPPER_JAW: None,
        }

        self.torque_on = False

        # ------------------------------------------------------------
        # ROS publishers
        # ------------------------------------------------------------

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

        # ------------------------------------------------------------
        # ROS subscribers
        # ------------------------------------------------------------

        rospy.Subscriber(
            "/servo/states",
            ServoStates,
            self.servo_state_callback,
        )

        rospy.on_shutdown(self.shutdown)

    # ================================================================
    # Servo state
    # ================================================================

    def servo_state_callback(self, msg):
        for servo in msg.servos:
            if servo.index in self.current:
                self.current[servo.index] = int(servo.angle)
                self.errors[servo.index] = int(servo.error)

    def states_available(self):
        return all(
            value is not None
            for value in self.current.values()
        )

    def has_hardware_error(self):
        error_found = False

        for index in [self.LOWER_JAW, self.UPPER_JAW]:
            error = self.errors[index]

            if error is not None and error != 0:
                print(
                    f"[ERROR] Servo index {index}: "
                    f"hardware error = {error}"
                )
                error_found = True

        return error_found

    # ================================================================
    # Torque control
    # ================================================================

    def enable_torque(self):
        if not self.states_available():
            print("Servo states are not available.")
            return

        if self.has_hardware_error():
            print("Torque ON cancelled due to hardware error.")
            return

        # Use the current positions as the first targets.
        #
        # IMPORTANT:
        # Do NOT publish torque_enable = 1 here.
        # In the current DirectServo firmware, publishing a target position
        # while torque is OFF sets the goal position first and then enables
        # torque automatically. This avoids the separate torque-enable path
        # that caused an approximately one-revolution jump in Extended
        # Position Control Mode.
        for index in [self.LOWER_JAW, self.UPPER_JAW]:
            self.target[index] = self.current[index]

        self.publish_targets([
            self.LOWER_JAW,
            self.UPPER_JAW,
        ])

        rospy.sleep(0.1)

        self.torque_on = True

        print("")
        print("Torque ON")
        print(
            f"Lower jaw : "
            f"{self.current[self.LOWER_JAW]}"
        )
        print(
            f"Upper jaw : "
            f"{self.current[self.UPPER_JAW]}"
        )
        print(
            f"Step      : "
            f"{self.step} count"
        )
        print("")

    def disable_torque(self):
        msg = ServoTorqueCmd()

        msg.index = [
            self.LOWER_JAW,
            self.UPPER_JAW,
        ]

        msg.torque_enable = [
            0,
            0,
        ]

        self.torque_pub.publish(msg)

        self.torque_on = False

        print("Torque OFF")

    # ================================================================
    # Servo target publishing
    # ================================================================

    def publish_targets(self, indices):
        msg = ServoControlCmd()

        msg.index = []
        msg.angles = []

        for index in indices:
            if self.target[index] is not None:
                msg.index.append(index)
                msg.angles.append(
                    int(self.target[index])
                )

        if len(msg.index) > 0:
            self.target_pub.publish(msg)

    # ================================================================
    # Servo movement
    # ================================================================

    def move_servo(self, index, delta):
        if not self.torque_on:
            print("Torque is OFF. Press 't' first.")
            return

        if self.current[index] is None:
            print(
                f"Servo index {index} position "
                f"is unavailable."
            )
            return

        if self.errors[index] not in [None, 0]:
            print(
                f"[ERROR] Servo index {index}: "
                f"hardware error = {self.errors[index]}"
            )
            return

        if self.target[index] is None:
            self.target[index] = self.current[index]

        # ------------------------------------------------------------
        # Calculate new target
        #
        # NOTE:
        # No software motion-range limit is applied here.
        # ------------------------------------------------------------

        new_target = (
            self.target[index]
            + delta
        )

        # ServoControlCmd.angles is int16.
        # This is only a message data-type limit,
        # not a gripper motion-range limit.
        new_target = max(
            -32768,
            min(
                32767,
                new_target,
            ),
        )

        self.target[index] = int(new_target)

        self.publish_targets([
            index,
        ])

        print(
            f"Servo {index}: "
            f"target={self.target[index]}, "
            f"current={self.current[index]}, "
            f"step={self.step}"
        )

    # ================================================================
    # Lower jaw
    # ================================================================

    def lower_open(self):
        print("Lower jaw: OPEN")

        self.move_servo(
            self.LOWER_JAW,
            self.LOWER_OPEN_SIGN * self.step,
        )

    def lower_close(self):
        print("Lower jaw: CLOSE")

        self.move_servo(
            self.LOWER_JAW,
            -self.LOWER_OPEN_SIGN * self.step,
        )

    # ================================================================
    # Upper jaw
    # ================================================================

    def upper_up(self):
        print("Upper jaw: UP")

        self.move_servo(
            self.UPPER_JAW,
            self.UPPER_UP_SIGN * self.step,
        )

    def upper_down(self):
        print("Upper jaw: DOWN")

        self.move_servo(
            self.UPPER_JAW,
            -self.UPPER_UP_SIGN * self.step,
        )

    # ================================================================
    # Step adjustment
    # ================================================================

    def increase_step(self):
        self.step += 10

        print(
            f"Step = {self.step} count"
        )

    def decrease_step(self):
        self.step = max(
            1,
            self.step - 10,
        )

        print(
            f"Step = {self.step} count"
        )

    def set_step(self, step):
        self.step = step

        print(
            f"Step = {self.step} count"
        )

    # ================================================================
    # Status
    # ================================================================

    def print_status(self):
        print("")
        print(
            "========== Gripper Status =========="
        )

        print("Lower jaw (index 0)")
        print(
            f"  current : "
            f"{self.current[self.LOWER_JAW]}"
        )
        print(
            f"  target  : "
            f"{self.target[self.LOWER_JAW]}"
        )
        print(
            f"  error   : "
            f"{self.errors[self.LOWER_JAW]}"
        )

        print("")

        print("Upper jaw (index 1)")
        print(
            f"  current : "
            f"{self.current[self.UPPER_JAW]}"
        )
        print(
            f"  target  : "
            f"{self.target[self.UPPER_JAW]}"
        )
        print(
            f"  error   : "
            f"{self.errors[self.UPPER_JAW]}"
        )

        print("")

        print(
            f"Torque : "
            f"{'ON' if self.torque_on else 'OFF'}"
        )

        print(
            f"Step   : "
            f"{self.step} count"
        )

        print(
            "===================================="
        )
        print("")

    # ================================================================
    # Shutdown
    # ================================================================

    def shutdown(self):
        try:
            self.disable_torque()
            rospy.sleep(0.1)

        except Exception:
            pass


# ====================================================================
# Keyboard input
# ====================================================================

def get_key(settings):
    tty.setcbreak(
        sys.stdin.fileno()
    )

    readable, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        0.1,
    )

    if readable:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings,
    )

    return key


# ====================================================================
# Help
# ====================================================================

def print_help():
    print(
        """
================================================
 Gripper Keyboard Teleoperation
================================================

 Lower jaw - index 0
 ------------------------------------------------
 o : OPEN
     angle increases

 c : CLOSE
     angle decreases


 Upper jaw - index 1
 ------------------------------------------------
 u : UP / OPEN
     angle decreases

 d : DOWN / CLOSE
     angle increases


 Movement step
 ------------------------------------------------
 + : increase step by 10 count
 - : decrease step by 10 count

 1 : step = 1 count
 2 : step = 5 count
 3 : step = 10 count
 4 : step = 20 count
 5 : step = 50 count


 General
 ------------------------------------------------
 t     : torque ON

 SPACE : torque OFF

 p     : print current status

 h     : print help

 q     : torque OFF and quit


 IMPORTANT
 ------------------------------------------------
 There is currently NO software motion-range limit.

================================================
"""
    )


# ====================================================================
# Main
# ====================================================================

def main():
    rospy.init_node(
        "gripper_keyboard_teleop"
    )

    teleop = GripperKeyboardTeleop()

    settings = termios.tcgetattr(
        sys.stdin.fileno()
    )

    print(
        "Waiting for /servo/states ..."
    )

    rate = rospy.Rate(20)

    while (
        not rospy.is_shutdown()
        and not teleop.states_available()
    ):
        rate.sleep()

    if rospy.is_shutdown():
        return

    print("")
    print("Both servos detected.")

    print(
        f"Lower jaw (index 0): "
        f"{teleop.current[0]}"
    )

    print(
        f"Upper jaw (index 1): "
        f"{teleop.current[1]}"
    )

    print_help()

    try:
        while not rospy.is_shutdown():

            key = get_key(
                settings
            )

            # ========================================================
            # Torque
            # ========================================================

            if key == "t":
                teleop.enable_torque()

            elif key == " ":
                teleop.disable_torque()

            # ========================================================
            # Lower jaw
            # ========================================================

            elif key == "o":
                teleop.lower_open()

            elif key == "c":
                teleop.lower_close()

            # ========================================================
            # Upper jaw
            # ========================================================

            elif key == "u":
                teleop.upper_up()

            elif key == "d":
                teleop.upper_down()

            # ========================================================
            # Step
            # ========================================================

            elif key in ["+", "="]:
                teleop.increase_step()

            elif key == "-":
                teleop.decrease_step()

            elif key == "1":
                teleop.set_step(1)

            elif key == "2":
                teleop.set_step(5)

            elif key == "3":
                teleop.set_step(10)

            elif key == "4":
                teleop.set_step(20)

            elif key == "5":
                teleop.set_step(50)

            # ========================================================
            # Other commands
            # ========================================================

            elif key == "p":
                teleop.print_status()

            elif key == "h":
                print_help()

            elif key == "q":
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
