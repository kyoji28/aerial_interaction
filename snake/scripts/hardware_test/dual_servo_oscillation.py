#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty

import rospy

from spinal.msg import (
    ServoStates,
    ServoControlCmd,
    ServoTorqueCmd,
)


class DualServoOscillation:
    COUNTS_PER_REV = 4096.0

    def __init__(self):
        # Control target
        self.servo_indices = [0, 1]

        # Motion parameters
        self.center_deg = rospy.get_param("~center_deg", 180.0)
        self.amplitude_deg = rospy.get_param("~amplitude_deg", 30.0)

        # Time to move from the current position to 180 deg
        self.move_duration = rospy.get_param("~move_duration", 3.0)

        # Oscillation period [s]
        # 4 s = one complete cycle every 4 seconds
        self.period = rospy.get_param("~period", 4.0)

        # Command frequency
        self.control_rate = rospy.get_param("~control_rate", 50.0)

        self.center_count = self.deg_to_count(self.center_deg)
        self.amplitude_count = self.deg_to_count(self.amplitude_deg)

        self.current_angles = {}
        self.current_errors = {}

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

        rospy.on_shutdown(self.shutdown)

        self.shutdown_done = False

    def deg_to_count(self, deg):
        return int(round(
            deg / 360.0 * self.COUNTS_PER_REV
        ))

    def count_to_deg(self, count):
        # Convert to one-turn representation
        wrapped = count % int(self.COUNTS_PER_REV)

        return (
            wrapped /
            self.COUNTS_PER_REV *
            360.0
        )

    def normalize_count(self, count):
        """
        Convert an angle to the one-turn DYNAMIXEL range:
        0 ... 4095
        """
        return int(count) % int(self.COUNTS_PER_REV)

    def servo_state_callback(self, msg):
        for servo in msg.servos:
            if servo.index in self.servo_indices:
                self.current_angles[servo.index] = int(servo.angle)
                self.current_errors[servo.index] = int(servo.error)

    def states_available(self):
        return all(
            index in self.current_angles
            for index in self.servo_indices
        )

    def print_current_states(self):
        print("")
        print("========== Current Servo States ==========")

        for index in self.servo_indices:
            raw = self.current_angles[index]
            normalized = self.normalize_count(raw)
            deg = self.count_to_deg(raw)
            error = self.current_errors.get(index, -1)

            print(
                f"Servo index {index}: "
                f"raw={raw}, "
                f"normalized={normalized}, "
                f"angle={deg:.2f} deg, "
                f"error={error}"
            )

        print("==========================================")
        print("")

    def publish_targets(self, targets):
        msg = ServoControlCmd()

        msg.index = self.servo_indices
        msg.angles = [
            int(targets[0]),
            int(targets[1]),
        ]

        self.target_pub.publish(msg)

    def torque_off(self):
        msg = ServoTorqueCmd()

        msg.index = self.servo_indices
        msg.torque_enable = [0, 0]

        self.torque_pub.publish(msg)

    def move_to_center(self):
        """
        Move both servos smoothly from their current positions
        to the 180-degree position.
        """

        start_positions = [
            self.normalize_count(
                self.current_angles[index]
            )
            for index in self.servo_indices
        ]

        print("Moving to center...")
        print(
            f"Target center: "
            f"{self.center_deg:.1f} deg "
            f"({self.center_count} counts)"
        )

        #
        # First send the current position itself.
        # /servo/target_states will enable torque,
        # so this minimizes an initial jump.
        #
        self.publish_targets(start_positions)

        rospy.sleep(0.5)

        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            elapsed = (
                rospy.Time.now().to_sec()
                - start_time
            )

            u = elapsed / self.move_duration

            if u >= 1.0:
                break

            #
            # Smoothstep interpolation
            # s = 3u^2 - 2u^3
            #
            s = (
                3.0 * u * u
                - 2.0 * u * u * u
            )

            targets = []

            for start in start_positions:
                target = (
                    start
                    + (
                        self.center_count
                        - start
                    ) * s
                )

                targets.append(
                    int(round(target))
                )

            self.publish_targets(targets)

            rate.sleep()

        self.publish_targets([
            self.center_count,
            self.center_count,
        ])

        rospy.sleep(0.5)

        print("Reached center position.")

    def oscillate(self):
        print("")
        print("Oscillation started.")
        print(
            f"Center    : {self.center_deg:.1f} deg "
            f"({self.center_count} counts)"
        )
        print(
            f"Amplitude : +/-{self.amplitude_deg:.1f} deg "
            f"(+/-{self.amplitude_count} counts)"
        )
        print(
            f"Period    : {self.period:.2f} s"
        )
        print("")
        print("Press 'q' to stop.")
        print("")

        old_settings = termios.tcgetattr(
            sys.stdin.fileno()
        )

        tty.setcbreak(
            sys.stdin.fileno()
        )

        rate = rospy.Rate(
            self.control_rate
        )

        start_time = (
            rospy.Time.now().to_sec()
        )

        try:
            while not rospy.is_shutdown():

                # Check keyboard without blocking
                readable, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0,
                )

                if readable:
                    key = sys.stdin.read(1)

                    if key.lower() == "q":
                        print("")
                        print("Stop requested.")
                        break

                t = (
                    rospy.Time.now().to_sec()
                    - start_time
                )

                phase = (
                    2.0 *
                    math.pi *
                    t /
                    self.period
                )

                offset = (
                    self.amplitude_count
                    * math.sin(phase)
                )

                target = int(round(
                    self.center_count
                    + offset
                ))

                #
                # Same command for both servos:
                # synchronized in phase.
                #
                self.publish_targets([
                    target,
                    target,
                ])

                rate.sleep()

        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings,
            )

    def shutdown(self):
        if self.shutdown_done:
            return

        self.shutdown_done = True

        try:
            self.torque_off()
            rospy.sleep(0.2)
        except Exception:
            pass


def main():
    rospy.init_node(
        "dual_servo_oscillation"
    )

    controller = DualServoOscillation()

    print("")
    print("Waiting for servo states...")

    timeout = rospy.Time.now().to_sec() + 10.0

    rate = rospy.Rate(20)

    while (
        not rospy.is_shutdown()
        and not controller.states_available()
    ):
        if (
            rospy.Time.now().to_sec()
            > timeout
        ):
            print(
                "ERROR: Could not receive states "
                "from both servo index 0 and 1."
            )
            return

        rate.sleep()

    if rospy.is_shutdown():
        return

    controller.print_current_states()

    # Check servo hardware errors before enabling torque
    for index in controller.servo_indices:
        error = controller.current_errors.get(
            index,
            -1,
        )

        if error != 0:
            print(
                f"ERROR: Servo index {index} "
                f"reports error={error}."
            )
            print(
                "Motion is cancelled."
            )
            return

    print(
        "Both servos are ready."
    )

    print(
        f"They will move to "
        f"{controller.center_deg:.1f} deg, "
        f"then oscillate +/-"
        f"{controller.amplitude_deg:.1f} deg."
    )

    print("")
    command = input(
        "Press ENTER to start "
        "(type q + ENTER to cancel): "
    )

    if command.strip().lower() == "q":
        print("Cancelled.")
        return

    try:
        controller.move_to_center()
        controller.oscillate()

    finally:
        controller.shutdown()

    print("Torque OFF. Program finished.")


if __name__ == "__main__":
    main()
