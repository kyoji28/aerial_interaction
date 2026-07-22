#!/usr/bin/env python3

"""Run one terminal-driven Fixed Square movement."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8


class FixedSquareDemo:
    HOVER_STATE = 5

    JOINT_PUBLISH_RATE = 10.0
    JOINT_TOLERANCE = 0.05
    SQUARE_SETTLE_DURATION = 1.0
    ARRIVAL_POSITION_TOLERANCE = 0.10
    ARRIVAL_SETTLE_DURATION = 1.0
    STATE_TIMEOUT = 0.5

    JOINT_NAMES = (
        "joint1_pitch",
        "joint1_yaw",
        "joint2_pitch",
        "joint2_yaw",
        "joint3_pitch",
        "joint3_yaw",
    )
    SQUARE_POSITIONS = (
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
    )

    def __init__(self):
        self.lock = threading.Lock()
        self.flight_state = None
        self.flight_state_rx_time = None
        self.cog_state = None
        self.cog_state_rx_time = None
        self.joint_positions = None
        self.joint_state_rx_time = None

        self.joint_command_pub = rospy.Publisher(
            "/dragon/joints_ctrl", JointState, queue_size=10
        )
        self.navigation_goal_pub = rospy.Publisher(
            "/dragon/target_pose", PoseStamped, queue_size=1
        )

        self.cog_odom_sub = rospy.Subscriber(
            "/dragon/uav/cog/odom",
            Odometry,
            self._cog_odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.joint_state_sub = rospy.Subscriber(
            "/dragon/joint_states",
            JointState,
            self._joint_state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.flight_state_sub = rospy.Subscriber(
            "/dragon/flight_state",
            UInt8,
            self._flight_state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        values = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("quaternion contains a non-finite value")
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-9:
            raise ValueError("quaternion norm is zero")
        x, y, z, w = (value / norm for value in values)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def _cog_odom_callback(self, message):
        try:
            position = message.pose.pose.position
            values = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("position contains a non-finite value")
            yaw = self._yaw_from_quaternion(message.pose.pose.orientation)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Invalid CoG odometry: %s", error)
            return

        with self.lock:
            self.cog_state = (position.x, position.y, position.z, yaw)
            self.cog_state_rx_time = rospy.Time.now()

    def _joint_state_callback(self, message):
        positions = {
            name: position
            for name, position in zip(message.name, message.position)
            if math.isfinite(position)
        }
        with self.lock:
            self.joint_positions = positions
            self.joint_state_rx_time = rospy.Time.now()

    def _flight_state_callback(self, message):
        with self.lock:
            self.flight_state = int(message.data)
            self.flight_state_rx_time = rospy.Time.now()

    def run(self):
        values = input("Enter target x y [m] in world: ").split()
        if len(values) != 2:
            raise ValueError("enter exactly two values: x y")
        goal_x, goal_y = (float(value) for value in values)
        if not math.isfinite(goal_x) or not math.isfinite(goal_y):
            raise ValueError("goal x and y must be finite")

        with self.lock:
            error = self._state_error_locked(rospy.Time.now())
        if error is not None:
            rospy.logerr("Fixed Square aborted: %s", error)
            return False

        rospy.loginfo(
            "Preparing Square posture for world goal x=%.3f y=%.3f",
            goal_x,
            goal_y,
        )
        rate = rospy.Rate(self.JOINT_PUBLISH_RATE)
        settle_start = None

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            with self.lock:
                error = self._state_error_locked(now)
                max_joint_error = (
                    self._max_joint_error_locked() if error is None else None
                )
            if error is not None:
                return self._abort(error)

            self._publish_square(now)
            if max_joint_error <= self.JOINT_TOLERANCE:
                if settle_start is None:
                    settle_start = now
                elif (now - settle_start).to_sec() >= self.SQUARE_SETTLE_DURATION:
                    break
            else:
                settle_start = None
            rate.sleep()

        if rospy.is_shutdown():
            return False

        now = rospy.Time.now()
        with self.lock:
            error = self._state_error_locked(now)
            cog_state = tuple(self.cog_state) if error is None else None
        if error is not None:
            return self._abort(error)

        navigation_goal = PoseStamped()
        navigation_goal.header.stamp = now
        navigation_goal.header.frame_id = "world"
        navigation_goal.pose.position.x = goal_x
        navigation_goal.pose.position.y = goal_y
        navigation_goal.pose.position.z = cog_state[2]
        half_yaw = 0.5 * cog_state[3]
        navigation_goal.pose.orientation.z = math.sin(half_yaw)
        navigation_goal.pose.orientation.w = math.cos(half_yaw)
        self.navigation_goal_pub.publish(navigation_goal)
        rospy.loginfo(
            "Published navigation goal once: world x=%.3f y=%.3f z=%.3f yaw=%.3f",
            goal_x,
            goal_y,
            cog_state[2],
            cog_state[3],
        )

        settle_start = None
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            with self.lock:
                error = self._state_error_locked(now)
                cog_xy = (
                    (self.cog_state[0], self.cog_state[1])
                    if error is None
                    else None
                )
            if error is not None:
                return self._abort(error)

            self._publish_square(now)
            position_error = math.hypot(goal_x - cog_xy[0], goal_y - cog_xy[1])
            if position_error <= self.ARRIVAL_POSITION_TOLERANCE:
                if settle_start is None:
                    settle_start = now
                elif (now - settle_start).to_sec() >= self.ARRIVAL_SETTLE_DURATION:
                    rospy.loginfo(
                        "Fixed Square goal reached; joint command publishing stopped"
                    )
                    return True
            else:
                settle_start = None
            rate.sleep()

        return False

    def _state_error_locked(self, now):
        if self.flight_state != self.HOVER_STATE:
            return "flight state is not HOVER ({})".format(self.flight_state)

        states = (
            ("cog_odom", self.cog_state, self.cog_state_rx_time),
            ("joint_states", self.joint_positions, self.joint_state_rx_time),
            ("flight_state", self.flight_state, self.flight_state_rx_time),
        )
        missing_or_stale = []
        for name, value, receive_time in states:
            if value is None or receive_time is None:
                missing_or_stale.append(name + "(missing)")
            elif (now - receive_time).to_sec() > self.STATE_TIMEOUT:
                missing_or_stale.append(name + "(stale)")
        if missing_or_stale:
            return "state is missing or stale: {}".format(
                ", ".join(missing_or_stale)
            )

        missing_joints = [
            name for name in self.JOINT_NAMES if name not in self.joint_positions
        ]
        if missing_joints:
            return "joint state is missing: {}".format(", ".join(missing_joints))
        return None

    def _max_joint_error_locked(self):
        return max(
            abs(self.joint_positions[name] - target)
            for name, target in zip(self.JOINT_NAMES, self.SQUARE_POSITIONS)
        )

    def _publish_square(self, now):
        command = JointState()
        command.header.stamp = now
        command.name = list(self.JOINT_NAMES)
        command.position = list(self.SQUARE_POSITIONS)
        self.joint_command_pub.publish(command)

    @staticmethod
    def _abort(reason):
        rospy.logerr(
            "Fixed Square aborted: %s. Joint commands stopped; no navigation "
            "cancellation, landing, or recovery command was sent",
            reason,
        )
        return False


def main():
    rospy.init_node("fixed_square_demo")
    try:
        FixedSquareDemo().run()
    except (EOFError, ValueError) as error:
        rospy.logerr("Fixed Square aborted: %s", error)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
