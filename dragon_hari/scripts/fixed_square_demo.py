#!/usr/bin/env python3

"""Hold DRAGON's square posture while executing one guarded horizontal goal."""

import math
import threading

import rospy
import tf2_geometry_msgs  # noqa: F401  Registers geometry_msgs TF conversions.
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8


class FixedSquareDemo:
    ARM_OFF_STATE = 0
    HOVER_STATE = 5

    IDLE = "IDLE"
    PREPARING_SQUARE = "PREPARING_SQUARE"
    MOVING = "MOVING"

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
        self._lock = threading.RLock()

        self.joint_publish_rate = self._positive_param(
            "~joint_publish_rate", 10.0
        )
        self.joint_tolerance = self._positive_param("~joint_tolerance", 0.05)
        self.square_settle_duration = self._positive_param(
            "~square_settle_duration", 1.0
        )
        self.square_timeout = self._positive_param("~square_timeout", 15.0)
        self.state_timeout = self._positive_param("~state_timeout", 0.5)
        self.max_goal_distance = self._positive_param(
            "~max_goal_distance", 3.0
        )
        self.arrival_position_tolerance = self._positive_param(
            "~arrival_position_tolerance", 0.10
        )
        self.arrival_settle_duration = self._positive_param(
            "~arrival_settle_duration", 1.0
        )
        self.motion_timeout = self._positive_param("~motion_timeout", 60.0)
        self.trajectory_mean_vel = self._positive_param(
            "~trajectory_mean_vel", 0.2
        )
        self.tf_timeout = self._positive_param("~tf_timeout", 0.25)

        self.mode = self.IDLE
        self.flight_state = None
        self.flight_state_rx_time = None
        self.cog_state = None
        self.cog_state_rx_time = None
        self.baselink_state = None
        self.baselink_state_rx_time = None
        self.joint_positions = None
        self.joint_state_rx_time = None

        self.goal_world_xy = None
        self.square_start_time = None
        self.square_settle_start = None
        self.motion_start_time = None
        self.arrival_settle_start = None
        self.start_cog_state = None
        self.start_baselink_yaw = None
        self.navigation_goal_count = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.joint_command_pub = rospy.Publisher(
            "/dragon/joints_ctrl", JointState, queue_size=10
        )
        self.navigation_goal_pub = rospy.Publisher(
            "/dragon/target_pose", PoseStamped, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "/dragon_hari/fixed_square/status",
            String,
            queue_size=1,
            latch=True,
        )

        self.goal_sub = rospy.Subscriber(
            "/dragon_hari/demo_goal",
            PoseStamped,
            self._goal_callback,
            queue_size=1,
        )
        self.cog_odom_sub = rospy.Subscriber(
            "/dragon/uav/cog/odom",
            Odometry,
            self._cog_odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.baselink_odom_sub = rospy.Subscriber(
            "/dragon/uav/baselink/odom",
            Odometry,
            self._baselink_odom_callback,
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

        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.joint_publish_rate), self._control_tick
        )
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self._status_tick)

        navigator_velocity = rospy.get_param(
            "/dragon/navigation/trajectory_mean_vel", None
        )
        if navigator_velocity is None:
            rospy.logwarn(
                "The BaseNavigator trajectory_mean_vel parameter is not set"
            )
        elif not math.isclose(
            float(navigator_velocity), self.trajectory_mean_vel, rel_tol=0.0, abs_tol=1e-9
        ):
            rospy.logwarn(
                "Node trajectory_mean_vel %.3f differs from BaseNavigator %.3f",
                self.trajectory_mean_vel,
                float(navigator_velocity),
            )

        rospy.loginfo(
            "Fixed Square demo ready; no robot command is published until a valid "
            "goal is received while flight_state=%d",
            self.HOVER_STATE,
        )
        self._publish_status("startup")

    @staticmethod
    def _positive_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and greater than zero".format(name))
        return value

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
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _angle_error(current, reference):
        return math.atan2(
            math.sin(current - reference), math.cos(current - reference)
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

        with self._lock:
            self.cog_state = (position.x, position.y, position.z, yaw)
            self.cog_state_rx_time = rospy.Time.now()

    def _baselink_odom_callback(self, message):
        try:
            position = message.pose.pose.position
            values = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("position contains a non-finite value")
            yaw = self._yaw_from_quaternion(message.pose.pose.orientation)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Invalid baselink odometry: %s", error)
            return

        with self._lock:
            self.baselink_state = (position.x, position.y, position.z, yaw)
            self.baselink_state_rx_time = rospy.Time.now()

    def _joint_state_callback(self, message):
        positions = {}
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                positions[name] = position

        with self._lock:
            self.joint_positions = positions
            self.joint_state_rx_time = rospy.Time.now()

    def _flight_state_callback(self, message):
        with self._lock:
            self.flight_state = int(message.data)
            self.flight_state_rx_time = rospy.Time.now()
            if self.mode != self.IDLE and self.flight_state != self.HOVER_STATE:
                self._fail_locked(
                    "flight state changed from HOVER to {}".format(
                        self.flight_state
                    )
                )

    def _goal_callback(self, message):
        frame_id = message.header.frame_id.strip()
        if not frame_id:
            self._reject_goal("frame_id is empty")
            return

        position_values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        if not all(math.isfinite(value) for value in position_values):
            self._reject_goal("position contains a non-finite value")
            return

        with self._lock:
            rejection = self._goal_state_rejection_locked(rospy.Time.now())
        if rejection is not None:
            self._reject_goal(rejection)
            return

        goal_world_xy = self._transform_goal_xy_to_world(message, frame_id)
        if goal_world_xy is None:
            return

        with self._lock:
            now = rospy.Time.now()
            rejection = self._goal_state_rejection_locked(now)
            if rejection is not None:
                self._reject_goal(rejection)
                return

            delta_x = goal_world_xy[0] - self.cog_state[0]
            delta_y = goal_world_xy[1] - self.cog_state[1]
            horizontal_distance = math.hypot(delta_x, delta_y)
            if horizontal_distance > self.max_goal_distance:
                self._reject_goal(
                    "horizontal distance {:.3f} m exceeds limit {:.3f} m".format(
                        horizontal_distance, self.max_goal_distance
                    )
                )
                return

            self.goal_world_xy = goal_world_xy
            self.square_start_time = now
            self.square_settle_start = None
            self.motion_start_time = None
            self.arrival_settle_start = None
            self.start_cog_state = None
            self.start_baselink_yaw = None
            self.mode = self.PREPARING_SQUARE
            rospy.loginfo(
                "Accepted Fixed Square goal at world x=%.3f y=%.3f; "
                "preparing Square posture",
                goal_world_xy[0],
                goal_world_xy[1],
            )
            self._publish_status_locked("goal_accepted")

    def _goal_state_rejection_locked(self, now):
        if self.mode != self.IDLE:
            return "another goal is already being processed"
        if self.flight_state != self.HOVER_STATE:
            return "flight state is not HOVER ({})".format(self.flight_state)
        stale = self._missing_or_stale_states_locked(now)
        if stale:
            return "state is missing or stale: {}".format(", ".join(stale))
        missing_joints = self._missing_square_joints_locked()
        if missing_joints:
            return "joint state is missing: {}".format(", ".join(missing_joints))
        return None

    def _transform_goal_xy_to_world(self, message, frame_id):
        normalized_frame = frame_id.lstrip("/")
        if not normalized_frame:
            self._reject_goal("frame_id is empty after normalization")
            return None
        if normalized_frame == "world":
            return (message.pose.position.x, message.pose.position.y)

        point = PointStamped()
        point.header.stamp = message.header.stamp
        point.header.frame_id = normalized_frame
        point.point = message.pose.position
        try:
            transformed = self.tf_buffer.transform(
                point, "world", rospy.Duration(self.tf_timeout)
            )
        except Exception as error:  # tf2 exception classes vary by ROS release.
            self._reject_goal(
                "cannot transform frame '{}' to world: {}".format(
                    frame_id, error
                )
            )
            return None

        transformed_xy = (transformed.point.x, transformed.point.y)
        if not all(math.isfinite(value) for value in transformed_xy):
            self._reject_goal("TF result contains a non-finite x or y value")
            return None
        return transformed_xy

    def _control_tick(self, _event):
        with self._lock:
            if self.mode == self.IDLE:
                return

            now = rospy.Time.now()
            if self.flight_state != self.HOVER_STATE:
                self._fail_locked(
                    "flight state is not HOVER ({})".format(self.flight_state)
                )
                return

            stale = self._missing_or_stale_states_locked(now)
            if stale:
                self._fail_locked(
                    "state became missing or stale: {}".format(", ".join(stale))
                )
                return

            missing_joints = self._missing_square_joints_locked()
            if missing_joints:
                self._fail_locked(
                    "joint state lost required joints: {}".format(
                        ", ".join(missing_joints)
                    )
                )
                return

            if self.mode == self.PREPARING_SQUARE:
                self._prepare_square_tick_locked(now)
            elif self.mode == self.MOVING:
                self._moving_tick_locked(now)

    def _prepare_square_tick_locked(self, now):
        if (now - self.square_start_time).to_sec() > self.square_timeout:
            self._fail_locked("Square posture preparation timed out")
            return

        self._publish_square_locked(now)
        max_joint_error = self._max_joint_error_locked()
        if max_joint_error > self.joint_tolerance:
            self.square_settle_start = None
            return

        if self.square_settle_start is None:
            self.square_settle_start = now
            return
        if (now - self.square_settle_start).to_sec() < self.square_settle_duration:
            return

        # Capture the latest states immediately before sending the one navigation goal.
        self.start_cog_state = tuple(self.cog_state)
        self.start_baselink_yaw = self.baselink_state[3]
        navigation_goal = PoseStamped()
        navigation_goal.header.stamp = now
        navigation_goal.header.frame_id = "world"
        navigation_goal.pose.position.x = self.goal_world_xy[0]
        navigation_goal.pose.position.y = self.goal_world_xy[1]
        navigation_goal.pose.position.z = self.start_cog_state[2]
        half_yaw = 0.5 * self.start_cog_state[3]
        navigation_goal.pose.orientation.z = math.sin(half_yaw)
        navigation_goal.pose.orientation.w = math.cos(half_yaw)

        self.mode = self.MOVING
        self.motion_start_time = now
        self.arrival_settle_start = None
        self.navigation_goal_count += 1
        self.navigation_goal_pub.publish(navigation_goal)
        rospy.loginfo(
            "Published navigation goal once: world x=%.3f y=%.3f z=%.3f, "
            "CoG yaw=%.3f, baselink yaw reference=%.3f, trajectory_mean_vel=%.3f",
            navigation_goal.pose.position.x,
            navigation_goal.pose.position.y,
            navigation_goal.pose.position.z,
            self.start_cog_state[3],
            self.start_baselink_yaw,
            self.trajectory_mean_vel,
        )
        self._publish_status_locked("navigation_goal_sent")

    def _moving_tick_locked(self, now):
        if (now - self.motion_start_time).to_sec() > self.motion_timeout:
            self._log_motion_errors_locked("Motion timed out")
            self._fail_locked("motion timed out; no cancellation command was sent")
            return

        self._publish_square_locked(now)
        horizontal_error = math.hypot(
            self.goal_world_xy[0] - self.cog_state[0],
            self.goal_world_xy[1] - self.cog_state[1],
        )
        if horizontal_error > self.arrival_position_tolerance:
            self.arrival_settle_start = None
            return

        if self.arrival_settle_start is None:
            self.arrival_settle_start = now
            return
        if (now - self.arrival_settle_start).to_sec() < self.arrival_settle_duration:
            return

        self._log_motion_errors_locked("Fixed Square goal succeeded")
        self.mode = self.IDLE
        self._publish_status_locked("success")
        rospy.loginfo("Fixed Square goal complete; joint command publishing stopped")

    def _publish_square_locked(self, now):
        command = JointState()
        command.header.stamp = now
        command.name = list(self.JOINT_NAMES)
        command.position = list(self.SQUARE_POSITIONS)
        self.joint_command_pub.publish(command)

    def _missing_or_stale_states_locked(self, now):
        states = (
            ("flight_state", self.flight_state, self.flight_state_rx_time),
            ("cog_odom", self.cog_state, self.cog_state_rx_time),
            ("baselink_odom", self.baselink_state, self.baselink_state_rx_time),
            ("joint_states", self.joint_positions, self.joint_state_rx_time),
        )
        missing_or_stale = []
        for name, value, receive_time in states:
            if value is None or receive_time is None:
                missing_or_stale.append(name + "(missing)")
            elif (now - receive_time).to_sec() > self.state_timeout:
                missing_or_stale.append(name + "(stale)")
        return missing_or_stale

    def _missing_square_joints_locked(self):
        if self.joint_positions is None:
            return list(self.JOINT_NAMES)
        return [
            name for name in self.JOINT_NAMES if name not in self.joint_positions
        ]

    def _max_joint_error_locked(self):
        return max(
            abs(self.joint_positions[name] - target)
            for name, target in zip(self.JOINT_NAMES, self.SQUARE_POSITIONS)
        )

    def _log_motion_errors_locked(self, prefix):
        if self.start_cog_state is None or self.start_baselink_yaw is None:
            rospy.logwarn("%s before motion reference states were captured", prefix)
            return
        rospy.loginfo(
            "%s: horizontal=%.3f m, height=%.3f m, CoG yaw=%.3f rad, "
            "baselink yaw=%.3f rad, max joint=%.3f rad",
            prefix,
            math.hypot(
                self.goal_world_xy[0] - self.cog_state[0],
                self.goal_world_xy[1] - self.cog_state[1],
            ),
            abs(self.cog_state[2] - self.start_cog_state[2]),
            abs(self._angle_error(self.cog_state[3], self.start_cog_state[3])),
            abs(self._angle_error(self.baselink_state[3], self.start_baselink_yaw)),
            self._max_joint_error_locked(),
        )

    def _fail_locked(self, reason):
        rospy.logerr("Fixed Square execution failed: %s", reason)
        self.mode = self.IDLE
        self.square_settle_start = None
        self.arrival_settle_start = None
        self._publish_status_locked("failure: " + reason)
        rospy.logerr(
            "Joint command publishing stopped; no navigation cancellation, landing, "
            "or other recovery command was sent"
        )

    def _reject_goal(self, reason):
        rospy.logwarn("Rejected Fixed Square goal: %s", reason)
        with self._lock:
            self._publish_status_locked("goal_rejected: " + reason)

    def _status_tick(self, _event):
        self._publish_status("heartbeat")

    def _publish_status(self, detail):
        with self._lock:
            self._publish_status_locked(detail)

    def _publish_status_locked(self, detail):
        now = rospy.Time.now()
        ready = not self._missing_or_stale_states_locked(now)
        message = String()
        message.data = (
            "state={} flight_state={} state_fresh={} navigation_goals_sent={} "
            "detail={}".format(
                self.mode,
                self.flight_state,
                str(ready).lower(),
                self.navigation_goal_count,
                detail,
            )
        )
        self.status_pub.publish(message)


def main():
    rospy.init_node("fixed_square_demo")
    try:
        FixedSquareDemo()
    except (TypeError, ValueError) as error:
        rospy.logfatal("Invalid Fixed Square configuration: %s", error)
        return
    rospy.spin()


if __name__ == "__main__":
    main()
