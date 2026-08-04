#!/usr/bin/env python3

import math
import time

import rospy
import tf2_ros

from aerial_robot_msgs.msg import FlightNav

from dragon_hari.motions.follow_the_leader.pose import (
    calculate_cog_pose,
    quaternion_to_yaw,
)
from dragon_hari.motions.follow_the_leader.geometry import (
    normalize_angle,
)
from dragon_hari.motions.follow_the_leader.joint_player import (
    FollowTheLeaderJointDemo,
)
from dragon_hari.motions.follow_the_leader.trajectory import (
    build_timed_trajectory,
)


NAV_TOPIC = "/dragon/uav/nav"

WORLD_FRAME = "world"
LINK1_FRAME = "dragon/link1"
COG_FRAME = "dragon/cog"


def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(maximum, value),
    )


def transform_plan_pose_to_world(
    local_x,
    local_y,
    local_yaw,
    alignment_x,
    alignment_y,
    alignment_yaw,
):
    cosine_yaw = math.cos(alignment_yaw)
    sine_yaw = math.sin(alignment_yaw)

    world_x = (
        alignment_x
        + cosine_yaw * local_x
        - sine_yaw * local_y
    )

    world_y = (
        alignment_y
        + sine_yaw * local_x
        + cosine_yaw * local_y
    )

    world_yaw = normalize_angle(
        alignment_yaw + local_yaw
    )

    return world_x, world_y, world_yaw


class FollowTheLeaderCombinedDemo:
    def __init__(self):
        self.joint_demo = (
            FollowTheLeaderJointDemo()
        )

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(10.0)
        )

        self.tf_listener = (
            tf2_ros.TransformListener(
                self.tf_buffer
            )
        )

        self.nav_publisher = rospy.Publisher(
            NAV_TOPIC,
            FlightNav,
            queue_size=1,
        )

        self.last_navigation_target = None

        rospy.on_shutdown(
            self.stop_navigation
        )

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        timeout,
    ):
        start_time = time.monotonic()
        last_error = None

        while not rospy.is_shutdown():
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as error:
                last_error = error

            if (
                time.monotonic() - start_time
                > timeout
            ):
                raise RuntimeError(
                    "Timed out while looking up "
                    f"{target_frame} <- "
                    f"{source_frame}: "
                    f"{last_error}"
                )

            rospy.sleep(0.01)

        raise rospy.ROSInterruptException(
            "ROS shutdown while waiting "
            "for a transform."
        )

    def get_plan_alignment(
        self,
        first_command,
        transform_timeout,
    ):
        transform = self.lookup_transform(
            target_frame=WORLD_FRAME,
            source_frame=LINK1_FRAME,
            timeout=transform_timeout,
        )

        translation = (
            transform.transform.translation
        )

        actual_link1_yaw = quaternion_to_yaw(
            transform.transform.rotation
        )

        alignment_yaw = normalize_angle(
            actual_link1_yaw
            - first_command["link1_yaw"]
        )

        cosine_yaw = math.cos(alignment_yaw)
        sine_yaw = math.sin(alignment_yaw)

        rotated_first_x = (
            cosine_yaw
            * first_command["link1_x"]
            - sine_yaw
            * first_command["link1_y"]
        )

        rotated_first_y = (
            sine_yaw
            * first_command["link1_x"]
            + cosine_yaw
            * first_command["link1_y"]
        )

        alignment_x = (
            translation.x - rotated_first_x
        )

        alignment_y = (
            translation.y - rotated_first_y
        )

        return (
            alignment_x,
            alignment_y,
            alignment_yaw,
        )

    def get_link1_to_cog(
        self,
        transform_timeout,
    ):
        transform = self.lookup_transform(
            target_frame=LINK1_FRAME,
            source_frame=COG_FRAME,
            timeout=transform_timeout,
        )

        translation = (
            transform.transform.translation
        )

        yaw = quaternion_to_yaw(
            transform.transform.rotation
        )

        return (
            translation.x,
            translation.y,
            yaw,
        )

    def publish_navigation_command(
        self,
        target_x,
        target_y,
        target_yaw,
        target_velocity_x,
        target_velocity_y,
        target_omega_z,
    ):
        message = FlightNav()

        message.header.stamp = rospy.Time.now()
        message.header.frame_id = WORLD_FRAME

        message.control_frame = (
            FlightNav.WORLD_FRAME
        )

        message.target = FlightNav.COG

        message.pos_xy_nav_mode = (
            FlightNav.POS_VEL_MODE
        )

        message.target_pos_x = target_x
        message.target_pos_y = target_y

        message.target_vel_x = (
            target_velocity_x
        )

        message.target_vel_y = (
            target_velocity_y
        )

        message.target_acc_x = 0.0
        message.target_acc_y = 0.0

        message.yaw_nav_mode = (
            FlightNav.POS_VEL_MODE
        )

        message.target_yaw = target_yaw
        message.target_omega_z = (
            target_omega_z
        )

        message.pos_z_nav_mode = (
            FlightNav.NO_NAVIGATION
        )

        self.last_navigation_target = {
            "x": target_x,
            "y": target_y,
            "yaw": target_yaw,
        }

        self.nav_publisher.publish(message)

    def play(
        self,
        trajectory,
        control_rate,
        playback_duration,
        final_hold_duration,
        transform_timeout,
        maximum_xy_velocity,
        maximum_yaw_rate,
    ):
        (
            alignment_x,
            alignment_y,
            alignment_yaw,
        ) = self.get_plan_alignment(
            first_command=trajectory[0],
            transform_timeout=transform_timeout,
        )

        rospy.loginfo(
            "Plan alignment: "
            "x=%.3f, y=%.3f, yaw=%.2f deg",
            alignment_x,
            alignment_y,
            math.degrees(alignment_yaw),
        )

        rate = rospy.Rate(control_rate)

        previous_target = None
        last_command = None
        last_target = None

        log_interval = max(
            1,
            int(round(control_rate)),
        )

        for sample_index, command in enumerate(
            trajectory
        ):
            if rospy.is_shutdown():
                break

            if (
                playback_duration > 0.0
                and command["time"]
                > playback_duration
            ):
                break

            self.joint_demo.publish_joint_positions(
                command["joint_positions"]
            )

            (
                cog_offset_x,
                cog_offset_y,
                cog_offset_yaw,
            ) = self.get_link1_to_cog(
                transform_timeout=transform_timeout
            )

            (
                target_link1_x,
                target_link1_y,
                target_link1_yaw,
            ) = transform_plan_pose_to_world(
                local_x=command["link1_x"],
                local_y=command["link1_y"],
                local_yaw=command["link1_yaw"],
                alignment_x=alignment_x,
                alignment_y=alignment_y,
                alignment_yaw=alignment_yaw,
            )

            (
                target_cog_x,
                target_cog_y,
                target_cog_yaw,
            ) = calculate_cog_pose(
                link1_x=target_link1_x,
                link1_y=target_link1_y,
                link1_yaw=target_link1_yaw,
                cog_offset_x=cog_offset_x,
                cog_offset_y=cog_offset_y,
                cog_offset_yaw=cog_offset_yaw,
            )

            target_velocity_x = 0.0
            target_velocity_y = 0.0
            target_omega_z = 0.0

            if previous_target is not None:
                delta_time = (
                    command["time"]
                    - previous_target["time"]
                )

                if delta_time > 0.0:
                    target_velocity_x = (
                        target_cog_x
                        - previous_target["x"]
                    ) / delta_time

                    target_velocity_y = (
                        target_cog_y
                        - previous_target["y"]
                    ) / delta_time

                    target_omega_z = (
                        normalize_angle(
                            target_cog_yaw
                            - previous_target["yaw"]
                        )
                        / delta_time
                    )

            target_velocity_x = clamp(
                target_velocity_x,
                -maximum_xy_velocity,
                maximum_xy_velocity,
            )

            target_velocity_y = clamp(
                target_velocity_y,
                -maximum_xy_velocity,
                maximum_xy_velocity,
            )

            target_omega_z = clamp(
                target_omega_z,
                -maximum_yaw_rate,
                maximum_yaw_rate,
            )

            self.publish_navigation_command(
                target_x=target_cog_x,
                target_y=target_cog_y,
                target_yaw=target_cog_yaw,
                target_velocity_x=target_velocity_x,
                target_velocity_y=target_velocity_y,
                target_omega_z=target_omega_z,
            )

            previous_target = {
                "time": command["time"],
                "x": target_cog_x,
                "y": target_cog_y,
                "yaw": target_cog_yaw,
            }

            last_command = command
            last_target = previous_target

            if sample_index % log_interval == 0:
                rospy.loginfo(
                    "Combined playback: "
                    "%.2f / %.2f s",
                    command["time"],
                    playback_duration,
                )

            rate.sleep()

        if (
            last_command is None
            or last_target is None
        ):
            raise RuntimeError(
                "No combined trajectory "
                "sample was published."
            )

        rospy.loginfo(
            "Holding the final command "
            "for %.2f seconds.",
            final_hold_duration,
        )

        hold_end_time = (
            rospy.Time.now()
            + rospy.Duration(
                final_hold_duration
            )
        )

        while (
            not rospy.is_shutdown()
            and rospy.Time.now()
            < hold_end_time
        ):
            self.joint_demo.publish_joint_positions(
                last_command[
                    "joint_positions"
                ]
            )

            self.publish_navigation_command(
                target_x=last_target["x"],
                target_y=last_target["y"],
                target_yaw=last_target["yaw"],
                target_velocity_x=0.0,
                target_velocity_y=0.0,
                target_omega_z=0.0,
            )

            rate.sleep()

    def stop_navigation(self):
        if self.last_navigation_target is None:
            return

        rospy.loginfo(
            "Stopping navigation velocity command."
        )

        for _ in range(3):
            self.publish_navigation_command(
                target_x=self.last_navigation_target["x"],
                target_y=self.last_navigation_target["y"],
                target_yaw=self.last_navigation_target["yaw"],
                target_velocity_x=0.0,
                target_velocity_y=0.0,
                target_omega_z=0.0,
            )

            time.sleep(0.05)


def main():
    rospy.init_node(
        "follow_the_leader_combined_demo"
    )

    end_tip_speed = rospy.get_param(
        "~end_tip_speed",
        0.025,
    )

    control_rate = rospy.get_param(
        "~control_rate",
        20.0,
    )

    transition_duration = rospy.get_param(
        "~transition_duration",
        5.0,
    )

    initial_hold_duration = rospy.get_param(
        "~initial_hold_duration",
        1.0,
    )

    playback_duration = rospy.get_param(
        "~playback_duration",
        5.0,
    )

    final_hold_duration = rospy.get_param(
        "~final_hold_duration",
        2.0,
    )

    transform_timeout = rospy.get_param(
        "~transform_timeout",
        1.0,
    )

    joint_state_timeout = rospy.get_param(
        "~joint_state_timeout",
        10.0,
    )

    maximum_xy_velocity = rospy.get_param(
        "~maximum_xy_velocity",
        0.15,
    )

    maximum_yaw_rate_deg = rospy.get_param(
        "~maximum_yaw_rate_deg",
        15.0,
    )

    if control_rate <= 0.0:
        raise ValueError(
            "control_rate must be positive."
        )

    if end_tip_speed <= 0.0:
        raise ValueError(
            "end_tip_speed must be positive."
        )

    trajectory, _, _ = build_timed_trajectory(
        end_tip_speed=end_tip_speed,
        control_rate=control_rate,
    )

    demo = FollowTheLeaderCombinedDemo()

    current_joint_positions = (
        demo.joint_demo.wait_for_joint_states(
            timeout=joint_state_timeout
        )
    )

    first_joint_positions = trajectory[0][
        "joint_positions"
    ]

    demo.joint_demo.interpolate_to_first_command(
        start_positions=current_joint_positions,
        target_positions=first_joint_positions,
        duration=transition_duration,
        control_rate=control_rate,
    )

    demo.joint_demo.hold_command(
        joint_positions=first_joint_positions,
        duration=initial_hold_duration,
        control_rate=control_rate,
    )

    rospy.loginfo(
        "Starting combined follow-the-leader "
        "playback for %.2f seconds.",
        playback_duration,
    )

    demo.play(
        trajectory=trajectory,
        control_rate=control_rate,
        playback_duration=playback_duration,
        final_hold_duration=final_hold_duration,
        transform_timeout=transform_timeout,
        maximum_xy_velocity=maximum_xy_velocity,
        maximum_yaw_rate=math.radians(
            maximum_yaw_rate_deg
        ),
    )

    rospy.loginfo(
        "Combined follow-the-leader "
        "playback completed."
    )


if __name__ == "__main__":
    try:
        main()
    except (
        rospy.ROSInterruptException,
        RuntimeError,
        ValueError,
    ) as error:
        rospy.logerr(str(error))