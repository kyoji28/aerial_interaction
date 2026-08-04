#!/usr/bin/env python3

import csv
import math
import time

import rospy
import tf2_ros

from dragon_hari.motions.follow_the_leader.geometry import (
    normalize_angle,
)
from dragon_hari.motions.follow_the_leader.pose import (
    calculate_cog_pose,
    quaternion_to_yaw,
)
from dragon_hari.motions.follow_the_leader.joint_player import (
    FollowTheLeaderJointDemo,
)
from dragon_hari.motions.follow_the_leader.path import (
    calculate_distance,
)
from dragon_hari.motions.follow_the_leader.trajectory import (
    build_timed_trajectory,
)


LINK1_FRAME = "dragon/link1"
COG_FRAME = "dragon/cog"


class CogMappingRecorder:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(10.0)
        )

        self.tf_listener = (
            tf2_ros.TransformListener(
                self.tf_buffer
            )
        )

    def lookup_link1_to_cog(
        self,
        timeout,
    ):
        start_time = time.monotonic()
        last_error = None

        while not rospy.is_shutdown():
            try:
                transform = (
                    self.tf_buffer.lookup_transform(
                        LINK1_FRAME,
                        COG_FRAME,
                        rospy.Time(0),
                        rospy.Duration(0.05),
                    )
                )

                translation = (
                    transform.transform.translation
                )

                rotation = (
                    transform.transform.rotation
                )

                return {
                    "x": translation.x,
                    "y": translation.y,
                    "z": translation.z,
                    "yaw": quaternion_to_yaw(
                        rotation
                    ),
                }

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
                    f"{LINK1_FRAME} -> {COG_FRAME}: "
                    f"{last_error}"
                )

            rospy.sleep(0.01)

        raise rospy.ROSInterruptException(
            "ROS shutdown while waiting "
            "for the CoG transform."
        )

    def record_trajectory(
        self,
        demo,
        trajectory,
        control_rate,
        transform_timeout,
    ):
        records = []
        rate = rospy.Rate(control_rate)

        log_interval = max(
            1,
            int(round(control_rate)),
        )

        rospy.loginfo(
            "Recording Link 1 to CoG mapping: "
            "%d samples at %.1f Hz.",
            len(trajectory),
            control_rate,
        )

        for sample_index, command in enumerate(
            trajectory
        ):
            if rospy.is_shutdown():
                break

            demo.publish_joint_positions(
                command["joint_positions"]
            )

            rate.sleep()

            actual_joint_positions = (
                demo.get_current_joint_positions()
            )

            if actual_joint_positions is None:
                raise RuntimeError(
                    "Actual joint positions "
                    "are unavailable."
                )

            cog_offset = (
                self.lookup_link1_to_cog(
                    timeout=transform_timeout
                )
            )

            cog_x, cog_y, cog_yaw = (
                calculate_cog_pose(
                    link1_x=command["link1_x"],
                    link1_y=command["link1_y"],
                    link1_yaw=command[
                        "link1_yaw"
                    ],
                    cog_offset_x=cog_offset["x"],
                    cog_offset_y=cog_offset["y"],
                    cog_offset_yaw=cog_offset[
                        "yaw"
                    ],
                )
            )

            records.append(
                {
                    "time": command["time"],
                    "link1_x": command["link1_x"],
                    "link1_y": command["link1_y"],
                    "link1_yaw": command[
                        "link1_yaw"
                    ],
                    "cog_offset_x": cog_offset[
                        "x"
                    ],
                    "cog_offset_y": cog_offset[
                        "y"
                    ],
                    "cog_offset_z": cog_offset[
                        "z"
                    ],
                    "cog_offset_yaw": cog_offset[
                        "yaw"
                    ],
                    "cog_x": cog_x,
                    "cog_y": cog_y,
                    "cog_yaw": cog_yaw,

                    "command_joint1_yaw":
                        command[
                            "joint_positions"
                        ][1],
                    "command_joint2_yaw":
                        command[
                            "joint_positions"
                        ][3],
                    "command_joint3_yaw":
                        command[
                            "joint_positions"
                        ][5],
                    "actual_joint1_yaw":
                        actual_joint_positions[1],
                    "actual_joint2_yaw":
                        actual_joint_positions[3],
                    "actual_joint3_yaw":
                        actual_joint_positions[5],
                }
            )

            if sample_index % log_interval == 0:
                rospy.loginfo(
                    "Mapping progress: %.2f / %.2f s",
                    command["time"],
                    trajectory[-1]["time"],
                )

        return records


def validate_records(records):
    if len(records) < 2:
        raise ValueError(
            "At least two mapping records "
            "are required."
        )

    maximum_cog_step = 0.0
    maximum_cog_yaw_step = 0.0

    for previous, current in zip(
        records,
        records[1:],
    ):
        previous_cog = (
            previous["cog_x"],
            previous["cog_y"],
        )

        current_cog = (
            current["cog_x"],
            current["cog_y"],
        )

        cog_step = calculate_distance(
            previous_cog,
            current_cog,
        )

        maximum_cog_step = max(
            maximum_cog_step,
            cog_step,
        )

        cog_yaw_step = abs(
            normalize_angle(
                current["cog_yaw"]
                - previous["cog_yaw"]
            )
        )

        maximum_cog_yaw_step = max(
            maximum_cog_yaw_step,
            cog_yaw_step,
        )

    return {
        "maximum_cog_step":
            maximum_cog_step,
        "maximum_cog_yaw_step":
            maximum_cog_yaw_step,
    }


def write_mapping_csv(
    records,
    output_path,
):
    field_names = [
        "time_s",
        "link1_x_m",
        "link1_y_m",
        "link1_yaw_rad",
        "cog_offset_x_m",
        "cog_offset_y_m",
        "cog_offset_z_m",
        "cog_offset_yaw_rad",
        "cog_x_m",
        "cog_y_m",
        "cog_yaw_rad",
        "command_joint1_yaw_rad",
        "command_joint2_yaw_rad",
        "command_joint3_yaw_rad",
        "actual_joint1_yaw_rad",
        "actual_joint2_yaw_rad",
        "actual_joint3_yaw_rad",
    ]

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=field_names,
        )

        writer.writeheader()

        for record in records:
            writer.writerow(
                {
                    "time_s":
                        record["time"],
                    "link1_x_m":
                        record["link1_x"],
                    "link1_y_m":
                        record["link1_y"],
                    "link1_yaw_rad":
                        record["link1_yaw"],
                    "cog_offset_x_m":
                        record["cog_offset_x"],
                    "cog_offset_y_m":
                        record["cog_offset_y"],
                    "cog_offset_z_m":
                        record["cog_offset_z"],
                    "cog_offset_yaw_rad":
                        record[
                            "cog_offset_yaw"
                        ],
                    "cog_x_m":
                        record["cog_x"],
                    "cog_y_m":
                        record["cog_y"],
                    "cog_yaw_rad":
                        record["cog_yaw"],
                    "command_joint1_yaw_rad":
                        record[
                            "command_joint1_yaw"
                        ],
                    "command_joint2_yaw_rad":
                        record[
                            "command_joint2_yaw"
                        ],
                    "command_joint3_yaw_rad":
                        record[
                            "command_joint3_yaw"
                        ],
                    "actual_joint1_yaw_rad":
                        record[
                            "actual_joint1_yaw"
                        ],
                    "actual_joint2_yaw_rad":
                        record[
                            "actual_joint2_yaw"
                        ],
                    "actual_joint3_yaw_rad":
                        record[
                            "actual_joint3_yaw"
                        ],
                }
            )


def print_range(
    label,
    values,
    unit,
):
    print(
        f"{label:20s}: "
        f"[{min(values):.6f}, "
        f"{max(values):.6f}] {unit}"
    )


def main():
    rospy.init_node(
        "follow_the_leader_cog_mapping"
    )

    end_tip_speed = rospy.get_param(
        "~end_tip_speed",
        0.05,
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

    transform_timeout = rospy.get_param(
        "~transform_timeout",
        1.0,
    )

    joint_state_timeout = rospy.get_param(
        "~joint_state_timeout",
        10.0,
    )

    trajectory, _, _ = build_timed_trajectory(
        end_tip_speed=end_tip_speed,
        control_rate=control_rate,
    )

    demo = FollowTheLeaderJointDemo()
    recorder = CogMappingRecorder()

    current_joint_positions = (
        demo.wait_for_joint_states(
            timeout=joint_state_timeout
        )
    )

    first_joint_positions = trajectory[0][
        "joint_positions"
    ]

    demo.interpolate_to_first_command(
        start_positions=current_joint_positions,
        target_positions=first_joint_positions,
        duration=transition_duration,
        control_rate=control_rate,
    )

    demo.hold_command(
        joint_positions=first_joint_positions,
        duration=initial_hold_duration,
        control_rate=control_rate,
    )

    records = recorder.record_trajectory(
        demo=demo,
        trajectory=trajectory,
        control_rate=control_rate,
        transform_timeout=transform_timeout,
    )

    validation = validate_records(
        records
    )

    output_path = (
        "/tmp/"
        "follow_the_leader_cog_mapping.csv"
    )

    write_mapping_csv(
        records=records,
        output_path=output_path,
    )

    print()
    print("CoG mapping result")
    print(
        f"samples             : {len(records)}"
    )

    print_range(
        "CoG offset x",
        [
            record["cog_offset_x"]
            for record in records
        ],
        "m",
    )

    print_range(
        "CoG offset y",
        [
            record["cog_offset_y"]
            for record in records
        ],
        "m",
    )

    offset_yaw_degrees = [
        math.degrees(
            record["cog_offset_yaw"]
        )
        for record in records
    ]

    print_range(
        "CoG offset yaw",
        offset_yaw_degrees,
        "deg",
    )

    maximum_cog_yaw_step_deg = (
        math.degrees(
            validation[
                "maximum_cog_yaw_step"
            ]
        )
    )

    print(
        "max CoG position step: "
        f"{validation['maximum_cog_step']:.6f} m"
    )

    print(
        "max CoG yaw step     : "
        f"{maximum_cog_yaw_step_deg:.6f} deg"
    )

    print(
        f"Saved mapping to {output_path}"
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