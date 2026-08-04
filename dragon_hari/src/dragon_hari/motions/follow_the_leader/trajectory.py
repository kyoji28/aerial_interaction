#!/usr/bin/env python3

import csv
import math

from .geometry import (
    calculate_joint_yaws_from_points,
    normalize_angle,
)
from .path import (
    calculate_distance,
    calculate_follow_the_leader_sequence,
    generate_sine_path,
)
from .timing import (
    remove_short_final_interval,
    resample_path_by_arc_length,
)


JOINT_NAMES = [
    "joint1_pitch",
    "joint1_yaw",
    "joint2_pitch",
    "joint2_yaw",
    "joint3_pitch",
    "joint3_yaw",
]


def build_timed_trajectory(
    end_tip_speed,
    control_rate,
):
    if end_tip_speed <= 0.0:
        raise ValueError(
            "end_tip_speed must be positive."
        )

    if control_rate <= 0.0:
        raise ValueError(
            "control_rate must be positive."
        )

    time_step = 1.0 / control_rate

    sample_spacing = (
        end_tip_speed
        * time_step
    )

    original_path = generate_sine_path(
        path_length=4.0,
        sample_interval=0.01,
        amplitude=0.25,
        wavelength=3.0,
    )

    resampled_path = (
        resample_path_by_arc_length(
            path_points=original_path,
            sample_spacing=sample_spacing,
        )
    )

    regular_path = (
        remove_short_final_interval(
            path_points=resampled_path,
            sample_spacing=sample_spacing,
        )
    )

    geometric_sequence = (
        calculate_follow_the_leader_sequence(
            regular_path
        )
    )

    trajectory = []

    for sample_index, state in enumerate(
        geometric_sequence
    ):
        points = state["points"]

        yaw_values = (
            calculate_joint_yaws_from_points(
                points
            )
        )

        joint_positions = [
            0.0,
            yaw_values["joint1_yaw"],
            0.0,
            yaw_values["joint2_yaw"],
            0.0,
            yaw_values["joint3_yaw"],
        ]

        link1_origin = points[
            "link1_origin"
        ]

        end_tip = points["end_tip"]

        trajectory.append(
            {
                "time": (
                    sample_index
                    * time_step
                ),
                "link1_x": link1_origin[0],
                "link1_y": link1_origin[1],
                "link1_yaw": yaw_values[
                    "link1_yaw"
                ],
                "end_tip_x": end_tip[0],
                "end_tip_y": end_tip[1],
                "joint_positions": (
                    joint_positions
                ),
            }
        )

    return (
        trajectory,
        time_step,
        sample_spacing,
    )


def validate_trajectory(
    trajectory,
    time_step,
    sample_spacing,
):
    if len(trajectory) < 2:
        raise ValueError(
            "At least two trajectory "
            "samples are required."
        )

    maximum_time_step_error = 0.0
    maximum_end_tip_step_error = 0.0
    maximum_link1_step = 0.0
    maximum_link1_yaw_step = 0.0

    maximum_joint_steps = [
        0.0,
        0.0,
        0.0,
    ]

    yaw_indices = [
        1,
        3,
        5,
    ]

    for previous, current in zip(
        trajectory,
        trajectory[1:],
    ):
        actual_time_step = (
            current["time"]
            - previous["time"]
        )

        maximum_time_step_error = max(
            maximum_time_step_error,
            abs(
                actual_time_step
                - time_step
            ),
        )

        previous_end_tip = (
            previous["end_tip_x"],
            previous["end_tip_y"],
        )

        current_end_tip = (
            current["end_tip_x"],
            current["end_tip_y"],
        )

        end_tip_step = calculate_distance(
            previous_end_tip,
            current_end_tip,
        )

        maximum_end_tip_step_error = max(
            maximum_end_tip_step_error,
            abs(
                end_tip_step
                - sample_spacing
            ),
        )

        previous_link1 = (
            previous["link1_x"],
            previous["link1_y"],
        )

        current_link1 = (
            current["link1_x"],
            current["link1_y"],
        )

        link1_step = calculate_distance(
            previous_link1,
            current_link1,
        )

        maximum_link1_step = max(
            maximum_link1_step,
            link1_step,
        )

        link1_yaw_step = abs(
            normalize_angle(
                current["link1_yaw"]
                - previous["link1_yaw"]
            )
        )

        maximum_link1_yaw_step = max(
            maximum_link1_yaw_step,
            link1_yaw_step,
        )

        for result_index, joint_index in enumerate(
            yaw_indices
        ):
            joint_step = abs(
                normalize_angle(
                    current[
                        "joint_positions"
                    ][joint_index]
                    - previous[
                        "joint_positions"
                    ][joint_index]
                )
            )

            maximum_joint_steps[
                result_index
            ] = max(
                maximum_joint_steps[
                    result_index
                ],
                joint_step,
            )

    return {
        "maximum_time_step_error":
            maximum_time_step_error,
        "maximum_end_tip_step_error":
            maximum_end_tip_step_error,
        "maximum_link1_step":
            maximum_link1_step,
        "maximum_link1_yaw_step":
            maximum_link1_yaw_step,
        "maximum_joint_steps":
            maximum_joint_steps,
    }


def write_trajectory_csv(
    trajectory,
    output_path,
):
    field_names = [
        "time_s",
        "link1_x_m",
        "link1_y_m",
        "link1_yaw_rad",
        "end_tip_x_m",
        "end_tip_y_m",
        "joint1_pitch_rad",
        "joint1_yaw_rad",
        "joint2_pitch_rad",
        "joint2_yaw_rad",
        "joint3_pitch_rad",
        "joint3_yaw_rad",
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

        for command in trajectory:
            joint_positions = command[
                "joint_positions"
            ]

            writer.writerow(
                {
                    "time_s":
                        command["time"],
                    "link1_x_m":
                        command["link1_x"],
                    "link1_y_m":
                        command["link1_y"],
                    "link1_yaw_rad":
                        command["link1_yaw"],
                    "end_tip_x_m":
                        command["end_tip_x"],
                    "end_tip_y_m":
                        command["end_tip_y"],
                    "joint1_pitch_rad":
                        joint_positions[0],
                    "joint1_yaw_rad":
                        joint_positions[1],
                    "joint2_pitch_rad":
                        joint_positions[2],
                    "joint2_yaw_rad":
                        joint_positions[3],
                    "joint3_pitch_rad":
                        joint_positions[4],
                    "joint3_yaw_rad":
                        joint_positions[5],
                }
            )


def print_command(
    title,
    command,
):
    print(title)

    print(
        "  time      : "
        f"{command['time']:.3f} s"
    )

    print(
        "  Link 1    : "
        f"({command['link1_x']:.4f}, "
        f"{command['link1_y']:.4f}), "
        f"yaw="
        f"{math.degrees(command['link1_yaw']):.3f} deg"
    )

    print(
        "  end tip   : "
        f"({command['end_tip_x']:.4f}, "
        f"{command['end_tip_y']:.4f})"
    )

    print(
        "  joint yaw : "
        f"["
        f"{math.degrees(command['joint_positions'][1]):.3f}, "
        f"{math.degrees(command['joint_positions'][3]):.3f}, "
        f"{math.degrees(command['joint_positions'][5]):.3f}"
        f"] deg"
    )


def main():
    end_tip_speed = 0.10
    control_rate = 20.0

    (
        trajectory,
        time_step,
        sample_spacing,
    ) = build_timed_trajectory(
        end_tip_speed=end_tip_speed,
        control_rate=control_rate,
    )

    validation = validate_trajectory(
        trajectory=trajectory,
        time_step=time_step,
        sample_spacing=sample_spacing,
    )

    output_path = (
        "/tmp/"
        "follow_the_leader_trajectory.csv"
    )

    write_trajectory_csv(
        trajectory=trajectory,
        output_path=output_path,
    )

    print("Trajectory command sequence")
    print(
        f"samples     : {len(trajectory)}"
    )
    print(
        "duration    : "
        f"{trajectory[-1]['time']:.3f} s"
    )
    print(
        "joint order : "
        + ", ".join(JOINT_NAMES)
    )

    print()

    print_command(
        "First command",
        trajectory[0],
    )

    print()

    print_command(
        "Last command",
        trajectory[-1],
    )

    print()
    print("Continuity validation")

    print(
        "max time-step error : "
        f"{validation['maximum_time_step_error']:.12f} s"
    )

    print(
        "max end-tip error   : "
        f"{validation['maximum_end_tip_step_error']:.12f} m"
    )

    print(
        "max Link 1 step     : "
        f"{validation['maximum_link1_step']:.6f} m"
    )

    maximum_link1_yaw_step_deg = math.degrees(
        validation["maximum_link1_yaw_step"]
    )

    print(
        "max Link 1 yaw step : "
        f"{maximum_link1_yaw_step_deg:.6f} deg"
    )

    for joint_number, maximum_step in enumerate(
        validation["maximum_joint_steps"],
        start=1,
    ):
        print(
            f"max joint{joint_number} yaw step: "
            f"{math.degrees(maximum_step):.6f} deg"
        )

    print()
    print(
        f"Saved trajectory to {output_path}"
    )


if __name__ == "__main__":
    main()