#!/usr/bin/env python3

import csv
import math

from .geometry import (
    calculate_joint_yaws_from_points,
    normalize_angle,
)
from .path import (
    DEFAULT_AMPLITUDE_M,
    DEFAULT_HISTORY_LENGTH_M,
    DEFAULT_NOMINAL_WAVELENGTH_M,
    DEFAULT_SAMPLE_INTERVAL_M,
    calculate_distance,
    calculate_follow_the_leader_sequence,
    generate_sine_path,
)
from .timing import (
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


def build_resampled_path_with_history(
    path,
    sample_spacing,
):
    """
    Resample only the start-to-goal part of the path while preserving
    the original path history behind the start point.

    The history is required to construct the initial positions of
    Link 4 -> Link 1, but the timed end-tip motion starts exactly at
    start_index.
    """
    path_points = path["points"]
    original_start_index = path["start_index"]
    original_goal_index = path["goal_index"]

    history_points = path_points[
        :original_start_index
    ]

    forward_points = path_points[
        original_start_index:
        original_goal_index + 1
    ]

    if len(forward_points) < 2:
        raise ValueError(
            "The start-to-goal path must contain "
            "at least two points."
        )

    resampled_forward_points = (
        resample_path_by_arc_length(
            path_points=forward_points,
            sample_spacing=sample_spacing,
        )
    )

    if len(resampled_forward_points) < 2:
        raise ValueError(
            "The resampled start-to-goal path must "
            "contain at least two points."
        )

    # Preserve the exact world start and goal coordinates.
    resampled_forward_points[0] = tuple(
        forward_points[0]
    )
    resampled_forward_points[-1] = tuple(
        forward_points[-1]
    )

    timed_path_points = (
        history_points
        + resampled_forward_points
    )

    timed_start_index = len(
        history_points
    )

    timed_goal_index = (
        len(timed_path_points) - 1
    )

    return (
        timed_path_points,
        timed_start_index,
        timed_goal_index,
    )


def build_timed_trajectory(
    start_point,
    goal_point,
    end_tip_speed,
    control_rate,
    path_sample_interval=DEFAULT_SAMPLE_INTERVAL_M,
    amplitude=DEFAULT_AMPLITUDE_M,
    nominal_wavelength=DEFAULT_NOMINAL_WAVELENGTH_M,
    history_length=DEFAULT_HISTORY_LENGTH_M,
):
    """
    Build a time-indexed planar follow-the-leader trajectory.

    start_point and goal_point are world-frame positions of the end tip.
    The end tip follows the sine path at approximately constant speed,
    while Link 4 -> Link 1 follow the same path behind it.
    """
    if end_tip_speed <= 0.0:
        raise ValueError(
            "end_tip_speed must be positive."
        )

    if control_rate <= 0.0:
        raise ValueError(
            "control_rate must be positive."
        )

    time_step = (
        1.0 / control_rate
    )

    sample_spacing = (
        end_tip_speed
        * time_step
    )

    path = generate_sine_path(
        start_point=start_point,
        goal_point=goal_point,
        sample_interval=path_sample_interval,
        amplitude=amplitude,
        nominal_wavelength=nominal_wavelength,
        history_length=history_length,
    )

    (
        timed_path_points,
        timed_start_index,
        timed_goal_index,
    ) = build_resampled_path_with_history(
        path=path,
        sample_spacing=sample_spacing,
    )

    geometric_sequence = (
        calculate_follow_the_leader_sequence(
            path_points=timed_path_points,
            start_index=timed_start_index,
            goal_index=timed_goal_index,
        )
    )

    if len(geometric_sequence) < 2:
        raise ValueError(
            "At least two trajectory samples "
            "are required."
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

        end_tip = points[
            "end_tip"
        ]

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
    start_point=None,
    goal_point=None,
):
    if len(trajectory) < 2:
        raise ValueError(
            "At least two trajectory "
            "samples are required."
        )

    maximum_time_step_error = 0.0

    maximum_regular_end_tip_step_error = (
        0.0
    )

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

    trajectory_pairs = list(
        zip(
            trajectory,
            trajectory[1:],
        )
    )

    for pair_index, (
        previous,
        current,
    ) in enumerate(trajectory_pairs):
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

        # The last interval can be shorter than
        # sample_spacing so that the exact goal is retained.
        if pair_index < len(
            trajectory_pairs
        ) - 1:
            maximum_regular_end_tip_step_error = (
                max(
                    maximum_regular_end_tip_step_error,
                    abs(
                        end_tip_step
                        - sample_spacing
                    ),
                )
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

        for (
            result_index,
            joint_index,
        ) in enumerate(yaw_indices):
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

    final_end_tip_step = calculate_distance(
        (
            trajectory[-2]["end_tip_x"],
            trajectory[-2]["end_tip_y"],
        ),
        (
            trajectory[-1]["end_tip_x"],
            trajectory[-1]["end_tip_y"],
        ),
    )

    start_error = None

    if start_point is not None:
        start_error = calculate_distance(
            (
                trajectory[0]["end_tip_x"],
                trajectory[0]["end_tip_y"],
            ),
            start_point,
        )

    goal_error = None

    if goal_point is not None:
        goal_error = calculate_distance(
            (
                trajectory[-1]["end_tip_x"],
                trajectory[-1]["end_tip_y"],
            ),
            goal_point,
        )

    return {
        "maximum_time_step_error":
            maximum_time_step_error,
        "maximum_regular_end_tip_step_error":
            maximum_regular_end_tip_step_error,
        "final_end_tip_step":
            final_end_tip_step,
        "maximum_link1_step":
            maximum_link1_step,
        "maximum_link1_yaw_step":
            maximum_link1_yaw_step,
        "maximum_joint_steps":
            maximum_joint_steps,
        "start_error":
            start_error,
        "goal_error":
            goal_error,
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
        "yaw="
        f"{math.degrees(command['link1_yaw']):.3f} deg"
    )

    print(
        "  end tip   : "
        f"({command['end_tip_x']:.4f}, "
        f"{command['end_tip_y']:.4f})"
    )

    print(
        "  joint yaw : "
        "["
        f"{math.degrees(command['joint_positions'][1]):.3f}, "
        f"{math.degrees(command['joint_positions'][3]):.3f}, "
        f"{math.degrees(command['joint_positions'][5]):.3f}"
        "] deg"
    )


def main():
    start_point = (
        0.0,
        0.0,
    )

    goal_point = (
        3.0,
        0.0,
    )

    end_tip_speed = 0.20
    control_rate = 20.0

    (
        trajectory,
        time_step,
        sample_spacing,
    ) = build_timed_trajectory(
        start_point=start_point,
        goal_point=goal_point,
        end_tip_speed=end_tip_speed,
        control_rate=control_rate,
    )

    validation = validate_trajectory(
        trajectory=trajectory,
        time_step=time_step,
        sample_spacing=sample_spacing,
        start_point=start_point,
        goal_point=goal_point,
    )

    output_path = (
        "/tmp/"
        "follow_the_leader_trajectory.csv"
    )

    write_trajectory_csv(
        trajectory=trajectory,
        output_path=output_path,
    )

    print("Trajectory configuration")
    print(
        "start point  : "
        f"({start_point[0]:.4f}, "
        f"{start_point[1]:.4f})"
    )
    print(
        "goal point   : "
        f"({goal_point[0]:.4f}, "
        f"{goal_point[1]:.4f})"
    )
    print(
        "end-tip speed: "
        f"{end_tip_speed:.3f} m/s"
    )
    print(
        "control rate : "
        f"{control_rate:.1f} Hz"
    )
    print(
        "time step    : "
        f"{time_step:.3f} s"
    )
    print(
        "path spacing : "
        f"{sample_spacing:.4f} m"
    )

    print()
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
    print("Trajectory validation")

    print(
        "max time-step error       : "
        f"{validation['maximum_time_step_error']:.12f} s"
    )

    print(
        "max regular end-tip error : "
        f"{validation['maximum_regular_end_tip_step_error']:.12f} m"
    )

    print(
        "final end-tip step        : "
        f"{validation['final_end_tip_step']:.6f} m"
    )

    print(
        "start error               : "
        f"{validation['start_error']:.12f} m"
    )

    print(
        "goal error                : "
        f"{validation['goal_error']:.12f} m"
    )

    print(
        "max Link 1 step           : "
        f"{validation['maximum_link1_step']:.6f} m"
    )

    maximum_link1_yaw_step_deg = (
        math.degrees(
            validation[
                "maximum_link1_yaw_step"
            ]
        )
    )

    print(
        "max Link 1 yaw step       : "
        f"{maximum_link1_yaw_step_deg:.6f} deg"
    )

    for (
        joint_number,
        maximum_step,
    ) in enumerate(
        validation[
            "maximum_joint_steps"
        ],
        start=1,
    ):
        print(
            f"max joint{joint_number} yaw step      : "
            f"{math.degrees(maximum_step):.6f} deg"
        )

    print()
    print(
        f"Saved trajectory to {output_path}"
    )


if __name__ == "__main__":
    main()