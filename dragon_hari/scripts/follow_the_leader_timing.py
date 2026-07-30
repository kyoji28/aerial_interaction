#!/usr/bin/env python3

import math

from follow_the_leader_geometry import (
    normalize_angle,
)
from follow_the_leader_path import (
    calculate_distance,
    calculate_follow_the_leader_sequence,
    generate_sine_path,
)


def calculate_cumulative_path_lengths(
    path_points,
):
    if len(path_points) < 2:
        raise ValueError(
            "At least two path points are required."
        )

    cumulative_lengths = [0.0]

    for previous_point, current_point in zip(
        path_points,
        path_points[1:],
    ):
        segment_length = calculate_distance(
            previous_point,
            current_point,
        )

        cumulative_lengths.append(
            cumulative_lengths[-1]
            + segment_length
        )

    return cumulative_lengths


def resample_path_by_arc_length(
    path_points,
    sample_spacing,
):
    if sample_spacing <= 0.0:
        raise ValueError(
            "sample_spacing must be positive."
        )

    cumulative_lengths = (
        calculate_cumulative_path_lengths(
            path_points
        )
    )

    total_path_length = cumulative_lengths[-1]

    number_of_full_intervals = int(
        math.floor(
            total_path_length
            / sample_spacing
        )
    )

    target_lengths = [
        index * sample_spacing
        for index in range(
            number_of_full_intervals + 1
        )
    ]

    remaining_length = (
        total_path_length
        - target_lengths[-1]
    )

    if remaining_length > 1.0e-9:
        target_lengths.append(
            total_path_length
        )

    resampled_points = []
    segment_index = 0

    for target_length in target_lengths:
        while (
            segment_index
            < len(path_points) - 2
            and cumulative_lengths[
                segment_index + 1
            ]
            < target_length
        ):
            segment_index += 1

        segment_start_length = (
            cumulative_lengths[
                segment_index
            ]
        )

        segment_end_length = (
            cumulative_lengths[
                segment_index + 1
            ]
        )

        segment_length = (
            segment_end_length
            - segment_start_length
        )

        if segment_length < 1.0e-12:
            resampled_points.append(
                path_points[segment_index]
            )
            continue

        interpolation_ratio = (
            target_length
            - segment_start_length
        ) / segment_length

        start_point = path_points[
            segment_index
        ]

        end_point = path_points[
            segment_index + 1
        ]

        interpolated_point = (
            start_point[0]
            + interpolation_ratio
            * (
                end_point[0]
                - start_point[0]
            ),
            start_point[1]
            + interpolation_ratio
            * (
                end_point[1]
                - start_point[1]
            ),
        )

        resampled_points.append(
            interpolated_point
        )

    return resampled_points

def remove_short_final_interval(
    path_points,
    sample_spacing,
):
    if len(path_points) < 2:
        raise ValueError(
            "At least two path points are required."
        )

    final_interval = calculate_distance(
        path_points[-2],
        path_points[-1],
    )

    if (
        abs(final_interval - sample_spacing)
        > 1.0e-6
    ):
        return path_points[:-1]

    return path_points


def unwrap_angle_sequence(angle_values):
    if not angle_values:
        return []

    unwrapped_values = [
        angle_values[0]
    ]

    previous_wrapped_angle = (
        angle_values[0]
    )

    for current_wrapped_angle in (
        angle_values[1:]
    ):
        angle_change = normalize_angle(
            current_wrapped_angle
            - previous_wrapped_angle
        )

        unwrapped_values.append(
            unwrapped_values[-1]
            + angle_change
        )

        previous_wrapped_angle = (
            current_wrapped_angle
        )

    return unwrapped_values


def calculate_finite_difference(
    values,
    time_step,
):
    if time_step <= 0.0:
        raise ValueError(
            "time_step must be positive."
        )

    return [
        (
            current_value
            - previous_value
        )
        / time_step
        for previous_value, current_value
        in zip(
            values,
            values[1:],
        )
    ]

def main():
    end_tip_speed = 0.10
    control_rate = 20.0

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

    sequence = (
        calculate_follow_the_leader_sequence(
            regular_path
        )
    )

    if len(sequence) < 3:
        raise ValueError(
            "The trajectory is too short "
            "to calculate derivatives."
        )

    trajectory_duration = (
        len(sequence) - 1
    ) * time_step

    end_tip_speeds = [
        calculate_distance(
            previous_state["points"][
                "end_tip"
            ],
            current_state["points"][
                "end_tip"
            ],
        )
        / time_step
        for previous_state, current_state
        in zip(
            sequence,
            sequence[1:],
        )
    ]

    print("Timing configuration")
    print(
        "end-tip speed : "
        f"{end_tip_speed:.3f} m/s"
    )
    print(
        "control rate  : "
        f"{control_rate:.1f} Hz"
    )
    print(
        "time step     : "
        f"{time_step:.3f} s"
    )
    print(
        "path spacing  : "
        f"{sample_spacing:.4f} m"
    )

    print()
    print("Trajectory information")
    print(
        "valid states  : "
        f"{len(sequence)}"
    )
    print(
        "first index   : "
        f"{sequence[0]['end_tip_index']}"
    )
    print(
        "last index    : "
        f"{sequence[-1]['end_tip_index']}"
    )
    print(
        "duration      : "
        f"{trajectory_duration:.3f} s"
    )
    print(
        "speed min     : "
        f"{min(end_tip_speeds):.6f} m/s"
    )
    print(
        "speed max     : "
        f"{max(end_tip_speeds):.6f} m/s"
    )

    print()
    print(
        "Joint angle, velocity, "
        "and acceleration"
    )

    for joint_name in [
        "joint1_yaw",
        "joint2_yaw",
        "joint3_yaw",
    ]:
        wrapped_angles = [
            state["joint_yaws"][
                joint_name
            ]
            for state in sequence
        ]

        joint_angles = (
            unwrap_angle_sequence(
                wrapped_angles
            )
        )

        joint_velocities = (
            calculate_finite_difference(
                values=joint_angles,
                time_step=time_step,
            )
        )

        joint_accelerations = (
            calculate_finite_difference(
                values=joint_velocities,
                time_step=time_step,
            )
        )

        minimum_angle = min(
            joint_angles
        )

        maximum_angle = max(
            joint_angles
        )

        maximum_velocity = max(
            abs(value)
            for value in joint_velocities
        )

        maximum_acceleration = max(
            abs(value)
            for value
            in joint_accelerations
        )

        print(
            f"{joint_name}"
        )

        print(
            "  angle range      : "
            f"["
            f"{math.degrees(minimum_angle):.3f}, "
            f"{math.degrees(maximum_angle):.3f}"
            f"] deg"
        )

        print(
            "  max velocity     : "
            f"{math.degrees(maximum_velocity):.3f} "
            "deg/s"
        )

        print(
            "  max acceleration : "
            f"{math.degrees(maximum_acceleration):.3f} "
            "deg/s^2"
        )



if __name__ == "__main__":
    main()