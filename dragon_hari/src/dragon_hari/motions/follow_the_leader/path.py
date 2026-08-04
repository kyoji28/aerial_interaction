#!/usr/bin/env python3

import math

from .geometry import (
    LINK_BODY_LENGTH_M,
    LINK_ORIGIN_SPACING_M,
    calculate_joint_yaws_from_points,
    normalize_angle
    )


def calculate_distance(point1, point2):
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]

    return math.hypot(delta_x, delta_y)


def generate_sine_path(
    path_length,
    sample_interval,
    amplitude,
    wavelength,
):
    number_of_steps = int(
        round(path_length / sample_interval)
    )

    path_points = []

    for index in range(number_of_steps + 1):
        x = index * sample_interval

        y = amplitude * math.sin(
            2.0 * math.pi * x / wavelength
        )

        path_points.append((x, y))

    return path_points


def find_previous_point_at_distance(
    path_points,
    reference_point,
    start_index,
    target_distance,
):
    recent_point = reference_point

    for older_index in range(start_index, -1, -1):
        older_point = path_points[older_index]

        segment_x = older_point[0] - recent_point[0]
        segment_y = older_point[1] - recent_point[1]

        relative_x = recent_point[0] - reference_point[0]
        relative_y = recent_point[1] - reference_point[1]

        coefficient_a = (
            segment_x**2
            + segment_y**2
        )

        if coefficient_a < 1.0e-12:
            recent_point = older_point
            continue

        coefficient_b = 2.0 * (
            relative_x * segment_x
            + relative_y * segment_y
        )

        coefficient_c = (
            relative_x**2
            + relative_y**2
            - target_distance**2
        )

        discriminant = (
            coefficient_b**2
            - 4.0
            * coefficient_a
            * coefficient_c
        )

        if discriminant >= -1.0e-12:
            discriminant = max(
                discriminant,
                0.0,
            )

            square_root = math.sqrt(
                discriminant
            )

            ratios = [
                (
                    -coefficient_b
                    - square_root
                )
                / (2.0 * coefficient_a),
                (
                    -coefficient_b
                    + square_root
                )
                / (2.0 * coefficient_a),
            ]

            valid_ratios = [
                ratio
                for ratio in ratios
                if 0.0 <= ratio <= 1.0
            ]

            if valid_ratios:
                ratio = min(valid_ratios)

                target_point = (
                    recent_point[0]
                    + ratio * segment_x,
                    recent_point[1]
                    + ratio * segment_y,
                )

                return target_point, older_index

        recent_point = older_point

    raise ValueError(
        "The path does not contain enough history."
    )

def calculate_follow_the_leader_points(
    path_points,
    end_tip_index,
):
    end_tip = path_points[end_tip_index]

    link4_origin, search_index = (
        find_previous_point_at_distance(
            path_points=path_points,
            reference_point=end_tip,
            start_index=end_tip_index - 1,
            target_distance=LINK_BODY_LENGTH_M,
        )
    )

    link3_origin, search_index = (
        find_previous_point_at_distance(
            path_points=path_points,
            reference_point=link4_origin,
            start_index=search_index,
            target_distance=LINK_ORIGIN_SPACING_M,
        )
    )

    link2_origin, search_index = (
        find_previous_point_at_distance(
            path_points=path_points,
            reference_point=link3_origin,
            start_index=search_index,
            target_distance=LINK_ORIGIN_SPACING_M,
        )
    )

    link1_origin, search_index = (
        find_previous_point_at_distance(
            path_points=path_points,
            reference_point=link2_origin,
            start_index=search_index,
            target_distance=LINK_ORIGIN_SPACING_M,
        )
    )

    return {
        "link1_origin": link1_origin,
        "link2_origin": link2_origin,
        "link3_origin": link3_origin,
        "link4_origin": link4_origin,
        "end_tip": end_tip,
    }


def calculate_follow_the_leader_sequence(path_points):
    sequence = []

    for end_tip_index in range(len(path_points)):
        try:
            points = calculate_follow_the_leader_points(
                path_points=path_points,
                end_tip_index=end_tip_index,
            )
        except ValueError:
            continue

        yaw_values = calculate_joint_yaws_from_points(
            points
        )

        sequence.append(
            {
                "end_tip_index": end_tip_index,
                "points": points,
                "joint_yaws": {
                    "joint1_yaw": yaw_values[
                        "joint1_yaw"
                    ],
                    "joint2_yaw": yaw_values[
                        "joint2_yaw"
                    ],
                    "joint3_yaw": yaw_values[
                        "joint3_yaw"
                    ],
                },
            }
        )

    return sequence


def calculate_max_step_change(
    sequence,
    joint_name,
):
    maximum_change = 0.0

    for previous_state, current_state in zip(
        sequence,
        sequence[1:],
    ):
        previous_yaw = previous_state[
            "joint_yaws"
        ][joint_name]

        current_yaw = current_state[
            "joint_yaws"
        ][joint_name]

        change = abs(
            normalize_angle(
                current_yaw - previous_yaw
            )
        )

        maximum_change = max(
            maximum_change,
            change,
        )

    return maximum_change

def main():
    path_points = generate_sine_path(
        path_length=4.0,
        sample_interval=0.01,
        amplitude=0.25,
        wavelength=3.0,
    )

    points = calculate_follow_the_leader_points(
        path_points=path_points,
        end_tip_index=len(path_points) - 1,
    )

    print("Follow-the-leader points")

    for name, point in points.items():
        print(
            f"{name:12s}: "
            f"({point[0]:.4f}, "
            f"{point[1]:.4f})"
        )

    segment_definitions = [
        (
            "link1-link2",
            "link1_origin",
            "link2_origin",
        ),
        (
            "link2-link3",
            "link2_origin",
            "link3_origin",
        ),
        (
            "link3-link4",
            "link3_origin",
            "link4_origin",
        ),
        (
            "link4-end",
            "link4_origin",
            "end_tip",
        ),
    ]

    print()
    print("Segment distances")

    for (
        segment_name,
        start_name,
        end_name,
    ) in segment_definitions:
        distance = calculate_distance(
            points[start_name],
            points[end_name],
        )

        print(
            f"{segment_name:12s}: "
            f"{distance:.4f} m"
        )

    recovered_yaws = (
        calculate_joint_yaws_from_points(
            points
        )
    )

    print()
    print("Joint yaw angles")

    for joint_name in [
        "joint1_yaw",
        "joint2_yaw",
        "joint3_yaw",
    ]:
        print(
            f"{joint_name:12s}: "
            f"{math.degrees(recovered_yaws[joint_name]):.3f} deg"
        )

    sequence = calculate_follow_the_leader_sequence(
        path_points
    )

    print()
    print("Sequence validation")
    print(
        f"valid states : {len(sequence)}"
    )
    print(
        "first index  : "
        f"{sequence[0]['end_tip_index']}"
    )
    print(
        "last index   : "
        f"{sequence[-1]['end_tip_index']}"
    )

    print()
    print("Joint yaw ranges and step changes")

    for joint_name in [
        "joint1_yaw",
        "joint2_yaw",
        "joint3_yaw",
    ]:
        yaw_values = [
            state["joint_yaws"][joint_name]
            for state in sequence
        ]

        minimum_yaw = min(yaw_values)
        maximum_yaw = max(yaw_values)

        maximum_step_change = (
            calculate_max_step_change(
                sequence,
                joint_name,
            )
        )

        print(
            f"{joint_name:12s}: "
            f"range=["
            f"{math.degrees(minimum_yaw):7.3f}, "
            f"{math.degrees(maximum_yaw):7.3f}] deg, "
            f"max step="
            f"{math.degrees(maximum_step_change):.3f} deg"
        )


if __name__ == "__main__":
    main()