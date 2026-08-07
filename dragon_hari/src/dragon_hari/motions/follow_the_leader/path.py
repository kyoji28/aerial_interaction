#!/usr/bin/env python3

import math

from .geometry import (
    LINK_BODY_LENGTH_M,
    LINK_ORIGIN_SPACING_M,
    calculate_joint_yaws_from_points,
    normalize_angle,
)


DEFAULT_SAMPLE_INTERVAL_M = 0.01
DEFAULT_AMPLITUDE_M = 0.25
DEFAULT_NOMINAL_WAVELENGTH_M = 3.0
DEFAULT_HISTORY_LENGTH_M = 2.30


def calculate_distance(point1, point2):
    """Return the planar distance between two points."""
    return math.hypot(
        point2[0] - point1[0],
        point2[1] - point1[1],
    )


def calculate_half_wave_count(axis_length, nominal_wavelength):
    """Choose an integer half-wave count so start and goal are zero crossings."""
    estimated_count = 2.0 * axis_length / nominal_wavelength
    return max(1, int(math.floor(estimated_count + 0.5)))


def build_longitudinal_samples(
    axis_length,
    history_length,
    sample_interval,
):
    """Create longitudinal samples from path history through the goal."""
    history_count = int(
        math.floor(history_length / sample_interval + 1.0e-9)
    )
    history = [
        -index * sample_interval
        for index in range(history_count, 0, -1)
    ]

    if not history or not math.isclose(
        history[0],
        -history_length,
        abs_tol=1.0e-12,
    ):
        history.insert(0, -history_length)
    else:
        history[0] = -history_length

    forward_count = int(
        math.floor(axis_length / sample_interval + 1.0e-9)
    )
    forward = [
        index * sample_interval
        for index in range(forward_count + 1)
    ]

    if math.isclose(
        forward[-1],
        axis_length,
        abs_tol=1.0e-12,
    ):
        forward[-1] = axis_length
    else:
        forward.append(axis_length)

    longitudinal_positions = history + forward
    start_index = len(history)
    goal_index = len(longitudinal_positions) - 1

    return longitudinal_positions, start_index, goal_index


def generate_sine_path(
    start_point,
    goal_point,
    sample_interval=DEFAULT_SAMPLE_INTERVAL_M,
    amplitude=DEFAULT_AMPLITUDE_M,
    nominal_wavelength=DEFAULT_NOMINAL_WAVELENGTH_M,
    history_length=DEFAULT_HISTORY_LENGTH_M,
):
    """
    Generate a zero-phase sine path around the start-goal axis.

    The same sine path is extended behind the start point so the initial
    follow-the-leader posture can be constructed from path history.
    """
    if sample_interval <= 0.0:
        raise ValueError("sample_interval must be positive.")
    if amplitude < 0.0:
        raise ValueError("amplitude must be non-negative.")
    if nominal_wavelength <= 0.0:
        raise ValueError("nominal_wavelength must be positive.")
    if history_length <= 0.0:
        raise ValueError("history_length must be positive.")

    dx = goal_point[0] - start_point[0]
    dy = goal_point[1] - start_point[1]
    axis_length = math.hypot(dx, dy)

    if axis_length < 1.0e-6:
        raise ValueError("start_point and goal_point must be different.")

    axis_x = dx / axis_length
    axis_y = dy / axis_length
    normal_x = -axis_y
    normal_y = axis_x

    half_wave_count = calculate_half_wave_count(
        axis_length=axis_length,
        nominal_wavelength=nominal_wavelength,
    )
    actual_wavelength = 2.0 * axis_length / half_wave_count

    (
        longitudinal_positions,
        start_index,
        goal_index,
    ) = build_longitudinal_samples(
        axis_length=axis_length,
        history_length=history_length,
        sample_interval=sample_interval,
    )

    path_points = []

    for longitudinal in longitudinal_positions:
        lateral = amplitude * math.sin(
            2.0 * math.pi * longitudinal / actual_wavelength
        )

        world_x = (
            start_point[0]
            + longitudinal * axis_x
            + lateral * normal_x
        )
        world_y = (
            start_point[1]
            + longitudinal * axis_y
            + lateral * normal_y
        )

        path_points.append((world_x, world_y))

    # Remove tiny floating-point errors at the two required endpoints.
    path_points[start_index] = tuple(start_point)
    path_points[goal_index] = tuple(goal_point)

    return {
        "points": path_points,
        "start_index": start_index,
        "goal_index": goal_index,
        "axis_length": axis_length,
        "half_wave_count": half_wave_count,
        "actual_wavelength": actual_wavelength,
        "amplitude": amplitude,
        "sample_interval": sample_interval,
        "history_length": history_length,
    }


def find_previous_point_at_distance(
    path_points,
    reference_point,
    start_index,
    target_distance,
):
    """
    Find the previous point on the piecewise-linear path whose Euclidean
    distance from reference_point equals target_distance.
    """
    if target_distance <= 0.0:
        raise ValueError("target_distance must be positive.")
    if not path_points:
        raise ValueError("path_points must not be empty.")
    if not 0 <= start_index < len(path_points):
        raise ValueError("The path does not contain enough history.")

    recent_point = reference_point

    for older_index in range(start_index, -1, -1):
        older_point = path_points[older_index]

        segment_x = older_point[0] - recent_point[0]
        segment_y = older_point[1] - recent_point[1]
        relative_x = recent_point[0] - reference_point[0]
        relative_y = recent_point[1] - reference_point[1]

        coefficient_a = segment_x**2 + segment_y**2

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
            - 4.0 * coefficient_a * coefficient_c
        )

        if discriminant >= -1.0e-12:
            square_root = math.sqrt(max(discriminant, 0.0))
            ratios = [
                (-coefficient_b - square_root) / (2.0 * coefficient_a),
                (-coefficient_b + square_root) / (2.0 * coefficient_a),
            ]
            valid_ratios = [
                ratio
                for ratio in ratios
                if 0.0 <= ratio <= 1.0
            ]

            if valid_ratios:
                ratio = min(valid_ratios)
                return (
                    recent_point[0] + ratio * segment_x,
                    recent_point[1] + ratio * segment_y,
                ), older_index

        recent_point = older_point

    raise ValueError("The path does not contain enough history.")


def calculate_follow_the_leader_points(
    path_points,
    end_tip_index,
):
    """
    Place Link 4 -> Link 1 behind the end tip on the same path while
    preserving the fixed physical link lengths.
    """
    if not 0 <= end_tip_index < len(path_points):
        raise ValueError("end_tip_index is out of range.")

    end_tip = path_points[end_tip_index]

    link4_origin, search_index = find_previous_point_at_distance(
        path_points=path_points,
        reference_point=end_tip,
        start_index=end_tip_index - 1,
        target_distance=LINK_BODY_LENGTH_M,
    )
    link3_origin, search_index = find_previous_point_at_distance(
        path_points=path_points,
        reference_point=link4_origin,
        start_index=search_index,
        target_distance=LINK_ORIGIN_SPACING_M,
    )
    link2_origin, search_index = find_previous_point_at_distance(
        path_points=path_points,
        reference_point=link3_origin,
        start_index=search_index,
        target_distance=LINK_ORIGIN_SPACING_M,
    )
    link1_origin, _ = find_previous_point_at_distance(
        path_points=path_points,
        reference_point=link2_origin,
        start_index=search_index,
        target_distance=LINK_ORIGIN_SPACING_M,
    )

    return {
        "link1_origin": link1_origin,
        "link2_origin": link2_origin,
        "link3_origin": link3_origin,
        "link4_origin": link4_origin,
        "end_tip": end_tip,
    }


def calculate_follow_the_leader_sequence(
    path_points,
    start_index,
    goal_index,
):
    """Generate the FTL geometry while the end tip moves start -> goal."""
    if not 0 <= start_index <= goal_index < len(path_points):
        raise ValueError("Invalid start_index or goal_index.")

    sequence = []

    for end_tip_index in range(start_index, goal_index + 1):
        points = calculate_follow_the_leader_points(
            path_points=path_points,
            end_tip_index=end_tip_index,
        )
        yaw_values = calculate_joint_yaws_from_points(points)

        sequence.append(
            {
                "end_tip_index": end_tip_index,
                "points": points,
                "joint_yaws": {
                    "joint1_yaw": yaw_values["joint1_yaw"],
                    "joint2_yaw": yaw_values["joint2_yaw"],
                    "joint3_yaw": yaw_values["joint3_yaw"],
                },
            }
        )

    return sequence


def calculate_max_step_change(sequence, joint_name):
    """Return the largest yaw change between adjacent FTL samples."""
    maximum_change = 0.0

    for previous_state, current_state in zip(sequence, sequence[1:]):
        previous_yaw = previous_state["joint_yaws"][joint_name]
        current_yaw = current_state["joint_yaws"][joint_name]

        change = abs(
            normalize_angle(current_yaw - previous_yaw)
        )
        maximum_change = max(maximum_change, change)

    return maximum_change


def main():
    """Minimal geometry check for this module."""
    path = generate_sine_path(
        start_point=(0.0, 0.0),
        goal_point=(3.0, 0.0),
    )

    points = path["points"]
    initial_points = calculate_follow_the_leader_points(
        path_points=points,
        end_tip_index=path["start_index"],
    )

    print("Sine path")
    print(f"  start             : {points[path['start_index']]}")
    print(f"  goal              : {points[path['goal_index']]}")
    print(f"  half-wave count   : {path['half_wave_count']}")
    print(f"  actual wavelength : {path['actual_wavelength']:.4f} m")

    print()
    print("Initial FTL segment distances")
    for name, point1, point2 in [
        ("link1-link2", "link1_origin", "link2_origin"),
        ("link2-link3", "link2_origin", "link3_origin"),
        ("link3-link4", "link3_origin", "link4_origin"),
        ("link4-end", "link4_origin", "end_tip"),
    ]:
        distance = calculate_distance(
            initial_points[point1],
            initial_points[point2],
        )
        print(f"  {name:12s}: {distance:.4f} m")


if __name__ == "__main__":
    main()