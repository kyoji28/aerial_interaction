#!/usr/bin/env python3

import matplotlib.pyplot as plt

from follow_the_leader_path import (
    calculate_follow_the_leader_sequence,
    generate_sine_path,
)


def extract_robot_coordinates(points):
    point_names = [
        "link1_origin",
        "link2_origin",
        "link3_origin",
        "link4_origin",
        "end_tip",
    ]

    x_coordinates = [
        points[name][0]
        for name in point_names
    ]

    y_coordinates = [
        points[name][1]
        for name in point_names
    ]

    return x_coordinates, y_coordinates


def select_evenly_spaced_states(
    sequence,
    number_of_states,
):
    if number_of_states < 2:
        raise ValueError(
            "number_of_states must be at least 2."
        )

    last_sequence_index = len(sequence) - 1

    selected_indices = [
        round(
            index
            * last_sequence_index
            / (number_of_states - 1)
        )
        for index in range(number_of_states)
    ]

    return [
        sequence[index]
        for index in selected_indices
    ]


def calculate_plot_limits(path_points, margin):
    path_x = [point[0] for point in path_points]
    path_y = [point[1] for point in path_points]

    min_x = min(path_x) - margin
    max_x = max(path_x) + margin
    min_y = min(path_y) - margin
    max_y = max(path_y) + margin

    return min_x, max_x, min_y, max_y


def main():
    path_points = generate_sine_path(
        path_length=4.0,
        sample_interval=0.01,
        amplitude=0.25,
        wavelength=3.0,
    )

    sequence = calculate_follow_the_leader_sequence(
        path_points
    )

    selected_states = select_evenly_spaced_states(
        sequence=sequence,
        number_of_states=5,
    )

    path_x = [
        point[0]
        for point in path_points
    ]

    path_y = [
        point[1]
        for point in path_points
    ]

    min_x, max_x, min_y, max_y = (
        calculate_plot_limits(
            path_points=path_points,
            margin=0.3,
        )
    )

    point_names = [
        "link1_origin",
        "link2_origin",
        "link3_origin",
        "link4_origin",
        "end_tip",
    ]

    for figure_index, state in enumerate(
        selected_states,
        start=1,
    ):
        plt.figure(figsize=(10, 5))

        plt.plot(
            path_x,
            path_y,
            linestyle="--",
            label="end-tip path",
        )

        robot_x, robot_y = extract_robot_coordinates(
            state["points"]
        )

        plt.plot(
            robot_x,
            robot_y,
            marker="o",
            label=(
                "robot shape "
                f"(index {state['end_tip_index']})"
            ),
        )

        for name in point_names:
            point = state["points"][name]

            plt.text(
                point[0],
                point[1],
                f" {name}",
                fontsize=9,
            )

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(
            "Planar follow-the-leader geometry "
            f"(state {figure_index}, "
            f"index {state['end_tip_index']})"
        )
        plt.axis("equal")
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)
        plt.grid()
        plt.legend()
        plt.tight_layout()

        output_path = (
            f"/tmp/follow_the_leader_state_"
            f"{figure_index}.png"
        )

        plt.savefig(
            output_path,
            dpi=150,
        )

        print(
            f"Saved visualization to {output_path}"
        )

    plt.show()


if __name__ == "__main__":
    main()