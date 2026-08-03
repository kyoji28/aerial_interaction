#!/usr/bin/env python3

import math


LINK_BODY_LENGTH_M = 0.474
LINK_ORIGIN_SPACING_M = 0.5255


def move_along_yaw(x, y, yaw, distance):
    next_x = x + distance * math.cos(yaw)
    next_y = y + distance * math.sin(yaw)

    return next_x, next_y

def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def calculate_yaw_between_points(start_point, end_point):
    delta_x = end_point[0] - start_point[0]
    delta_y = end_point[1] - start_point[1]

    return math.atan2(
        delta_y,
        delta_x,
    )


def calculate_joint_yaws_from_points(points):
    link1_yaw = calculate_yaw_between_points(
        points["link1_origin"],
        points["link2_origin"],
    )

    link2_yaw = calculate_yaw_between_points(
        points["link2_origin"],
        points["link3_origin"],
    )

    link3_yaw = calculate_yaw_between_points(
        points["link3_origin"],
        points["link4_origin"],
    )

    link4_yaw = calculate_yaw_between_points(
        points["link4_origin"],
        points["end_tip"],
    )

    joint1_yaw = normalize_angle(
        link2_yaw - link1_yaw
    )

    joint2_yaw = normalize_angle(
        link3_yaw - link2_yaw
    )

    joint3_yaw = normalize_angle(
        link4_yaw - link3_yaw
    )

    return {
        "link1_yaw": link1_yaw,
        "link2_yaw": link2_yaw,
        "link3_yaw": link3_yaw,
        "link4_yaw": link4_yaw,
        "joint1_yaw": joint1_yaw,
        "joint2_yaw": joint2_yaw,
        "joint3_yaw": joint3_yaw,
    }


def calculate_planar_forward_kinematics(
    root_x,
    root_y,
    root_yaw,
    joint1_yaw,
    joint2_yaw,
    joint3_yaw,
):
    link1_yaw = root_yaw
    link2_yaw = link1_yaw + joint1_yaw
    link3_yaw = link2_yaw + joint2_yaw
    link4_yaw = link3_yaw + joint3_yaw

    link1_origin = (root_x, root_y)

    link2_origin = move_along_yaw(
        link1_origin[0],
        link1_origin[1],
        link1_yaw,
        LINK_ORIGIN_SPACING_M,
    )

    link3_origin = move_along_yaw(
        link2_origin[0],
        link2_origin[1],
        link2_yaw,
        LINK_ORIGIN_SPACING_M,
    )

    link4_origin = move_along_yaw(
        link3_origin[0],
        link3_origin[1],
        link3_yaw,
        LINK_ORIGIN_SPACING_M,
    )

    end_tip = move_along_yaw(
        link4_origin[0],
        link4_origin[1],
        link4_yaw,
        LINK_BODY_LENGTH_M,
    )

    return {
        "link1_origin": link1_origin,
        "link2_origin": link2_origin,
        "link3_origin": link3_origin,
        "link4_origin": link4_origin,
        "end_tip": end_tip,
    }


def print_points(title, points):
    print(title)

    for name, point in points.items():
        print(
            f"{name:12s}: "
            f"({point[0]:.4f}, {point[1]:.4f})"
        )

    print()


def main():
    straight_points = calculate_planar_forward_kinematics(
        root_x=0.0,
        root_y=0.0,
        root_yaw=0.0,
        joint1_yaw=0.0,
        joint2_yaw=0.0,
        joint3_yaw=0.0,
    )

    square_points = calculate_planar_forward_kinematics(
        root_x=0.0,
        root_y=0.0,
        root_yaw=0.0,
        joint1_yaw=math.pi / 2.0,
        joint2_yaw=math.pi / 2.0,
        joint3_yaw=math.pi / 2.0,
    )

    curved_points = calculate_planar_forward_kinematics(
    root_x=0.0,
    root_y=0.0,
    root_yaw=0.0,
    joint1_yaw=math.radians(80.0),
    joint2_yaw=math.radians(80.0),
    joint3_yaw=math.radians(-35.0),
)


    print_points(
        "Straight posture",
        straight_points,
    )

    print_points(
        "Square posture",
        square_points,
    )

    print_points(
    "Curved posture",
    curved_points,
    )

    recovered_yaws = calculate_joint_yaws_from_points(
        curved_points
    )

    print("Recovered yaw angles")

    for name, yaw in recovered_yaws.items():
        print(
            f"{name:12s}: "
            f"{math.degrees(yaw):.3f} deg"
        )

if __name__ == "__main__":
    main()