#!/usr/bin/env python3

import math

from tf.transformations import euler_from_quaternion

from .geometry import normalize_angle


def quaternion_to_yaw(quaternion):
    _, _, yaw = euler_from_quaternion(
        [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]
    )

    return yaw


def calculate_cog_pose(
    link1_x,
    link1_y,
    link1_yaw,
    cog_offset_x,
    cog_offset_y,
    cog_offset_yaw,
):
    cosine_yaw = math.cos(link1_yaw)
    sine_yaw = math.sin(link1_yaw)

    cog_x = (
        link1_x
        + cosine_yaw * cog_offset_x
        - sine_yaw * cog_offset_y
    )

    cog_y = (
        link1_y
        + sine_yaw * cog_offset_x
        + cosine_yaw * cog_offset_y
    )

    cog_yaw = normalize_angle(
        link1_yaw + cog_offset_yaw
    )

    return cog_x, cog_y, cog_yaw
