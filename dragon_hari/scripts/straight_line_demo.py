#!/usr/bin/env python3

import subprocess
import sys


PACKAGE_NAME = "dragon_hari"

DEMO_OPTIONS = {
    "1": (
        "fixed_square",
        "straight_line_fixed_square.py",
    ),
    "2": (
        "vertex_forward",
        "straight_line_vertex_forward.py",
    ),
    "3": (
        "link4_forward",
        "straight_line_link4_forward.py",
    ),
    "4": (
        "body_wave",
        "straight_line_body_wave.py",
    ),
}

MODE_ALIASES = {
    mode_name: option_number
    for option_number, (mode_name, _) in DEMO_OPTIONS.items()
}

QUIT_COMMANDS = {
    "q",
    "quit",
    "exit",
}


def print_menu():
    print()
    print("Select mode:")
    print("1: fixed_square")
    print("2: vertex_forward")
    print("3: link4_forward")
    print("4: body_wave")
    print("q: quit")


def read_mode():
    while True:
        print_menu()

        try:
            user_input = input("Mode: ").strip().lower()
        except EOFError:
            return None

        if user_input in QUIT_COMMANDS:
            return None

        option_number = MODE_ALIASES.get(
            user_input,
            user_input,
        )

        selected = DEMO_OPTIONS.get(option_number)

        if selected is not None:
            return selected

        print(
            "Enter 1, 2, 3, 4, "
            "a mode name, or q."
        )


def read_goal():
    while True:
        try:
            user_input = input(
                "Enter target x y [m]: "
            ).strip()
        except EOFError:
            return None

        if user_input.lower() in QUIT_COMMANDS:
            return None

        values = user_input.split()

        if len(values) != 2:
            print("Enter exactly 2 numbers: x y.")
            continue

        try:
            goal_x, goal_y = map(float, values)
        except ValueError:
            print("Enter exactly 2 numbers: x y.")
            continue

        return goal_x, goal_y


def run_demo(
    mode_name,
    script_name,
    goal_x,
    goal_y,
):
    command = [
        "rosrun",
        PACKAGE_NAME,
        script_name,
    ]

    print()
    print(
        f"Starting {mode_name} demo: "
        f"goal=({goal_x:.3f}, {goal_y:.3f})"
    )

    try:
        result = subprocess.run(
            command,
            input=f"{goal_x} {goal_y}\n",
            text=True,
            check=False,
        )
    except FileNotFoundError:
        print(
            "rosrun command was not found.",
            file=sys.stderr,
        )
        return
    except KeyboardInterrupt:
        print()
        print(f"{mode_name} demo interrupted.")
        return

    if result.returncode == 0:
        print()
        print(f"{mode_name} demo finished.")
    else:
        print()
        print(
            f"{mode_name} demo exited with "
            f"code {result.returncode}.",
            file=sys.stderr,
        )


def main():
    print("Straight-line demo launcher started.")

    try:
        while True:
            selected = read_mode()

            if selected is None:
                break

            mode_name, script_name = selected

            goal = read_goal()

            if goal is None:
                break

            goal_x, goal_y = goal

            run_demo(
                mode_name,
                script_name,
                goal_x,
                goal_y,
            )

    except KeyboardInterrupt:
        print()
        print("Launcher interrupted.")

    print("Straight-line demo launcher terminated.")


if __name__ == "__main__":
    main()
