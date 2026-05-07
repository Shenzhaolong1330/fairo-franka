#!/usr/bin/env python
"""Smoke test for polymetis.dual_franka_interface_client.

This script is intentionally read-only by default. It checks that the ZeroRPC
dual-arm server is reachable and that both robot state APIs return sane shapes.
Pass --with-grippers to also initialize gripper clients and read gripper state.
"""

import argparse
import sys
import threading
from typing import Any

import numpy as np
import zerorpc

from polymetis.dual_franka_interface_client import DualFrankaInterfaceClient


def _format_array(value: np.ndarray) -> str:
    return np.array2string(value, precision=4, suppress_small=False)


def _check_vector(name: str, value: Any, expected_size: int) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (expected_size,):
        raise RuntimeError(f"{name} shape is {array.shape}, expected ({expected_size},)")
    if not np.all(np.isfinite(array)):
        raise RuntimeError(f"{name} contains non-finite values: {array}")
    return array


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only smoke test for DualFrankaInterfaceClient."
    )
    parser.add_argument("--ip", default="localhost", help="Dual interface server IP.")
    parser.add_argument("--port", type=int, default=4243, help="Dual interface port.")
    parser.add_argument(
        "--with-grippers",
        action="store_true",
        help="Also initialize grippers and read their states.",
    )
    parser.add_argument(
        "--move-safe",
        action="store_true",
        help="Move both end effectors to symmetric safe midpoint-frame positions.",
    )
    parser.add_argument(
        "--safe-x",
        type=float,
        default=0.20,
        help="Absolute X offset in meters from midpoint for symmetric safe poses.",
    )
    parser.add_argument(
        "--safe-y",
        type=float,
        default=0.0,
        help="Y coordinate in meters for both safe poses in midpoint frame.",
    )
    parser.add_argument(
        "--safe-z",
        type=float,
        default=0.35,
        help="Z coordinate in meters for both safe poses in midpoint frame.",
    )
    parser.add_argument(
        "--time-to-go",
        type=float,
        default=4.0,
        help="Move duration in seconds for safe pose commands.",
    )
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=0.05,
        help="Allowed final position error in meters after --move-safe.",
    )
    parser.add_argument(
        "--parallel-move",
        action="store_true",
        help="Send left/right safe pose commands in parallel. Default is sequential.",
    )
    args = parser.parse_args()

    print(f"Connecting to dual Franka interface at {args.ip}:{args.port}...")
    client = DualFrankaInterfaceClient(ip=args.ip, port=args.port)

    try:
        left_q = _check_vector("left joint positions", client.left_robot_get_joint_positions(), 7)
        right_q = _check_vector(
            "right joint positions", client.right_robot_get_joint_positions(), 7
        )
        left_dq = _check_vector(
            "left joint velocities", client.left_robot_get_joint_velocities(), 7
        )
        right_dq = _check_vector(
            "right joint velocities", client.right_robot_get_joint_velocities(), 7
        )
        left_pose = _check_vector("left ee pose", client.left_robot_get_ee_pose(), 6)
        right_pose = _check_vector("right ee pose", client.right_robot_get_ee_pose(), 6)

        print("Robot state reads OK.")
        print(f"  left q:      {_format_array(left_q)}")
        print(f"  right q:     {_format_array(right_q)}")
        print(f"  left dq:     {_format_array(left_dq)}")
        print(f"  right dq:    {_format_array(right_dq)}")
        print(f"  left pose:   {_format_array(left_pose)}")
        print(f"  right pose:  {_format_array(right_pose)}")

        if args.move_safe:
            left_target = left_pose.copy()
            right_target = right_pose.copy()
            left_target[:3] = np.array([-args.safe_x, args.safe_y, args.safe_z])
            right_target[:3] = np.array([args.safe_x, args.safe_y, args.safe_z])

            print("Moving end effectors to symmetric midpoint-frame safe poses.")
            print(f"  left target:  {_format_array(left_target)}")
            print(f"  right target: {_format_array(right_target)}")

            errors = []

            def move_arm(arm: str, target: np.ndarray):
                move_client = DualFrankaInterfaceClient(ip=args.ip, port=args.port)
                try:
                    if arm == "left":
                        move_client.left_robot_move_to_ee_pose(
                            target, time_to_go=args.time_to_go, delta=False
                        )
                    else:
                        move_client.right_robot_move_to_ee_pose(
                            target, time_to_go=args.time_to_go, delta=False
                        )
                except Exception as exc:
                    errors.append((arm, exc))
                finally:
                    move_client.close()

            def move_left():
                move_arm("left", left_target)

            def move_right():
                move_arm("right", right_target)

            if args.parallel_move:
                left_thread = threading.Thread(target=move_left, name="move-left-safe")
                right_thread = threading.Thread(target=move_right, name="move-right-safe")
                left_thread.start()
                right_thread.start()
                left_thread.join()
                right_thread.join()
            else:
                move_left()
                move_right()

            if errors:
                details = ", ".join(f"{arm}: {exc}" for arm, exc in errors)
                raise RuntimeError(f"safe pose move failed: {details}")

            left_after = _check_vector(
                "left ee pose after move", client.left_robot_get_ee_pose(), 6
            )
            right_after = _check_vector(
                "right ee pose after move", client.right_robot_get_ee_pose(), 6
            )
            print("Safe pose move finished.")
            print(f"  left after:  {_format_array(left_after)}")
            print(f"  right after: {_format_array(right_after)}")

            left_error = float(np.linalg.norm(left_after[:3] - left_target[:3]))
            right_error = float(np.linalg.norm(right_after[:3] - right_target[:3]))
            print(f"  left position error:  {left_error:.4f} m")
            print(f"  right position error: {right_error:.4f} m")
            if left_error > args.position_tolerance or right_error > args.position_tolerance:
                raise RuntimeError(
                    "safe pose move did not reach the target; "
                    f"errors: left={left_error:.4f} m, right={right_error:.4f} m. "
                    "Check the Franka arm logs for motion aborts/reflex errors."
                )

        if args.with_grippers:
            print("Initializing gripper clients...")
            client.gripper_initialize()
            left_gripper = client.left_gripper_get_state()
            right_gripper = client.right_gripper_get_state()
            print("Gripper state reads OK.")
            print(f"  left gripper:  {left_gripper}")
            print(f"  right gripper: {right_gripper}")

    except zerorpc.exceptions.RemoteError as exc:
        print(f"Remote server error: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Smoke test failed: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()

    print("Dual Franka client smoke test passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
