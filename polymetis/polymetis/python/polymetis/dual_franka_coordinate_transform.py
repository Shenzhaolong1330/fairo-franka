"""Coordinate transforms for the dual Franka installation.

Frame convention:
  midpoint/world frame:
    origin: midpoint between the two robot bases
    +X: from the left base toward the right base
    +Y/+Z: right-handed with +X

  left base frame:
    origin: left Franka base, 0.25 m from the midpoint along -X
    mounting rotation: +45 deg about +Y in right-hand-rule convention

  right base frame:
    origin: right Franka base, 0.25 m from the midpoint along +X
    mounting rotation: -45 deg about +X in right-hand-rule convention

The pose format used here matches DualFrankaInterfaceClient:
  [x, y, z, rx, ry, rz]
where [rx, ry, rz] is a rotation vector.
"""

from dataclasses import dataclass
from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation


LEFT_ARM = "left"
RIGHT_ARM = "right"

LEFT_ROBOT_IP = "172.16.0.2"
RIGHT_ROBOT_IP = "172.16.0.3"

BASE_DISTANCE_M = 0.50
LEFT_BASE_TRANSLATION_M = np.array([-BASE_DISTANCE_M / 2.0, 0.0, 0.0])
RIGHT_BASE_TRANSLATION_M = np.array([BASE_DISTANCE_M / 2.0, 0.0, 0.0])

LEFT_BASE_ROTATION = Rotation.from_euler("y", 45.0, degrees=True)
RIGHT_BASE_ROTATION = Rotation.from_euler("x", -45.0, degrees=True)


@dataclass(frozen=True)
class ArmMount:
    """Rigid transform from an arm base frame to the midpoint/world frame."""

    name: str
    robot_ip: str
    translation_midpoint_from_base: np.ndarray
    rotation_midpoint_from_base: Rotation

    @property
    def T_midpoint_base(self) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = self.rotation_midpoint_from_base.as_matrix()
        transform[:3, 3] = self.translation_midpoint_from_base
        return transform

    @property
    def T_base_midpoint(self) -> np.ndarray:
        return np.linalg.inv(self.T_midpoint_base)


ARM_MOUNTS: Dict[str, ArmMount] = {
    LEFT_ARM: ArmMount(
        name=LEFT_ARM,
        robot_ip=LEFT_ROBOT_IP,
        translation_midpoint_from_base=LEFT_BASE_TRANSLATION_M,
        rotation_midpoint_from_base=LEFT_BASE_ROTATION,
    ),
    RIGHT_ARM: ArmMount(
        name=RIGHT_ARM,
        robot_ip=RIGHT_ROBOT_IP,
        translation_midpoint_from_base=RIGHT_BASE_TRANSLATION_M,
        rotation_midpoint_from_base=RIGHT_BASE_ROTATION,
    ),
}

ARM_BY_IP = {mount.robot_ip: mount.name for mount in ARM_MOUNTS.values()}


def get_mount(arm: str) -> ArmMount:
    try:
        return ARM_MOUNTS[arm]
    except KeyError as exc:
        raise ValueError(f"arm must be '{LEFT_ARM}' or '{RIGHT_ARM}', got {arm!r}") from exc


def arm_from_ip(robot_ip: str) -> str:
    try:
        return ARM_BY_IP[robot_ip]
    except KeyError as exc:
        raise ValueError(f"unknown robot IP {robot_ip!r}") from exc


def transform_matrix_base_to_midpoint(arm: str) -> np.ndarray:
    return get_mount(arm).T_midpoint_base.copy()


def transform_matrix_midpoint_to_base(arm: str) -> np.ndarray:
    return get_mount(arm).T_base_midpoint.copy()


def point_base_to_midpoint(arm: str, point_base) -> np.ndarray:
    mount = get_mount(arm)
    point_base = np.asarray(point_base, dtype=float)
    return mount.rotation_midpoint_from_base.apply(point_base) + mount.translation_midpoint_from_base


def point_midpoint_to_base(arm: str, point_midpoint) -> np.ndarray:
    mount = get_mount(arm)
    point_midpoint = np.asarray(point_midpoint, dtype=float)
    return mount.rotation_midpoint_from_base.inv().apply(
        point_midpoint - mount.translation_midpoint_from_base
    )


def pose_base_to_midpoint(arm: str, pose_base) -> np.ndarray:
    """Convert [xyz, rotvec] from an arm base frame into midpoint/world frame."""
    mount = get_mount(arm)
    pose_base = np.asarray(pose_base, dtype=float)
    if pose_base.shape != (6,):
        raise ValueError(f"pose_base must have shape (6,), got {pose_base.shape}")

    position_midpoint = point_base_to_midpoint(arm, pose_base[:3])
    rotation_base_ee = Rotation.from_rotvec(pose_base[3:])
    rotation_midpoint_ee = mount.rotation_midpoint_from_base * rotation_base_ee
    return np.concatenate([position_midpoint, rotation_midpoint_ee.as_rotvec()])


def pose_midpoint_to_base(arm: str, pose_midpoint) -> np.ndarray:
    """Convert [xyz, rotvec] from midpoint/world frame into an arm base frame."""
    mount = get_mount(arm)
    pose_midpoint = np.asarray(pose_midpoint, dtype=float)
    if pose_midpoint.shape != (6,):
        raise ValueError(f"pose_midpoint must have shape (6,), got {pose_midpoint.shape}")

    position_base = point_midpoint_to_base(arm, pose_midpoint[:3])
    rotation_midpoint_ee = Rotation.from_rotvec(pose_midpoint[3:])
    rotation_base_ee = mount.rotation_midpoint_from_base.inv() * rotation_midpoint_ee
    return np.concatenate([position_base, rotation_base_ee.as_rotvec()])


def poses_base_to_midpoint(left_pose_base, right_pose_base) -> Dict[str, np.ndarray]:
    return {
        LEFT_ARM: pose_base_to_midpoint(LEFT_ARM, left_pose_base),
        RIGHT_ARM: pose_base_to_midpoint(RIGHT_ARM, right_pose_base),
    }


def poses_midpoint_to_base(left_pose_midpoint, right_pose_midpoint) -> Dict[str, np.ndarray]:
    return {
        LEFT_ARM: pose_midpoint_to_base(LEFT_ARM, left_pose_midpoint),
        RIGHT_ARM: pose_midpoint_to_base(RIGHT_ARM, right_pose_midpoint),
    }
