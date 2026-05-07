"""
ZeroRPC server for controlling two Franka arms and two grippers through Polymetis.

Run this on the machine that can reach the Polymetis robot/gripper servers.
By default it expects:
  left arm      -> localhost:50051
  left gripper  -> localhost:50052
  right arm     -> localhost:50053
  right gripper -> localhost:50054
"""

import logging

import numpy as np
import scipy.spatial.transform as st
import torch
import zerorpc

from polymetis import GripperInterface, RobotInterface
from polymetis.dual_franka_coordinate_transform import (
    pose_base_to_midpoint,
    pose_midpoint_to_base,
)


log = logging.getLogger(__name__)


class DualFrankaInterfaceServer:
    def __init__(
        self,
        left_robot_ip: str = "localhost",
        left_robot_port: int = 50051,
        right_robot_ip: str = "localhost",
        right_robot_port: int = 50053,
        left_gripper_ip: str = "localhost",
        left_gripper_port: int = 50052,
        right_gripper_ip: str = "localhost",
        right_gripper_port: int = 50054,
        enforce_version: bool = False,
        use_midpoint_frame: bool = True,
    ):
        self.use_midpoint_frame = use_midpoint_frame
        self.robot_cfgs = {
            "left": (left_robot_ip, left_robot_port),
            "right": (right_robot_ip, right_robot_port),
        }
        self.gripper_cfgs = {
            "left": (left_gripper_ip, left_gripper_port),
            "right": (right_gripper_ip, right_gripper_port),
        }

        self.robots = {}
        self.grippers = {}

        for arm, (ip, port) in self.robot_cfgs.items():
            try:
                self.robots[arm] = RobotInterface(
                    ip_address=ip,
                    port=port,
                    enforce_version=enforce_version,
                )
                log.info("Connected to %s robot at %s:%s", arm, ip, port)
            except Exception:
                log.exception("Failed to connect to %s robot at %s:%s", arm, ip, port)

    def _check_arm(self, arm: str) -> str:
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")
        return arm

    def _robot(self, arm: str):
        arm = self._check_arm(arm)
        if arm not in self.robots:
            raise RuntimeError(f"{arm} robot is not connected")
        return self.robots[arm]

    def _gripper(self, arm: str):
        arm = self._check_arm(arm)
        if arm not in self.grippers:
            raise RuntimeError(f"{arm} gripper is not connected; call gripper_initialize first")
        return self.grippers[arm]

    @staticmethod
    def _state_to_dict(state) -> dict:
        return {
            "width": state.width,
            "is_moving": state.is_moving,
            "is_grasped": state.is_grasped,
            "prev_command_successful": state.prev_command_successful,
            "error_code": state.error_code,
        }

    def gripper_initialize(self, arm: str = "both"):
        arms = ("left", "right") if arm == "both" else (self._check_arm(arm),)
        for name in arms:
            ip, port = self.gripper_cfgs[name]
            try:
                self.grippers[name] = GripperInterface(ip_address=ip, port=port)
                log.info("Connected to %s gripper at %s:%s", name, ip, port)
            except Exception:
                log.exception("Failed to connect to %s gripper at %s:%s", name, ip, port)

    def gripper_goto(
        self,
        arm: str,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ):
        self._gripper(arm).goto(
            width=width,
            speed=speed,
            force=force,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking,
        )

    def gripper_grasp(
        self,
        arm: str,
        speed: float,
        force: float,
        grasp_width: float = 0.0,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ):
        self._gripper(arm).grasp(
            speed=speed,
            force=force,
            grasp_width=grasp_width,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking,
        )

    def gripper_get_state(self, arm: str) -> dict:
        return self._state_to_dict(self._gripper(arm).get_state())

    def robot_get_joint_positions(self, arm: str) -> list:
        return self._robot(arm).get_joint_positions().numpy().tolist()

    def robot_get_joint_velocities(self, arm: str) -> list:
        return self._robot(arm).get_joint_velocities().numpy().tolist()

    def robot_get_ee_pose(self, arm: str) -> list:
        position, quat_xyzw = self._robot(arm).get_ee_pose()
        rot_vec = st.Rotation.from_quat(quat_xyzw.numpy()).as_rotvec()
        pose_base = np.concatenate([position.numpy(), rot_vec])
        if self.use_midpoint_frame:
            return pose_base_to_midpoint(arm, pose_base).tolist()
        return pose_base.tolist()

    def robot_move_to_joint_positions(
        self,
        arm: str,
        positions: list,
        time_to_go: float = None,
        delta: bool = False,
        Kq: list = None,
        Kqd: list = None,
    ):
        self._robot(arm).move_to_joint_positions(
            positions=torch.Tensor(positions),
            time_to_go=time_to_go,
            delta=delta,
            Kq=torch.Tensor(Kq) if Kq is not None else None,
            Kqd=torch.Tensor(Kqd) if Kqd is not None else None,
        )

    def robot_go_home(self, arm: str):
        self._robot(arm).go_home()

    def robot_move_to_ee_pose(
        self,
        arm: str,
        pose: list,
        time_to_go: float = None,
        delta: bool = False,
        Kx: list = None,
        Kxd: list = None,
        op_space_interp: bool = True,
    ):
        if delta and self.use_midpoint_frame:
            raise ValueError(
                "delta=True is not supported when use_midpoint_frame=True; "
                "send an absolute midpoint-frame pose instead"
            )
        if self.use_midpoint_frame:
            pose = pose_midpoint_to_base(arm, pose).tolist()
        pose_tensor = torch.Tensor(pose)
        self._robot(arm).move_to_ee_pose(
            position=pose_tensor[:3],
            orientation=torch.Tensor(st.Rotation.from_rotvec(pose_tensor[3:]).as_quat()),
            time_to_go=time_to_go,
            delta=delta,
            Kx=torch.Tensor(Kx) if Kx is not None else None,
            Kxd=torch.Tensor(Kxd) if Kxd is not None else None,
            op_space_interp=op_space_interp,
        )

    def robot_start_joint_impedance_control(
        self,
        arm: str,
        Kq: list = None,
        Kqd: list = None,
        adaptive: bool = True,
    ):
        self._robot(arm).start_joint_impedance(
            Kq=torch.Tensor(Kq) if Kq is not None else None,
            Kqd=torch.Tensor(Kqd) if Kqd is not None else None,
            adaptive=adaptive,
        )

    def robot_start_cartesian_impedance_control(
        self,
        arm: str,
        Kx: list = None,
        Kxd: list = None,
    ):
        self._robot(arm).start_cartesian_impedance(
            Kx=torch.Tensor(Kx) if Kx is not None else None,
            Kxd=torch.Tensor(Kxd) if Kxd is not None else None,
        )

    def robot_update_desired_joint_positions(self, arm: str, positions: list):
        self._robot(arm).update_desired_joint_positions(positions=torch.Tensor(positions))

    def robot_update_desired_ee_pose(self, arm: str, pose: list):
        if self.use_midpoint_frame:
            pose = pose_midpoint_to_base(arm, pose).tolist()
        pose_tensor = torch.Tensor(pose)
        self._robot(arm).update_desired_ee_pose(
            position=pose_tensor[:3],
            orientation=torch.Tensor(st.Rotation.from_rotvec(pose_tensor[3:]).as_quat()),
        )

    def robot_terminate_current_policy(self, arm: str):
        self._robot(arm).terminate_current_policy()

    def left_gripper_goto(self, *args, **kwargs):
        return self.gripper_goto("left", *args, **kwargs)

    def right_gripper_goto(self, *args, **kwargs):
        return self.gripper_goto("right", *args, **kwargs)

    def left_gripper_grasp(self, *args, **kwargs):
        return self.gripper_grasp("left", *args, **kwargs)

    def right_gripper_grasp(self, *args, **kwargs):
        return self.gripper_grasp("right", *args, **kwargs)

    def left_gripper_get_state(self):
        return self.gripper_get_state("left")

    def right_gripper_get_state(self):
        return self.gripper_get_state("right")

    def left_robot_get_joint_positions(self):
        return self.robot_get_joint_positions("left")

    def right_robot_get_joint_positions(self):
        return self.robot_get_joint_positions("right")

    def left_robot_get_joint_velocities(self):
        return self.robot_get_joint_velocities("left")

    def right_robot_get_joint_velocities(self):
        return self.robot_get_joint_velocities("right")

    def left_robot_get_ee_pose(self):
        return self.robot_get_ee_pose("left")

    def right_robot_get_ee_pose(self):
        return self.robot_get_ee_pose("right")

    def left_robot_move_to_joint_positions(self, *args, **kwargs):
        return self.robot_move_to_joint_positions("left", *args, **kwargs)

    def right_robot_move_to_joint_positions(self, *args, **kwargs):
        return self.robot_move_to_joint_positions("right", *args, **kwargs)

    def left_robot_go_home(self):
        return self.robot_go_home("left")

    def right_robot_go_home(self):
        return self.robot_go_home("right")

    def left_robot_move_to_ee_pose(self, *args, **kwargs):
        return self.robot_move_to_ee_pose("left", *args, **kwargs)

    def right_robot_move_to_ee_pose(self, *args, **kwargs):
        return self.robot_move_to_ee_pose("right", *args, **kwargs)

    def left_robot_start_joint_impedance_control(self, *args, **kwargs):
        return self.robot_start_joint_impedance_control("left", *args, **kwargs)

    def right_robot_start_joint_impedance_control(self, *args, **kwargs):
        return self.robot_start_joint_impedance_control("right", *args, **kwargs)

    def left_robot_start_cartesian_impedance_control(self, *args, **kwargs):
        return self.robot_start_cartesian_impedance_control("left", *args, **kwargs)

    def right_robot_start_cartesian_impedance_control(self, *args, **kwargs):
        return self.robot_start_cartesian_impedance_control("right", *args, **kwargs)

    def left_robot_update_desired_joint_positions(self, positions):
        return self.robot_update_desired_joint_positions("left", positions)

    def right_robot_update_desired_joint_positions(self, positions):
        return self.robot_update_desired_joint_positions("right", positions)

    def left_robot_update_desired_ee_pose(self, pose):
        return self.robot_update_desired_ee_pose("left", pose)

    def right_robot_update_desired_ee_pose(self, pose):
        return self.robot_update_desired_ee_pose("right", pose)

    def left_robot_terminate_current_policy(self):
        return self.robot_terminate_current_policy("left")

    def right_robot_terminate_current_policy(self):
        return self.robot_terminate_current_policy("right")


if __name__ == "__main__":
    server = DualFrankaInterfaceServer()
    s = zerorpc.Server(server)
    s.bind("tcp://0.0.0.0:4243")
    s.run()
