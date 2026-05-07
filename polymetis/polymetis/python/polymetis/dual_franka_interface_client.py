"""
ZeroRPC client for DualFrankaInterfaceServer.

Run this on the user machine to control the left/right Franka arms and
left/right grippers exposed by dual_franka_interface_server.py.
"""

import logging

import numpy as np
import zerorpc


log = logging.getLogger(__name__)


class DualFrankaInterfaceClient:
    def __init__(self, ip: str = "192.168.100.63", port: int = 4243):
        try:
            self.server = zerorpc.Client(heartbeat=20)
            self.server.connect(f"tcp://{ip}:{port}")
            log.info("Connected to dual Franka server")
        except Exception:
            log.exception("Failed to connect to dual Franka server")

    @staticmethod
    def _to_list(value):
        if value is None:
            return None
        return value.tolist() if hasattr(value, "tolist") else value

    def gripper_initialize(self, arm: str = "both"):
        self.server.gripper_initialize(arm)

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
        self.server.gripper_goto(
            arm,
            width,
            speed,
            force,
            epsilon_inner,
            epsilon_outer,
            blocking,
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
        self.server.gripper_grasp(
            arm,
            speed,
            force,
            grasp_width,
            epsilon_inner,
            epsilon_outer,
            blocking,
        )

    def gripper_get_state(self, arm: str) -> dict:
        return self.server.gripper_get_state(arm)

    def robot_get_joint_positions(self, arm: str) -> np.ndarray:
        return np.array(self.server.robot_get_joint_positions(arm))

    def robot_get_joint_velocities(self, arm: str) -> np.ndarray:
        return np.array(self.server.robot_get_joint_velocities(arm))

    def robot_get_ee_pose(self, arm: str) -> np.ndarray:
        return np.array(self.server.robot_get_ee_pose(arm))

    def robot_move_to_joint_positions(
        self,
        arm: str,
        positions: np.ndarray,
        time_to_go: float = None,
        delta: bool = False,
        Kq: np.ndarray = None,
        Kqd: np.ndarray = None,
    ):
        self.server.robot_move_to_joint_positions(
            arm,
            self._to_list(positions),
            time_to_go,
            delta,
            self._to_list(Kq),
            self._to_list(Kqd),
        )

    def robot_go_home(self, arm: str):
        self.server.robot_go_home(arm)

    def robot_move_to_ee_pose(
        self,
        arm: str,
        pose: np.ndarray,
        time_to_go: float = None,
        delta: bool = False,
        Kx: np.ndarray = None,
        Kxd: np.ndarray = None,
        op_space_interp: bool = True,
    ):
        self.server.robot_move_to_ee_pose(
            arm,
            self._to_list(pose),
            time_to_go,
            delta,
            self._to_list(Kx),
            self._to_list(Kxd),
            op_space_interp,
        )

    def robot_start_joint_impedance_control(
        self,
        arm: str,
        Kq: np.ndarray = None,
        Kqd: np.ndarray = None,
        adaptive: bool = True,
    ):
        self.server.robot_start_joint_impedance_control(
            arm,
            self._to_list(Kq),
            self._to_list(Kqd),
            adaptive,
        )

    def robot_start_cartesian_impedance_control(
        self,
        arm: str,
        Kx: np.ndarray = None,
        Kxd: np.ndarray = None,
    ):
        self.server.robot_start_cartesian_impedance_control(
            arm,
            self._to_list(Kx),
            self._to_list(Kxd),
        )

    def robot_update_desired_joint_positions(self, arm: str, positions: np.ndarray):
        self.server.robot_update_desired_joint_positions(arm, self._to_list(positions))

    def robot_update_desired_ee_pose(self, arm: str, pose: np.ndarray):
        self.server.robot_update_desired_ee_pose(arm, self._to_list(pose))

    def robot_terminate_current_policy(self, arm: str):
        self.server.robot_terminate_current_policy(arm)

    def left_gripper_goto(self, *args, **kwargs):
        return self.gripper_goto("left", *args, **kwargs)

    def right_gripper_goto(self, *args, **kwargs):
        return self.gripper_goto("right", *args, **kwargs)

    def left_gripper_grasp(self, *args, **kwargs):
        return self.gripper_grasp("left", *args, **kwargs)

    def right_gripper_grasp(self, *args, **kwargs):
        return self.gripper_grasp("right", *args, **kwargs)

    def left_gripper_get_state(self) -> dict:
        return self.gripper_get_state("left")

    def right_gripper_get_state(self) -> dict:
        return self.gripper_get_state("right")

    def left_robot_get_joint_positions(self) -> np.ndarray:
        return self.robot_get_joint_positions("left")

    def right_robot_get_joint_positions(self) -> np.ndarray:
        return self.robot_get_joint_positions("right")

    def left_robot_get_joint_velocities(self) -> np.ndarray:
        return self.robot_get_joint_velocities("left")

    def right_robot_get_joint_velocities(self) -> np.ndarray:
        return self.robot_get_joint_velocities("right")

    def left_robot_get_ee_pose(self) -> np.ndarray:
        return self.robot_get_ee_pose("left")

    def right_robot_get_ee_pose(self) -> np.ndarray:
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

    def left_robot_update_desired_joint_positions(self, positions: np.ndarray):
        return self.robot_update_desired_joint_positions("left", positions)

    def right_robot_update_desired_joint_positions(self, positions: np.ndarray):
        return self.robot_update_desired_joint_positions("right", positions)

    def left_robot_update_desired_ee_pose(self, pose: np.ndarray):
        return self.robot_update_desired_ee_pose("left", pose)

    def right_robot_update_desired_ee_pose(self, pose: np.ndarray):
        return self.robot_update_desired_ee_pose("right", pose)

    def left_robot_terminate_current_policy(self):
        return self.robot_terminate_current_policy("left")

    def right_robot_terminate_current_policy(self):
        return self.robot_terminate_current_policy("right")

    def close(self):
        self.server.close()


if __name__ == "__main__":
    franka = DualFrankaInterfaceClient(ip="localhost", port=4243)
    franka.gripper_initialize()

    left_q = franka.left_robot_get_joint_positions()
    right_q = franka.right_robot_get_joint_positions()
    print(f"Left joint positions: {left_q}")
    print(f"Right joint positions: {right_q}")
    left_ee_pose = franka.left_robot_get_ee_pose()
    right_ee_pose = franka.right_robot_get_ee_pose()
    print(f"Left end-effector pose: {left_ee_pose}")
    print(f"Right end-effector pose: {right_ee_pose}")

    franka.left_gripper_goto(width=0.085, speed=0.1, force=10.0)
    franka.right_gripper_goto(width=0.085, speed=0.1, force=10.0)
