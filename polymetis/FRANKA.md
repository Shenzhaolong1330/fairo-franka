# Adaptions for Franka Research 3

## Work Space 
In our teleoperation tests, we found that the work space of the Franka Research 3 robot is limited and once the robot reaches the work space boundary, it will twitch like shit.

You can customize the work space by changing the parameters `limits` in the [hardware.yaml](./polymetis/conf/robot_client/franka_hardware.yaml).

## Gripper Command
We made some adaptations in the gripper interface function `goto()` to enable gripper grasp things,
by setting `epsilon_inner` to a proper value.
Details can be found in the [gripper_interface.py](./polymetis/python/polymetis/gripper_interface.py).

## Installation from Source
I recommend you to install polymetis from source.
As we have installed libfranka beforehand in our computer,
I will link the libfranka in the installation process.

1. Clone repo:

```
git clonehttps://github.com/Shenzhaolong1330/fairo-franka.git
cd fairo-franka/polymetis
```
2. Create environment:
```
conda env create -f ./polymetis/environment.yml
conda activate polymetis-local
```
3. Install Python package in editable mode:
```
pip install -e ./polymetis
```
4. Build Polymetis from source:
```
mkdir -p ./polymetis/build
cd ./polymetis/build

cmake -DBUILD_FRANKA=ON -DFranka_DIR=/home/deepcybo/libfranka/build -DCMAKE_BUILD_TYPE=Release -DBUILD_DOCS=ON ..
make -j
```

## Usage
If you want to use the interface to control the Franka robot, follow the steps below:
```
cd polymetis/polymetis/python/scripts
python launch_robot.py robot_client=franka_hardware
python launch_gripper.py gripper=franka_hand
# optional: only if you want to control it from a remote machine
python launch_server.py
```
remember to use this command after launch robot
```
sudo pkill -9 run_server
```

If you want to use robotiq gripper, uncomment the content in `goto` in [gripper_interface.py](polymetis/polymetis/python/polymetis/gripper_interface.py)
```python
# CHOOSE to use robotiq gripper
cmd = polymetis_pb2.GripperCommand(
    width=width, speed=speed, force=force, grasp=False
)
```
and launch the gripper directly.
```bash
python launch_gripper.py
```

if you encounter the error `Permission denied`, try to run the command below:
```bash
sudo chmod 666 /dev/ttyUSB0
```

## Control Two Franka Arms with Two Robotiq Grippers

Polymetis controls one robot arm through one gRPC server and one hardware client.
To control two Franka arms, launch two independent Polymetis robot servers with different ports and different robot IPs.
The two Robotiq grippers should also use two independent gripper servers, two different ports, and two different serial devices.

Recommended port layout:

| Device | Robot/serial address | Polymetis port |
|--------|----------------------|----------------|
| Franka left arm | `172.16.0.2` | `50051` |
| Robotiq gripper 1 | `/dev/ttyUSB0` | `50052` |
| Franka right arm | `172.16.0.3` | `50053` |
| Robotiq gripper 2 | `/dev/ttyUSB1` | `50054` |

Make sure the two Franka control boxes do not have the same IP address on the same network.
If both robots still use the default `172.16.0.2`, change one robot's IP or use separate network interfaces/routes so each `franka_panda_client` reaches the correct robot.

### 1. Check gripper serial ports

Connect both Robotiq grippers and check their USB serial devices:

```bash
ls /dev/ttyUSB*
```

Grant access if needed:

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

USB device numbers may swap after reconnecting cables, so confirm them before launching.

### 2. Launch two Franka arms

Open two terminals and launch one arm per terminal:

```bash
cd polymetis/polymetis/python/scripts
export PYTHONNOUSERSITE=1
```

Terminal 1, Franka left arm:

```bash
python launch_robot.py \
  robot_client=franka_hardware \
  port=50051 \
  robot_client.executable_cfg.robot_ip=172.16.0.2
```

Terminal 2, Franka right arm:

```bash
python launch_robot.py \
  robot_client=franka_hardware \
  port=50053 \
  robot_client.executable_cfg.robot_ip=172.16.0.3
```

The `port` is the Polymetis gRPC server port used by your Python program.
The `robot_client.executable_cfg.robot_ip` is the real Franka control box IP.

### 3. Launch two Robotiq grippers

Open two more terminals:

```bash
cd polymetis/polymetis/python/scripts
export PYTHONNOUSERSITE=1
```

Terminal 3, Robotiq gripper 1:

```bash
python launch_gripper.py \
  gripper=robotiq_2f \
  port=50052 \
  gripper.comport=/dev/ttyUSB0
```

Terminal 4, Robotiq gripper 2:

```bash
python launch_gripper.py \
  gripper=robotiq_2f \
  port=50054 \
  gripper.comport=/dev/ttyUSB1
```

This repository also contains `launch_gripper2.yaml` and `gripper/robotiq_2f_2.yaml` for dual Robotiq usage, but the default second gripper port there is `50053`.
When running two arms and two grippers together, use `50054` for the second gripper to avoid colliding with the Franka right arm.

### 4. Control both arms and grippers in Python

```python
from polymetis import RobotInterface, GripperInterface

left_arm = RobotInterface(ip_address="localhost", port=50051)
left_gripper = GripperInterface(ip_address="localhost", port=50052)

right_arm = RobotInterface(ip_address="localhost", port=50053)
right_gripper = GripperInterface(ip_address="localhost", port=50054)

left_q = left_arm.get_joint_positions()
right_q = right_arm.get_joint_positions()

left_gripper.goto(width=0.05, speed=0.1, force=10)
right_gripper.goto(width=0.05, speed=0.1, force=10)
```

You can command both robots from one Python process, or split them into separate processes.
For synchronized motion, read both states first, compute both target commands, and then send commands to `left_arm` and `right_arm` in the same control loop.

### 5. Optional ZeroRPC dual-arm interface

This repository also provides a ZeroRPC wrapper similar to `franka_interface_server.py` and `franka_interface_client.py`:

- server: `polymetis/polymetis/python/polymetis/dual_franka_interface_server.py`
- client: `polymetis/polymetis/python/polymetis/dual_franka_interface_client.py`

After launching both Polymetis robot servers and both gripper servers, start the dual-arm ZeroRPC server on the robot machine:

```bash
cd polymetis/polymetis/python/scripts
python launch_dual_franka_interface_server.py
```

Then connect from the user machine:

```python
import numpy as np
from polymetis.dual_franka_interface_client import DualFrankaInterfaceClient

franka = DualFrankaInterfaceClient(ip="<ROBOT_MACHINE_IP>", port=4243)
franka.gripper_initialize()

left_q = franka.left_robot_get_joint_positions()
right_q = franka.right_robot_get_joint_positions()

franka.left_gripper_goto(width=0.05, speed=0.1, force=10.0)
franka.right_gripper_goto(width=0.05, speed=0.1, force=10.0)

left_target = np.array([-0.14, -0.02, -0.05, -1.57, 0.05, 1.50, -0.91])
right_target = np.array([0.14, -0.02, 0.05, -1.57, -0.05, 1.50, 0.91])

franka.left_robot_move_to_joint_positions(left_target, time_to_go=3.0)
franka.right_robot_move_to_joint_positions(right_target, time_to_go=3.0)
```

### 6. Cleanup

After finishing, stop the launch terminals with `Ctrl+C`.
If stale servers remain, kill them before launching again:

```bash
sudo pkill -9 run_server
```
