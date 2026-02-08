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
