# Dual Robotiq Gripper Launch Guide

本文档介绍如何同时 launch 两个 Robotiq 2F 夹爪。

## 架构说明

每个 Gripper 的 launch 由两部分组成：

- **gRPC Server**（`GripperServerLauncher`）：在指定 `ip:port` 上监听用户指令
- **Gripper Client**（`RobotiqGripperClient`）：通过串口（`/dev/ttyUSBx`）与物理夹爪通信，并连接到上述 Server

两个夹爪必须使用**不同的串口**和**不同的 gRPC 端口**：

| 参数 | Gripper 1 | Gripper 2 |
|------|-----------|-----------|
| `comport` | `/dev/ttyUSB0` | `/dev/ttyUSB1` |
| `port` | `50052` | `50053` |

---

## 第一步：确认串口设备

将两个 Robotiq 夹爪通过 USB 连接到计算机，然后确认串口编号：

```bash
ls /dev/ttyUSB*
```

预期输出类似：
```
/dev/ttyUSB0  /dev/ttyUSB1
```

授予串口读写权限：

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

---

## 第二步：新增配置文件

### 2.1 新增第二个 Gripper 的 gripper 配置

在 `polymetis/conf/gripper/` 目录下新建 `robotiq_2f_2.yaml`：

```yaml
# polymetis/conf/gripper/robotiq_2f_2.yaml
gripper:
  _target_: polymetis.robot_client.robotiq_gripper.RobotiqGripperClient
  server_ip: ${ip}
  server_port: ${port}
  comport: /dev/ttyUSB1
```

### 2.2 新增第二个 Gripper 的 launch 配置

在 `polymetis/conf/` 目录下新建 `launch_gripper2.yaml`：

```yaml
# polymetis/conf/launch_gripper2.yaml
defaults:
  - gripper: robotiq_2f_2

ip: 0.0.0.0
port: 50053
timeout: 15
```

> 已在本仓库中创建好上述两个文件，可直接使用。

---

## 第三步：Launch 两个 Gripper

分别在**两个终端**中执行（先激活环境）：

```bash
conda activate polymetis-local
cd polymetis/polymetis/python/scripts
export PYTHONNOUSERSITE=1
```

**终端 1 —— Gripper 1（ttyUSB0，port 50052）：**

```bash
python launch_gripper.py
```

**终端 2 —— Gripper 2（ttyUSB1，port 50053）：**

```bash
python launch_gripper.py --config-name=launch_gripper2
```

也可以不新建配置文件，直接通过命令行参数覆盖：

```bash
python launch_gripper.py port=50053 gripper.comport=/dev/ttyUSB1
```

---

## 第四步：在用户程序中同时访问两个 Gripper

```python
from polymetis import GripperInterface

# 连接两个 gripper server
gripper1 = GripperInterface(ip="localhost", port=50052)
gripper2 = GripperInterface(ip="localhost", port=50053)

# 控制示例
gripper1.goto(width=0.05, speed=0.1, force=10)
gripper2.goto(width=0.08, speed=0.1, force=10)
```

---

## 注意事项

- **串口不可共用**：两个物理夹爪必须接在不同的 USB 串口，否则 Modbus 通信会产生冲突。
- **端口不可相同**：两个 gRPC Server 监听的端口必须不同。
- **`launch_gripper.py` 使用 `os.fork()`**：Server 和 Client 在同一脚本内通过 fork 分离运行，两次 launch 彼此独立、互不干扰。
- **USB 编号可能变化**：每次重新插拔 USB 设备后，`ttyUSB0/1` 的编号可能互换，建议通过 `udev` 规则绑定固定串口名称，或每次 launch 前用 `ls /dev/ttyUSB*` 确认。
