# pb2025_infantry_ws

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. 项目介绍

深圳北理莫斯科大学北极熊战队 步兵机器人 ROS 工作空间，包含串口通信、视觉模块。

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Install [OpenVINO 2023.3](https://docs.openvino.ai/2025/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2023_3_0&OP_SYSTEM=LINUX&DISTRIBUTION=APT)

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
sudo pip install vcstool2 xmacro
```

```bash
mkdir -p ~/pb2025_infantry_ws
cd ~/pb2025_infantry_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_infantry_ws.git
```

```bash
vcs import --recursive < dependencies.repos
```

> [!NOTE]
> `dependencies.repos` 文件已包含所有模块所依赖的仓库地址，无需手动查阅子模块的 README 手动克隆依赖。

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 pb2025_infantry_ws 广泛使用了 launch.py 文件和 yaml 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。

### 2.4 Running

### 2.4.1 Vision and Serial

```bash
ros2 launch pb2025_infantry_bringup bringup_launch.py
```

### 2.4.2 Tools

云台键盘控制

```bash
ros2 run teleop_gimbal_keyboard teleop_gimbal_keyboard
```
