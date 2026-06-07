# 智能船舶 3D 控制平台 / Intelligent Ship 3D Control Platform

> 面向智能船舶仿真、模型交互、手动/手势驾驶、路径规划、任务导航、Arduino 硬件联调与 ROS-Gazebo 联合仿真的综合型 3D 控制平台。  
> A comprehensive 3D control and simulation platform for intelligent ship modeling, manual/gesture driving, path planning, waypoint navigation, Arduino hardware integration, and ROS-Gazebo co-simulation.

---

## 1. Project Overview / 项目概述

本项目以浏览器端 3D 可视化控制面板为核心，结合 **Three.js + Cannon.js** 实现船舶模型加载、物理浮力仿真、水面波浪、天气时间环境、路径规划、任务导航与交互式驾驶控制。同时项目提供 **Arduino hardware interface** 和 **Python WebSocket bridge server**，用于将浏览器控制指令与真实硬件串口通信连接起来；此外还包含 `ship_navigation_sim` ROS-Gazebo 仿真包，可用于限制水域场景下的导航算法、路径跟踪和实验指标采集。

The platform is designed as a hybrid simulation-and-control system. The browser UI provides real-time 3D visualization, STL model importing, buoyancy-related physical behavior, Gerstner-wave water surface, environment switching, manual driving, gesture-based control, A*/Dijkstra planning and waypoint navigation. The Arduino module enables serial communication with motors, rudder servo and sensors through a Python WebSocket bridge. The ROS-Gazebo package supports autonomous ship navigation experiments in restricted waterways.

---

## 2. Main Features / 核心功能

### 2.1 Web 3D Control Panel / 浏览器 3D 控制面板

- **STL Model Import / STL 模型导入**：支持点击或拖拽上传 STL 船舶模型，并自动加入模型列表。
- **Model Selection & Transform / 模型选择与变换**：支持模型选中、高亮、位置、旋转、体积与密度参数编辑。
- **Physics Simulation / 物理仿真**：基于物理引擎模拟重力、浮力、水体密度、阻力和冲量作用。
- **Gerstner Wave System / Gerstner 波浪系统**：支持多组波浪参数配置，包括波长、波高、方向、速度和随机化。
- **Manual Driving / 手动驾驶**：支持键盘 `W/A/S/D` 控制选中模型前进、后退、左转、右转。
- **Gesture Control / 手势控制**：支持调用摄像头进行手势识别，通过手部位置映射驾驶输入。
- **Camera & Observation / 视角观察**：支持水上、水下、跟随模型、环绕、俯视、正视、侧视和自由观察。
- **Environment System / 时间天气环境**：支持模拟时间轴、时间流速、晴朗/云量/风浪等环境状态变化。
- **Training Scene / 虚拟训练场景**：支持创建封闭区域和动态障碍物，用于航行训练与避障测试。
- **Path Planning / 路径规划**：内置 A* 与 Dijkstra 算法，用于在训练场景中生成可视化路径。
- **Waypoint Navigation / 任务导航**：支持添加航点、启动自动导航和清除任务点。
- **Hardware Monitor / Arduino 硬件监控**：可通过 WebSocket 显示温度、电池、电流/速度、深度、距离、舵角等数据，并下发电机、舵机、急停命令。

### 2.2 Arduino Hardware Interface / Arduino 硬件接口

- 支持 Arduino Uno / Nano / Mega / Leonardo。
- 电机控制接口兼容 L298N / TB6612 等双电机驱动。
- 舵机接口用于 rudder steering。
- 支持温度、电池电压、电流、深度/压力、速度、超声波距离等传感器采集。
- 串口协议支持 `M` 电机控制、`S` 舵机控制、`R` 传感器读取、`P` 参数设置、`T` 测试模式、`H` 心跳。

### 2.3 ROS-Gazebo Co-Simulation / ROS-Gazebo 联合仿真

- ROS 包名：`ship_navigation_sim`
- 提供限制水域仿真世界 `restricted_waterway.world`。
- 提供船舶 URDF 模型、控制器配置、代价地图配置、EKF 定位配置、全局规划和局部规划参数。
- 包含全局规划、DWA 局部规划、Pure Pursuit 路径跟踪、实验运行器和指标采集脚本。
- 支持直线航行、障碍物、动态障碍、狭窄水道等实验 launch 文件。

---

## 3. Project Structure / 项目结构

```text
.
├── index.html                              # Landing page / 平台入口页
├── control-full.html                       # Full control panel / 完整控制面板
├── ship_control_frontend_3d_dashboard.html # Lightweight dashboard / 轻量三维主页面
├── arduino/
│   ├── arduino_control.ino                 # Arduino firmware / Arduino 固件
│   └── bridge_server.py                    # Serial-WebSocket bridge / 串口-WebSocket 桥接服务
└── ros_gazebo_sim/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/                             # Controller, costmap, planner and EKF configs
    ├── launch/                             # Gazebo and navigation launch files
    ├── scripts/                            # Planner, controller, experiment and metrics nodes
    ├── urdf/ship.urdf                      # Ship robot model
    └── worlds/restricted_waterway.world    # Restricted waterway simulation world
```

---

## 4. Requirements / 运行环境

### 4.1 Browser Requirements / 浏览器环境

Recommended:

- Google Chrome / Microsoft Edge / Firefox 最新版本
- WebGL enabled
- Camera permission enabled when using gesture control
- Local HTTP server is recommended instead of directly opening HTML files, especially when using external resources or camera APIs

推荐使用现代浏览器运行，并在启用手势控制时允许摄像头权限。建议通过本地 HTTP 服务打开页面，而不是直接双击 HTML 文件。

### 4.2 Arduino Bridge Requirements / Arduino 桥接环境

```bash
pip install pyserial websockets aiohttp
```

Hardware side:

- Arduino Uno / Nano / Mega / Leonardo
- Motor driver: L298N / TB6612 or compatible module
- Rudder servo
- Optional sensors: temperature, battery voltage, current, depth/pressure, speed sensor, ultrasonic sensor

### 4.3 ROS-Gazebo Requirements / ROS-Gazebo 环境

The ROS package is designed for a Catkin workspace. Typical dependencies include:

- ROS Noetic or compatible ROS 1 environment
- Gazebo + gazebo_ros
- robot_state_publisher / joint_state_publisher
- ros_control / ros_controllers
- diff_drive_controller
- robot_localization
- costmap_2d
- map_server
- rviz

---

## 5. Quick Start / 快速开始

### 5.1 Start Web Platform / 启动 Web 平台

在项目根目录运行本地静态服务器：

```bash
python -m http.server 8000
```

Then open in browser:

```text
http://localhost:8000/index.html
```

入口页提供两个主要入口：

1. **进入控制面板 / Full Control Panel**  
   Open `control-full.html`. This is the recommended page for full simulation, model import, physics settings, driving control, gesture control, path planning, waypoint navigation and Arduino hardware connection.

2. **打开主页面 / Lightweight Dashboard**  
   Open `ship_control_frontend_3d_dashboard.html`. This page focuses on model display, basic scene interaction, sensor data and environment switching.

---

## 6. Platform Operation Guide / 平台操作流程与方法

### Step 1 — Enter the Platform / 进入平台

1. 启动本地服务器后访问 `index.html`。
2. 点击 **进入控制面板** 打开完整仿真界面。
3. 页面左侧为控制面板，中央为 3D 场景，顶部显示连接状态、FPS、模型数量与时间信息。

### Step 2 — Import Ship Model / 导入船舶模型

1. 在左侧 **模型管理 / Model Management** 面板中点击上传区域。
2. 选择 `.stl` 文件，或将 STL 文件直接拖拽到上传区域。
3. 模型加载后会出现在 3D 水域场景中，同时加入模型列表。
4. 单击模型或模型列表项即可选中模型。Selected model will be highlighted and used as the active operation target.

### Step 3 — Adjust Model Transform / 调整模型位姿

在 **模型变换 / Model Transform** 面板中可设置：

- `X / Y / Z` position：模型位置
- `X / Y / Z` rotation：模型旋转角度
- `Volume`：模型体积

操作方法：

1. 选中目标模型。
2. 在输入框中修改位置、旋转或体积。
3. 点击 **应用全部 / Apply All** 或直接触发对应输入框变更。
4. 点击 **重置 / Reset** 可恢复当前模型默认变换。

### Step 4 — Configure Density & Buoyancy / 配置密度与浮态

在 **模型密度 / Model Density** 面板中输入模型密度，单位为 `kg/m³`。平台会结合体积、密度、水体密度和物理参数更新模型浮态表现。

Typical density references:

- Water / 水：`1000 kg/m³`
- Default model density / 默认模型密度：`250 kg/m³`
- Wood / 木材：约 `500 kg/m³`
- Plastic / 塑料：约 `950 kg/m³`

Operation:

1. Select the model.
2. Input density value.
3. Observe the hydrostatic readout and model behavior in the water scene.

### Step 5 — Configure Physics / 配置物理引擎

在 **物理引擎 / Physics Engine** 面板中可以调整：

- Gravity / 重力
- Water density / 水体密度
- Drag coefficient / 阻力系数
- Physics pause/resume / 暂停或恢复物理
- Apply impulse / 对当前模型施加冲量

建议在模型导入后先确认密度与体积，再调整物理参数，以获得更稳定的浮态和运动表现。

### Step 6 — Configure Camera and Environment / 配置视角与时间天气

在 **视角与时间环境 / View & Environment** 面板中：

1. 点击 **水上视角 / Above-water View** 查看水面上方场景。
2. 点击 **水下视角 / Underwater View** 查看水下环境。
3. 点击 **跟随模型 / Follow Model** 让相机自动跟随当前选中模型。
4. 拖动时间轴改变模拟时间。
5. 调整时间流速以控制昼夜变化速度。
6. 观察天气、风速和浪高变化对场景表现的影响。

### Step 7 — Configure Wave System / 配置波浪系统

在 **波浪系统 / Wave System** 面板中：

1. 设置波浪组数。
2. 选择某一组波浪。
3. 配置 wavelength、amplitude、direction 和 speed。
4. 点击 **随机 / Randomize** 可生成不同海况。
5. 点击 **重置 / Reset** 恢复默认 Gerstner wave 参数。

The wave system is useful for testing model stability and driving behavior under different sea states.

### Step 8 — Manual Keyboard Driving / 键盘手动驾驶

1. 先导入并选中一个模型。
2. 在 **驾驶控制 / Driving Control** 面板中确认驾驶模型。
3. 设置最大速度和加速度。
4. 选择 **键盘 WASD / Keyboard WASD**。
5. 点击 **启动驾驶 / Start Driving**。
6. 使用键盘控制：

```text
W : Forward / 前进
S : Backward / 后退
A : Turn Left / 左转
D : Turn Right / 右转
```

注意：当前控制目标应为选中的模型。若切换模型，请先点击目标模型或模型列表项，再继续驾驶。

### Step 9 — Gesture Driving / 手势驾驶

1. 使用支持摄像头权限的浏览器打开 `control-full.html`。
2. 导入并选中需要控制的模型。
3. 在 **驾驶控制 / Driving Control** 中选择 **手势操控 / Gesture Control**。
4. 允许浏览器访问摄像头。
5. 点击 **启动驾驶 / Start Driving**。
6. 将手放入摄像头识别区域，通过手部方向与位置控制模型运动。

Gesture control maps hand movement to driving commands. Keep the hand clearly visible and avoid strong backlight for better recognition.

### Step 10 — Create Training Scene / 创建虚拟训练场景

在 **虚拟训练场景 / Virtual Training Scene** 面板中：

1. 设置封闭区域尺寸：`X` length、`Z` width、`Y` height。
2. 设置区域位置。
3. 调整墙壁透明度。
4. 点击 **创建封闭区域 / Create Enclosure**。
5. 点击 **动态障碍 / Dynamic Obstacle** 添加运动障碍物。

This mode is suitable for collision-avoidance tests, navigation training and path planning experiments.

### Step 11 — Run Path Planning / 执行路径规划

在 **路径规划 / Path Planning** 面板中：

1. 选择算法：`A*` 或 `Dijkstra`。
2. 点击 **规划 / Plan**。
3. 查看路径统计：nodes、planning time、path length。
4. 点击 **清除路径 / Clear Path** 可移除当前路径。

A* is generally faster for heuristic grid search, while Dijkstra can be used as a baseline shortest-path method.

### Step 12 — Waypoint Navigation / 任务航点导航

在 **任务导航 / Mission Navigation** 面板中：

1. 点击 **添加航点 / Add Waypoint** 创建目标点。
2. 添加多个航点后点击 **自动导航 / Auto Navigation**。
3. 平台会按航点顺序驱动目标模型执行任务。
4. 点击 **清除 / Clear** 删除所有航点。

### Step 13 — Observation Modes / 展示观察模式

在 **展示观察 / Observation** 面板中可以切换：

- Orbit / 环绕观察
- Top / 俯视
- Front / 正视
- Side / 侧视
- Free / 自由观察

These modes help inspect model attitude, wave interaction, navigation path and collision situation from different perspectives.

### Step 14 — Arduino Hardware Connection / Arduino 硬件连接

#### 14.1 Upload Firmware / 上传固件

1. 使用 Arduino IDE 打开：

```text
arduino/arduino_control.ino
```

2. 选择开发板和串口。
3. 上传固件到 Arduino。
4. 确认电机、舵机和传感器接线符合代码中的 pin definitions。

#### 14.2 Start Bridge Server / 启动桥接服务

Install dependencies:

```bash
pip install pyserial websockets aiohttp
```

Run with auto-detected serial port:

```bash
python arduino/bridge_server.py
```

Or specify serial port and WebSocket port:

```bash
python arduino/bridge_server.py --port COM3 --ws-port 8080
```

Linux example:

```bash
python arduino/bridge_server.py --port /dev/ttyUSB0 --ws-port 8080
```

#### 14.3 Connect from Web UI / 在网页端连接硬件

1. 打开 `control-full.html`。
2. 找到 **Arduino 硬件 / Arduino Hardware** 面板。
3. WebSocket 地址默认：

```text
ws://localhost:8080
```

4. 点击 **连接 / Connect**。
5. 连接成功后可查看 temperature、battery、speed、depth、distance、rudder angle 等数据。
6. 使用按钮发送控制命令：

```text
M50,50  : Forward motor command / 双电机前进
M0,0    : Stop motors / 停止
S90     : Set rudder to 90 degrees / 舵角 90°
R       : Read sensor data / 读取传感器
ESTOP   : Emergency stop / 急停
```

---

## 7. ROS-Gazebo Simulation Guide / ROS-Gazebo 仿真流程

### 7.1 Copy Package to Catkin Workspace / 放入 Catkin 工作空间

```bash
mkdir -p ~/catkin_ws/src
cp -r ros_gazebo_sim ~/catkin_ws/src/ship_navigation_sim
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 7.2 Launch Simulation World / 启动仿真世界

```bash
roslaunch ship_navigation_sim world_only.launch
```

### 7.3 Launch Full Simulation / 启动完整仿真

```bash
roslaunch ship_navigation_sim full_simulation.launch
```

### 7.4 Launch Navigation Stack / 启动导航模块

```bash
roslaunch ship_navigation_sim navigation.launch
```

### 7.5 Run Experiments / 运行实验场景

```bash
roslaunch ship_navigation_sim experiment1_straight.launch
roslaunch ship_navigation_sim experiment2_obstacles.launch
roslaunch ship_navigation_sim experiment3_dynamic.launch
roslaunch ship_navigation_sim experiment4_narrow.launch
```

The scripts directory includes:

- `global_planner_node.py`：global path planning
- `dwa_planner_node.py`：local dynamic window approach planner
- `pure_pursuit_node.py`：path tracking controller
- `experiment_runner.py`：experiment automation
- `metrics_collector.py`：metrics collection, including path length, planning time, navigation time, lateral error, heading variation, safety distance and collision count

---

## 8. Control Protocol / 控制协议

Arduino serial commands follow a compact line-based protocol:

```text
M<L>,<R>   Motor control, left/right speed range -255~255
S<ANGLE>   Servo rudder angle, range 0~180
R          Request sensor data
P<ID>=<V>  Set parameter
T<0/1>     Enable or disable test mode
H          Heartbeat
ESTOP      Emergency stop
```

Example:

```text
M120,120
S90
R
ESTOP
```

---

## 9. Recommended Workflow / 推荐使用流程

For software-only simulation:

```text
Start local HTTP server
→ Open index.html
→ Enter control-full.html
→ Import STL ship model
→ Set volume and density
→ Adjust physics and waves
→ Select model
→ Start keyboard or gesture driving
→ Create training scene
→ Run path planning or waypoint navigation
```

For hardware-in-the-loop testing:

```text
Upload Arduino firmware
→ Connect sensors, motors and rudder servo
→ Start bridge_server.py
→ Open control-full.html
→ Connect ws://localhost:8080
→ Observe live sensor data
→ Send motor/rudder commands from Web UI
→ Test emergency stop and sensor feedback
```

For ROS-Gazebo experiments:

```text
Copy ros_gazebo_sim into catkin_ws/src
→ catkin_make
→ source devel/setup.bash
→ launch world/full simulation/navigation
→ run experiment launch files
→ collect metrics for algorithm evaluation
```

---

## 10. Notes / 注意事项

- 手势控制需要浏览器摄像头权限，并建议使用 HTTPS 或 localhost 环境。
- 如果 STL 模型过大，浏览器加载和物理计算可能变慢，建议提前简化网格。
- 模型体积、密度和水体密度会显著影响浮态表现，应根据实际模型尺度进行校准。
- Arduino 硬件测试前请先空载验证电机方向、舵机角度和急停逻辑。
- ROS-Gazebo 部分依赖本机 ROS 环境，运行前需确认相关 packages 已安装。
- 若 Gazebo 中引用的外部 mesh/model 缺失，需要补充对应模型资源或替换为简化几何体。

---

## 11. License / 许可证

This project is released under the MIT License as declared in the ROS package metadata.  
本项目按照 ROS 包元信息中声明的 MIT License 使用。

---

## 12. Summary / 项目总结

智能船舶 3D 控制平台将 Web 端交互式三维仿真、物理浮态建模、手动/手势驾驶、虚拟训练环境、路径规划、任务导航、Arduino 硬件通信和 ROS-Gazebo 算法实验整合到同一工程中。它既可作为教学演示平台，也可作为智能船舶控制算法、传感器联调和人机交互控制方法的原型验证系统。

The system can be used as an educational visualization tool, a prototype for intelligent ship control, a testbed for navigation algorithms, and a bridge between browser-based simulation and embedded hardware experiments.
