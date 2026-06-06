# Intelligent Ship Navigation & 3D Control Platform

# 智能船舶三维控制与导航仿真平台

## 1. Project Overview / 项目概述

The **Intelligent Ship Navigation & 3D Control Platform** is a web-based 3D visualization and interactive control system for intelligent ship navigation, virtual simulation, manual driving, gesture-based control, and hardware/simulation extension. The platform is deployed online and can be accessed directly through a browser:

**Project Website / 项目网页：**
https://johnyhonf.github.io/ship-navigation/

本项目基于 Web 端三维可视化技术构建智能船舶三维控制与仿真平台。前端以 **Three.js** 为核心渲染框架，通过加载船舶模型、三维水域场景、交互式控制面板和动态控制模块，完成智能船舶虚拟运行环境的搭建。用户可通过浏览器访问项目网页，进行三维展示、手动驾驶、键盘控制、YOYO 手势驾驶、航行状态观察、姿态控制与交互式操作。

The system integrates model loading, 3D scene rendering, manual control, keyboard driving, YOYO gesture recognition, ship motion response, Arduino hardware interface, Flask-SocketIO communication, and ROS/Gazebo simulation support. It forms a complete workflow of:

**Model Import → 3D Visualization → Driving Control → Gesture Interaction → Hardware Communication → Simulation Verification**

该平台可为后续智能船舶路径规划、航行控制算法验证、虚实结合测试、无人船人机交互研究以及智能航行系统演示提供基础支撑。

---

## 2. Key Features / 核心功能

### 2.1 Web-Based 3D Visualization / Web端三维可视化

The platform provides a browser-accessible 3D interface for displaying and controlling an intelligent ship model.

平台提供基于浏览器的三维可视化界面，可用于智能船舶模型展示、视角观察和交互控制。

Main capabilities include:

* 3D ship model loading and rendering
* 三维船舶模型加载与渲染
* Interactive virtual water environment
* 虚拟水域环境交互展示
* Camera view switching and observation
* 摄像机视角切换与观察
* Ship pose and movement visualization
* 船舶姿态与运动状态可视化
* Web-based access without installing desktop software
* 无需安装桌面软件，浏览器即可访问

---

### 2.2 Manual Driving Control / 手动驾驶控制

The platform supports manual ship control through the web interface and keyboard input. Users can control the ship movement in real time, observe its response, and test basic navigation behavior.

平台支持通过网页控制按钮和键盘输入对船舶进行手动驾驶。用户可以实时控制船舶前进、后退、左转、右转和停止，并观察模型的动态响应。

Typical keyboard control method:

| Key | Function           |
| --- | ------------------ |
| `W` | Move Forward / 前进  |
| `S` | Move Backward / 后退 |
| `A` | Turn Left / 左转     |
| `D` | Turn Right / 右转    |

---

### 2.3 YOYO Gesture-Based Control / YOYO手势驾驶控制

The platform includes a YOYO gesture control module. By using the browser camera, the system recognizes the user’s hand movement and maps palm trajectory to ship driving commands.

平台集成 YOYO 手势驾驶模块，可调用浏览器摄像头识别用户手部动作，并将掌心运动轨迹映射为船舶控制指令。

Gesture control characteristics:

* Open-hand activation
* 张开手掌后进入控制状态
* Palm movement recognition
* 识别掌心前后、左右运动
* Gesture-based forward, backward, left-turn and right-turn commands
* 支持手势控制前进、后退、左转、右转
* Automatic stop when the hand is lost or the gesture is released
* 手部丢失或手势释放后自动停止
* Real-time camera preview and gesture feedback
* 实时摄像头预览与手势状态反馈

This module provides a natural human–machine interaction method for intelligent ship control and demonstration.

该模块为智能船舶控制提供了更加自然的人机交互方式，适合用于演示、实验和交互式教学场景。

---

### 2.4 Ship Motion and Buoyancy Response / 船舶运动与浮力响应

The system includes basic ship motion response and buoyancy-related visualization logic, allowing the ship model to respond dynamically during control operations.

系统包含基础船舶运动响应与浮力表现逻辑，使船舶模型在控制过程中具备一定的动态反馈效果。

It supports:

* Ship translation and rotation
* 船舶平移与转向
* Floating response visualization
* 浮力响应可视化
* Motion state feedback
* 运动状态反馈
* More realistic interaction during driving control
* 提升驾驶交互过程中的真实感

---

### 2.5 Hardware Interface Extension / 硬件接口扩展

The project includes Arduino-related files and backend bridge logic, which can be used for future hardware-in-the-loop experiments.

项目包含 Arduino 相关控制文件和后端桥接逻辑，可用于后续硬件在环实验和物理船模控制扩展。

Potential hardware-related functions include:

* Serial communication with Arduino
* 与 Arduino 进行串口通信
* Control command forwarding
* 控制指令转发
* Connection between web control and physical actuator systems
* Web 控制端与物理执行机构之间的连接
* Prototype testing for intelligent ship hardware
* 智能船舶硬件原型测试

---

### 2.6 Backend Communication Support / 后端通信支持

The platform includes a Flask-SocketIO backend interface, which can be extended for real-time data communication between the web frontend, hardware modules, and simulation systems.

平台包含 Flask-SocketIO 后端通信接口，可扩展用于 Web 前端、硬件模块和仿真系统之间的实时数据交互。

Possible extensions include:

* Real-time command transmission
* 实时控制指令传输
* Multi-client monitoring
* 多客户端监控
* Sensor data visualization
* 传感器数据可视化
* Integration with external control algorithms
* 与外部控制算法集成

---

### 2.7 ROS/Gazebo Simulation Support / ROS/Gazebo仿真支持

The project also contains ROS/Gazebo simulation-related modules, which can be used for navigation algorithm verification, path planning experiments, and simulation-based testing.

项目包含 ROS/Gazebo 仿真相关模块，可用于航行控制算法验证、路径规划实验和仿真测试。

The simulation module can support:

* Ship model simulation
* 船舶模型仿真
* Waterway environment construction
* 水域环境构建
* Navigation algorithm testing
* 导航算法测试
* Planner parameter configuration
* 规划器参数配置
* Experiment data collection
* 实验数据采集

---

## 3. System Workflow / 系统工作流程

The platform follows the following technical workflow:

平台整体技术链路如下：

```text
Model Loading
模型加载
    ↓
3D Scene Rendering
三维场景渲染
    ↓
User Interaction
用户交互
    ↓
Manual / Keyboard / Gesture Control
手动控制 / 键盘控制 / 手势控制
    ↓
Ship Motion Response
船舶运动响应
    ↓
Hardware or Simulation Extension
硬件接口或仿真系统扩展
    ↓
Navigation Algorithm Verification
航行算法验证
```

---

## 4. Platform Operation Guide / 平台操作流程

### 4.1 Access the Platform / 访问平台

Open the following URL in a modern browser:

在浏览器中打开以下网址：

```text
https://johnyhonf.github.io/ship-navigation/
```

Recommended browsers:

推荐浏览器：

* Google Chrome
* Microsoft Edge
* Firefox

For camera-based gesture control, Chrome or Edge is recommended.

如需使用摄像头手势控制，建议使用 Chrome 或 Edge 浏览器。

---

### 4.2 Basic Page Loading / 页面加载

After opening the website, wait for the page and 3D resources to load.

打开网页后，等待页面和三维资源加载完成。

If the model or scene does not appear immediately:

如果模型或场景没有立即显示：

1. Wait for several seconds.
   等待数秒。
2. Refresh the page.
   刷新页面。
3. Check whether the browser blocks scripts or camera permissions.
   检查浏览器是否阻止脚本或摄像头权限。

---

### 4.3 3D Scene Observation / 三维场景观察

Users can observe the ship model and virtual environment through mouse operations.

用户可以通过鼠标操作观察船舶模型和虚拟环境。

Common operations:

常见操作方式：

| Operation        | Function              |
| ---------------- | --------------------- |
| Left mouse drag  | Rotate view / 旋转视角    |
| Right mouse drag | Pan view / 平移视角       |
| Mouse wheel      | Zoom in or out / 缩放视角 |

The user may also switch camera views if the interface provides camera buttons or view controls.

如果界面中提供摄像机按钮或视角切换功能，也可以通过按钮切换观察视角。

---

### 4.4 Keyboard Driving / 键盘驾驶方法

Click on the 3D scene or page area first to make sure the browser receives keyboard input.

使用键盘控制前，建议先点击页面或三维场景区域，确保浏览器可以接收键盘输入。

Keyboard control:

键盘控制方式：

```text
W：Forward / 前进
S：Backward / 后退
A：Turn Left / 左转
D：Turn Right / 右转
```

Operation steps:

操作步骤：

1. Open the platform website.
   打开平台网页。
2. Wait until the ship model is loaded.
   等待船舶模型加载完成。
3. Click inside the page.
   点击页面空白区域或三维场景区域。
4. Press `W`, `A`, `S`, `D` to control the ship.
   按下 `W`、`A`、`S`、`D` 控制船舶运动。
5. Release the key to stop or reduce the corresponding control command.
   松开按键后停止或减弱对应控制输入。

---

### 4.5 Manual Button Control / 手动按钮控制

If the interface provides control buttons, users can also operate the ship through the on-screen control panel.

如果界面中提供了控制按钮，用户也可以通过网页控制面板操作船舶。

Typical control buttons may include:

常见控制按钮包括：

* Forward / 前进
* Backward / 后退
* Turn Left / 左转
* Turn Right / 右转
* Stop / 停止
* Reset / 重置

Operation method:

操作方法：

1. Locate the control panel on the webpage.
   找到网页中的控制面板。
2. Click the corresponding button.
   点击对应控制按钮。
3. Observe the movement and attitude response of the ship model.
   观察船舶模型的运动和姿态变化。
4. Use the stop or reset function when necessary.
   必要时使用停止或重置功能。

---

### 4.6 YOYO Gesture Driving / YOYO手势驾驶方法

The YOYO gesture control mode allows users to control the ship through hand movements captured by the camera.

YOYO 手势驾驶模式允许用户通过摄像头识别手部动作来控制船舶。

Before using this function:

使用该功能前请确认：

* The device has a working camera.
  设备具备可用摄像头。
* The browser allows camera access.
  浏览器允许网页访问摄像头。
* The page is opened through HTTPS or localhost.
  页面通过 HTTPS 或 localhost 打开。
* The hand is clearly visible in front of the camera.
  手部在摄像头前清晰可见。

Gesture control steps:

手势控制步骤：

1. Open the platform website.
   打开平台网页。
2. Find and enable the gesture control function.
   找到并开启手势控制功能。
3. Allow camera permission when the browser asks.
   浏览器弹出权限请求时，允许摄像头访问。
4. Place your hand in front of the camera.
   将手放在摄像头前。
5. Open your palm to activate gesture recognition.
   张开手掌以激活手势识别。
6. Move the palm forward, backward, left, or right to control the ship.
   移动掌心完成前进、后退、左转、右转控制。
7. Remove the hand or close the palm to stop gesture control.
   移开手部或收起手掌后停止手势控制。

Suggested gesture mapping:

建议手势映射方式：

| Gesture Movement                | Ship Response      |
| ------------------------------- | ------------------ |
| Palm moves upward or forward    | Move forward / 前进  |
| Palm moves downward or backward | Move backward / 后退 |
| Palm moves left                 | Turn left / 左转     |
| Palm moves right                | Turn right / 右转    |
| Hand lost or closed             | Stop / 停止          |

---

### 4.7 Reset and Re-Operation / 重置与重新操作

If the model position becomes abnormal or the system response becomes unstable, users can reset the scene or refresh the page.

如果模型位置异常或系统响应不稳定，可以重置场景或刷新页面。

Recommended method:

推荐处理方式：

1. Stop current control input.
   停止当前控制输入。
2. Click the reset button if available.
   如果页面提供重置按钮，点击重置。
3. If the issue continues, refresh the webpage.
   如果问题仍然存在，刷新网页。
4. Re-enable manual or gesture control.
   重新开启手动控制或手势控制。

---

## 5. Deployment Method / 项目部署方法

This project can be deployed as a static website through GitHub Pages.

本项目可通过 GitHub Pages 作为静态网页部署。

Basic deployment steps:

基本部署流程：

1. Upload all project files and folders to the root directory of the GitHub repository.
   将项目全部文件和文件夹上传至 GitHub 仓库根目录。

2. Make sure `index.html` is located in the repository root.
   确保 `index.html` 位于仓库根目录。

3. Make sure `.nojekyll` is retained.
   保留 `.nojekyll` 文件，避免 GitHub Pages 处理静态资源时出现路径问题。

4. If using GitHub Actions, make sure the workflow file exists:
   如果使用 GitHub Actions 部署，确保存在工作流文件：

```text
.github/workflows/pages.yml
```

5. Open repository settings:
   打开仓库设置：

```text
Settings → Pages
```

6. Select deployment source:
   设置部署来源：

```text
GitHub Actions
```

7. Run the deployment workflow in the `Actions` tab.
   在 `Actions` 页面运行部署工作流。

8. After successful deployment, open the GitHub Pages URL.
   部署成功后，打开 GitHub Pages 网页链接。

---

## 6. Recommended Repository Structure / 推荐项目结构

The recommended project structure is:

推荐项目结构如下：

```text
ship-navigation/
├── index.html
├── control-full.html
├── ship_control_frontend_3d_dashboard.html
├── 智能船舶3D控制平台.html
├── 404.html
├── .nojekyll
├── .github/
│   └── workflows/
│       └── pages.yml
├── arduino/
│   ├── arduino_control.ino
│   └── bridge_server.py
├── ros_gazebo_sim/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   ├── launch/
│   ├── scripts/
│   ├── urdf/
│   └── worlds/
├── ship_control_backend_flask_socketio.py
├── ship_control_arduino_hardware_interface.c
├── YOYO手势修改说明.txt
└── GITHUB_PAGES_部署说明.txt
```

Important note:

注意：

```text
index.html must be placed directly in the repository root.
index.html 必须直接放在仓库根目录。
```

Do not upload the entire outer folder as an extra nested directory.

不要把整个外层文件夹作为额外目录上传，否则 GitHub Pages 可能无法正确识别首页文件。

---

## 7. Local Running Method / 本地运行方法

Although the frontend can be opened directly in a browser, it is recommended to run it through a local server for better compatibility.

虽然前端页面可以直接用浏览器打开，但为了保证模型加载、脚本运行和摄像头权限更加稳定，建议使用本地服务器运行。

Using Python:

使用 Python 启动本地服务器：

```bash
python -m http.server 8000
```

Then open:

然后在浏览器中访问：

```text
http://localhost:8000
```

If using gesture control locally, `localhost` is usually allowed by browsers for camera access.

如果本地使用手势控制，浏览器通常允许 `localhost` 调用摄像头。

---

## 8. Application Scenarios / 应用场景

This platform can be used in the following scenarios:

该平台可应用于以下场景：

* Intelligent ship control demonstration
  智能船舶控制演示
* Web-based marine robotics visualization
  Web 端船舶机器人可视化
* Human–machine interaction research
  人机交互研究
* Gesture-based ship driving experiments
  手势控制船舶实验
* Navigation algorithm verification
  航行控制算法验证
* Hardware-in-the-loop prototype testing
  硬件在环原型测试
* ROS/Gazebo-based ship simulation
  ROS/Gazebo 船舶仿真
* Teaching and experimental demonstration
  教学与实验展示

---

## 9. Notes / 注意事项

* Camera access is required for YOYO gesture control.
  YOYO 手势控制需要摄像头权限。

* For best performance, use Chrome or Edge on a desktop computer.
  为获得更好性能，建议使用桌面端 Chrome 或 Edge 浏览器。

* If the page becomes slow, close other browser tabs and refresh the page.
  如果页面运行卡顿，可关闭其他浏览器标签页并刷新页面。

* If keyboard control does not work, click inside the webpage first.
  如果键盘控制无响应，请先点击网页区域。

* If GitHub Pages shows a 404 error, check whether `index.html` is in the repository root.
  如果 GitHub Pages 出现 404，请检查 `index.html` 是否位于仓库根目录。

* If gesture control cannot start, check browser camera permission and HTTPS access.
  如果手势控制无法启动，请检查浏览器摄像头权限和 HTTPS 访问环境。

---

## 10. Future Work / 后续扩展方向

Future development may include:

后续可进一步扩展：

* Autonomous path planning
  自主路径规划
* Real-time sensor data visualization
  实时传感器数据可视化
* LiDAR or radar simulation
  激光雷达或雷达仿真
* Multi-ship cooperative control
  多船协同控制
* More accurate hydrodynamic modeling
  更精确的船舶水动力建模
* Digital twin-based ship monitoring
  基于数字孪生的船舶监控
* Integration with physical unmanned surface vehicles
  与实体无人船平台结合
* Advanced gesture command recognition
  高级手势指令识别

---

## 11. License / 开源许可

This project is intended for academic research, engineering demonstration, and educational use. If the project is released publicly or reused by others, it is recommended to add an appropriate open-source license.

本项目可用于学术研究、工程演示和教学实验。如需公开发布或供他人复用，建议补充合适的开源许可证。

---

## 12. Acknowledgements / 致谢

This project integrates web-based 3D visualization, intelligent ship control, gesture interaction, hardware communication, and simulation extension. It provides a lightweight and extensible platform for intelligent ship navigation research and interactive demonstration.

本项目融合 Web 三维可视化、智能船舶控制、手势交互、硬件通信与仿真扩展能力，为智能船舶导航控制研究和交互式演示提供了轻量化、可扩展的平台基础。
