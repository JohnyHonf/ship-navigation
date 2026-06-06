# Intelligent Ship 3D Control Platform

## Overview

The **Intelligent Ship 3D Control Platform** is a web-based interactive control and simulation system designed for smart vessel navigation, human–machine interaction, and autonomous control experiments. The project integrates a 3D frontend dashboard, gesture-based control, keyboard driving, Arduino hardware communication, Flask-SocketIO backend interfaces, and ROS/Gazebo simulation modules.

This platform is intended for research, demonstration, and prototyping of intelligent ship control systems, especially in scenarios involving restricted waterways, path planning, obstacle avoidance, and multimodal control interaction.

## Key Features

### 1. Web-Based 3D Ship Control Dashboard

The project provides a browser-accessible 3D control interface for visualizing and controlling an intelligent ship model. The frontend includes:

* 3D ship visualization
* Interactive control panel
* Keyboard-based manual driving
* Model loading and display
* Real-time control state feedback
* Browser-based access through GitHub Pages or local deployment

Main frontend files include:

* `index.html`
* `control-full.html`
* `ship_control_frontend_3d_dashboard.html`
* `智能船舶3D控制平台.html`

### 2. YOYO Gesture-Based Control

The platform includes an enhanced YOYO gesture control mode based on palm trajectory recognition. The gesture control logic is designed to improve stability and reduce accidental triggering.

Main control logic includes:

* Gesture activation only when four or five fingers are detected
* Automatic stop when fewer than four fingers are detected or the hand is lost
* Palm movement toward or away from the camera for forward and backward control
* Palm left/right trajectory for turning control
* Palm trajectory buffering
* Slow depth-baseline self-calibration
* Dead zone and release zone design
* Exponential smoothing for reducing jitter
* Startup calibration frames for more stable initialization
* Real-time hand skeleton, fingertip, palm center, and trajectory visualization in camera preview

This design allows the user to control the ship naturally by opening the palm and moving the hand in space.

### 3. Keyboard Manual Control

In addition to gesture interaction, the system supports traditional keyboard-based manual driving. This provides a simple and reliable fallback control mode for testing, debugging, and direct operation.

Typical keyboard control logic follows the WASD-style driving convention:

* Forward
* Backward
* Turn left
* Turn right
* Stop or release control input

### 4. Arduino Hardware Interface

The project contains Arduino-related control files for connecting the web or backend control logic with physical hardware.

Relevant files include:

* `arduino/arduino_control.ino`
* `arduino/bridge_server.py`
* `ship_control_arduino_hardware_interface.c`

These files provide a foundation for serial communication, hardware signal output, and integration with embedded control devices.

### 5. Flask + SocketIO Backend Interface

The project includes a Python backend service based on Flask and SocketIO. This backend can be used as a communication bridge between the frontend dashboard, hardware interface, and external control modules.

Relevant file:

* `ship_control_backend_flask_socketio.py`

The backend is suitable for:

* Real-time control command forwarding
* WebSocket communication
* Hardware or simulation data exchange
* Future extension to remote monitoring and control

### 6. ROS/Gazebo Simulation Support

The project includes a ROS/Gazebo simulation package for intelligent ship navigation experiments.

Main simulation directory:

```text
ros_gazebo_sim/
```

The simulation module includes:

* ROS package configuration
* Gazebo world files
* Ship URDF model
* Navigation launch files
* Controller configuration
* Costmap configuration
* DWA planner parameters
* Pure pursuit parameters
* Global planner parameters
* EKF localization configuration
* Experiment launch files
* Metrics collection scripts

Important files include:

```text
ros_gazebo_sim/package.xml
ros_gazebo_sim/CMakeLists.txt
ros_gazebo_sim/urdf/ship.urdf
ros_gazebo_sim/worlds/restricted_waterway.world
ros_gazebo_sim/config/controller.yaml
ros_gazebo_sim/config/dwa_params.yaml
ros_gazebo_sim/config/pure_pursuit_params.yaml
ros_gazebo_sim/config/ekf_localization.yaml
ros_gazebo_sim/scripts/global_planner_node.py
ros_gazebo_sim/scripts/dwa_planner_node.py
ros_gazebo_sim/scripts/pure_pursuit_node.py
ros_gazebo_sim/scripts/metrics_collector.py
ros_gazebo_sim/scripts/experiment_runner.py
```

The simulation environment is designed for evaluating ship navigation behavior in restricted waterways and obstacle-rich scenarios.

## Project Structure

```text
.
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
├── ship_control_arduino_hardware_interface.c
├── ship_control_backend_flask_socketio.py
├── YOYO手势修改说明.txt
└── GITHUB_PAGES_部署说明.txt
```

## Deployment on GitHub Pages

This project supports static deployment through GitHub Pages using GitHub Actions.

### Deployment Steps

1. Upload all project files to the root directory of your GitHub repository.
2. Make sure the following files and folders are located directly in the repository root:

```text
index.html
control-full.html
ship_control_frontend_3d_dashboard.html
.github/
.nojekyll
```

3. Open the repository on GitHub.
4. Go to:

```text
Settings -> Pages
```

5. Set the source to:

```text
GitHub Actions
```

6. Go to the `Actions` tab.
7. Run or wait for the workflow named:

```text
Deploy static site to GitHub Pages
```

8. After successful deployment, open the GitHub Pages URL provided by GitHub.

## GitHub Actions Workflow

The project includes an official GitHub Pages workflow:

```text
.github/workflows/pages.yml
```

This workflow performs the following steps:

* Checks out the repository
* Configures GitHub Pages
* Uploads the static site artifact
* Deploys the project to GitHub Pages

The workflow supports both `main` and `master` branches and can also be triggered manually through `workflow_dispatch`.

## Local Usage

Because the frontend is primarily static HTML, it can be opened directly in a browser. However, for better compatibility with browser security policies, camera access, and module loading, it is recommended to run a local static server.

Example using Python:

```bash
python -m http.server 8000
```

Then open:

```text
http://localhost:8000
```

For gesture control features, the browser may require camera permission. Some browsers only allow camera access under `localhost` or HTTPS.

## Gesture Control Usage

1. Open the main dashboard page.
2. Load or select the ship model if required.
3. Enable gesture control.
4. Show an open palm in front of the camera.
5. Keep four or five fingers visible to activate control.
6. Move the palm forward, backward, left, or right to control the ship.
7. Close the hand or move the hand out of the camera view to stop control input.

## Hardware Interface Usage

The Arduino module can be used to connect the control system with physical hardware. A typical workflow is:

1. Upload `arduino/arduino_control.ino` to the Arduino board.
2. Connect the Arduino to the computer through a serial port.
3. Run the bridge server if required:

```bash
python arduino/bridge_server.py
```

4. Connect the frontend or backend control module to the bridge interface.
5. Test command forwarding and hardware response.

The exact serial port and baud rate may need to be adjusted according to the local hardware configuration.

## Backend Usage

The Flask-SocketIO backend can be started with Python after installing the required dependencies.

Example:

```bash
pip install flask flask-socketio
python ship_control_backend_flask_socketio.py
```

The backend can be extended to support:

* Real-time control commands
* Remote monitoring
* Hardware communication
* Simulation data exchange
* Multi-client control interfaces

## ROS/Gazebo Simulation Usage

The `ros_gazebo_sim` folder contains a ROS simulation package. It can be placed inside a ROS catkin workspace.

Example workflow:

```bash
mkdir -p ~/catkin_ws/src
cp -r ros_gazebo_sim ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Launch the full simulation:

```bash
roslaunch ros_gazebo_sim full_simulation.launch
```

Other available launch files include:

```text
navigation.launch
world_only.launch
experiment1_straight.launch
experiment2_obstacles.launch
experiment3_dynamic.launch
experiment4_narrow.launch
```

These launch files are designed for different experimental conditions such as straight navigation, obstacle scenarios, dynamic environments, and narrow waterway tests.

## Research and Application Scenarios

This project can be used for:

* Intelligent ship control demonstration
* Human–machine interaction research
* Gesture-based control experiments
* Autonomous navigation algorithm testing
* Restricted waterway simulation
* ROS/Gazebo navigation experiments
* Web-based digital twin visualization
* Hardware-in-the-loop prototyping
* Educational demonstrations for marine robotics and intelligent transportation systems

## Notes

* The web frontend can be deployed as a static site.
* Camera-based gesture control requires browser permission.
* Hardware control requires correct serial communication settings.
* ROS/Gazebo simulation requires a properly configured ROS environment.
* GitHub Pages deployment requires the files to be placed directly in the repository root.
* The `.nojekyll` file should be retained for static deployment compatibility.

## License

This project is intended for academic research, engineering demonstration, and educational use. Please add a suitable open-source license before public distribution if the project will be shared or reused by others.

## Acknowledgements

This project integrates frontend visualization, gesture interaction, embedded hardware interfaces, backend communication, and ROS/Gazebo simulation to support intelligent ship control research and demonstration.
