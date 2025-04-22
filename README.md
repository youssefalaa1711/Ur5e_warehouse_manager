# 🤖 UR5e Warehouse Expiry Detection System

This ROS 2 project implements a semi-automated warehouse management system using a **Universal Robots UR5e** robotic arm and a **camera-based OCR system** to identify expired products on a shelf and remove them. The goal is to simulate and deploy an intelligent robotic assistant that helps manage inventory by checking expiry dates and physically handling outdated stock.

---

## 🧠 Project Overview

In warehouse environments, expired goods are a critical issue. This system leverages **robot vision**, **robot motion planning**, and **custom URDF modeling** to automate part of this inspection process:

1. A camera observes product labels on a shelf.
2. OCR is used to detect printed expiry dates.
3. If the date is expired, a Python node sends movement commands to the UR5e.
4. The robot arm pushes the expired box off the shelf.
5. The system can be tested and visualized in **RViz**, or deployed in a real lab setup.

---

## 🖼️ Visual Demonstration

### 🧪 Simulation View (RViz)

This is the simulated warehouse environment showing the UR5e robot, a shelf, and visualized workspace.

![Simulation View](docs/simulation_image.png)

---

### 🏭 Real-World Lab Setup

This is the actual physical setup used in the robotics lab. The UR5e is mounted on a frame and observes cardboard boxes labeled with expiry dates.

![Real Setup](docs/real_setup_image.jpeg)

---

## 🗂️ Project Folder Structure

```
project_ws/
├── src/
│   └── ur5e_robot/               # ROS 2 Package
│       ├── ur5e_robot/           # Python nodes (camera, move control, OCR logic)
│       ├── test/                 # Python lint and format tests
│       ├── resource/             # Package resources
│       ├── package.xml           # Package manifest
│       └── setup.py              # Build instructions
├── ur_macro.xacro                # Macro to define URDF robot/environment
├── urdf modified backup/         # Real-world inspired URDF with shelf and inverted arm
├── Rviz_commands/                # Saved commands or RViz config used to simulate the system
└── docs/
    ├── simulation_image.png
    └── real_setup_image.jpeg
```

---

## ⚙️ System Workflow

1. **OCR Node**:
   - Captures image from camera.
   - Uses Tesseract OCR (or OpenCV) to extract expiry text.
   - Publishes detection results.

2. **Decision Node**:
   - Parses the detected expiry date.
   - Compares it with the current date.
   - If expired, triggers a robot command node.

3. **UR5e Control Node**:
   - Receives signal and sends trajectory/movement commands.
   - Moves the UR5e to push the expired product.

---

## 🛠️ Technologies Used

- ROS 2 (Humble or compatible)
- URDF / Xacro for robot and environment modeling
- RViz for robot simulation and visualization
- Tesseract OCR or OpenCV (Python-based vision)
- Universal Robots UR5e

---

## 🚀 How to Run the Project

```bash
# Clone and build the workspace
cd ~/ros2_ws/src
git clone https://github.com/youssefalaa1711/ur5e_warehouse_manager.git
cd ..
colcon build
source install/setup.bash

# Launch RViz or your full system (adjust as needed)
ros2 launch ur5e_robot your_launch_file.launch.py
```

---

## 📜 License

This project is licensed under the **MIT License**. See the LICENSE file for usage terms.

---

**Author**: [@youssefalaa1711](https://github.com/youssefalaa1711)  
"# Ur5e_warehouse_manager" 
