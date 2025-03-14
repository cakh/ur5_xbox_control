# UR5 Xbox Controller

**UR5 Xbox Controller** provides an easy way to control a Universal Robots UR5 robot using an Xbox controller.

## 📌 Overview

This project integrates:
- **RTDE Interface** (Real-Time Data Exchange) for UR robot communication.
- **Xbox controller input** for intuitive robot control.

## ⚙️ Features

- 🎮 Real-time joystick-based control of robot pose.
- 🚀 Adjustable motion limits and speeds.
- ⚙️ Multithreaded architecture.

## 📁 Project Structure
```
ur5_xbox_control/
├── src/
│   ├── main.py                 # Main execution file
│   ├── ur5_control/
│   │   ├── ur5_controller.py   # UR5 robot control logic via RTDE
│   │   └── __init__.py
│   └── xbox/
│       ├── xbox_control.py     # Xbox controller input handling
│       └── __init__.py
└── README.md
```

## 🛠 Installation

### Requirements:
- [RTDE (Universal Robots)](https://github.com/UniversalRobots/RTDE_Python_Client_Library)
- Python module `inputs`

Install dependencies:
```bash
pip install inputs
```

### Clone and Setup:
```bash
git clone https://github.com/cakh/ur5_xbox_control.git
cd ur5_xbox_control
```

## 🚀 Usage

1. Connect your Xbox controller to your computer.
2. Ensure your UR5 robot (or URSim) is running and accessible via the configured IP (`localhost` by default).
3. Start the controller script:
```bash
sudo python src/main.py
```

## 🎮 Xbox Controller Mapping

The default mapping is:

| Control | Action |
|---|---|
| Left Joystick | X, Y axis motion |
| Left Trigger | Adjust Z axis motion (downwards) |
| Right Trigger | Adjust Z axis motion (upwards) |
| Button A | Custom action (modifiable) |

You can adjust these mappings in `xbox_control.py`.

## ⚠️ Safety Notice
Always test robot movements at low speeds initially to ensure safety and correctness.

## 📜 License
This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

