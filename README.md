# 🏭 Factory Bent Simulation with a BCR Robot Arm

Vision-guided pick and place on factory conveyor belts ("bents"), in **ROS 2 Jazzy**, **Gazebo Harmonic** and **OpenCV**.

🌐 **Project website:** https://huseyintozluyurt.github.io/Factory-Bent-Simulation-with-BCR-Robot-Arm/

![Dual circular factory with the BCR arm](docs/images/screenshot_2026-10-31_dual_circular_factory.png)

A 7-axis Black Coffee Robotics (BCR) arm watches moving products through three cameras. It picks each product
off one bent and places it on another bent, or drops it into a trash or recycling bin.

## Features

- **OpenCV vision** with three cameras: overhead detection and tracking, a side-camera cross-check, and a drop-zone check
- **Pick and place** with URDF-based IK, catching moving products, and six arm movement styles
- **Sorting** into trash and recycling bins
- **Product generator** with 100 realistic products and your own 3D models
- **Order GUI** (PyQt5) for orders, belt speed and simulation speed
- **Worlds** with circular and straight bents in every combination
- **Vision lab** for testing your own OpenCV or YOLO detectors

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and the BCR arm packages in `~/ros2_ws/src`.

```bash
sudo apt install python3-opencv python3-pyqt5 python3-gz-transport13 python3-gz-msgs10
```

## Build and run

```bash
cd ~/ros2_ws/src
git clone https://github.com/huseyinTozluyurt/Factory-Bent-Simulation-with-BCR-Robot-Arm.git factory_bent_project
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export GZ_VERSION=harmonic

cd src/factory_bent_project
./run_factory.sh
```

The order window opens first. Choose products and press **Start simulation with this order**. Gazebo, the arm,
the cameras and the controller then start. Press **Ctrl+C** to stop.

## Worlds

| Command | Bent 1 | Bent 2 |
|---|---|---|
| `./run_dual_circular.sh` | circular | circular |
| `./run_single_circular.sh` | circular | none |
| `./run_straight_conveyor.sh` | straight | none |
| `./run_exp_straight_to_circular.sh` | straight | circular |
| `./run_exp_circular_to_straight.sh` | circular | straight |
| `./run_exp_straight_to_straight.sh` | straight | straight |

## More

- [Project website](https://huseyintozluyurt.github.io/Factory-Bent-Simulation-with-BCR-Robot-Arm/): the full explanation with images
- [COMMANDS.md](COMMANDS.md): every command and option
- [docs/reports/](docs/reports/): PDF reports on how the project evolved
- [website/](website/): source of the website (React + Vite)

---

Developed by **Hüseyin Tozluyurt**
