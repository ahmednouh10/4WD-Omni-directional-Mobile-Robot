# 4WD-Omni-directional-Mobile-Robot
# Mecanum Mobile Robot (ROS 2 + ESP32 + Arduino UNO + Gazebo/RViz)

This project implements a full workflow for a 4‑mecanum wheeled robot:

- Teleoperation in **ROS 2** (`geometry_msgs/Twist`)
- **Mecanum inverse kinematics** (Twist → 4 wheel speeds)
- Real robot control over Wi‑Fi: **ROS 2 → TCP (JSON) → ESP32 → UART → Arduino UNO → Motor Driver → Motors**
- Gazebo Classic simulation + RViz2 visualization
- Optional MPU‑6500 on ESP32 for roll/pitch/yaw visualization

> The system is **open‑loop** (no wheel encoders). In parallel mode, Gazebo and the real robot follow the *same teleop commands*; perfect real-world position matching is not expected without sensors.

---

## Hardware Schematic


![Robot schematic](docs/schematic.png)

---

## Architecture

### Real Robot Control Path
1. A teleop node publishes `Twist` (commonly on `/cmd_vel`).
2. A ROS 2 bridge node subscribes to the teleop Twist topic and computes mecanum wheel speeds.
3. The bridge sends newline‑delimited JSON frames to the ESP32 (TCP port 5000).
4. ESP32 forwards wheel commands to Arduino UNO via UART.
5. Arduino UNO drives the motor driver.

### Simulation Path
- Gazebo listens to `/cmd_vel` (or a remapped teleop topic) and moves the simulated base.
- RViz visualizes the robot model using TF and optional joint states.

---

## Data Protocol (ROS → ESP32)

The bridge sends one JSON object per line:

```json
{"u":"pwm","w":[p1,p2,p3,p4],"t":1700000000.123}
