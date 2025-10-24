# Visual Servoing - Project 3 GROUP 1

# 🤖 UR3 IBVS RealSense Visual Servoing

This project implements an **eye-in-hand Image-Based Visual Servoing (IBVS)** system using an **Intel RealSense** RGB-D camera and a **UR3 robotic arm** controlled via ROS (through `rosbridge`).  

The robot continuously tracks a **checkerboard target** by keeping it centred and scaled in the image plane. When the checkerboard moves, the UR3 automatically reacts to maintain alignment using a visual feedback loop.

---

## 📂 Project Structure

| File | Description |
|------|-------------|
| `realsense_publisher.py` | RealSense publisher — detects a checkerboard, computes the image error and visual velocity (`v_c`), and streams this data over TCP. |
| `ur3_ibvs_real_follow.py` | UR3 controller — connects to `rosbridge`, subscribes to joint states, receives `v_c`, computes joint velocities using the UR3 Jacobian, and commands micro-step trajectories in real time. |

---

## 🧰 System Requirements

- **Intel RealSense D4xx** or similar depth camera  
- **Universal Robots UR3** (connected to ROS machine)  
- **ROS bridge** running on the robot host:  
  ```bash
  roslaunch rosbridge_server rosbridge_websocket.launch
