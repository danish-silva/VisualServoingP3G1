# UR3 IBVS RealSense Visual Servoing - Project 3 GROUP 1

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
- **ROS bridge** running on the UR3 Pi

---

## ⚙️ How It Works

### 🟢 The RealSense Publisher
- Detects a **6×5 checkerboard pattern** in the RGB image.  
- Uses the depth map to estimate each corner’s **3D distance (Z)**.  
- Computes the **image-based error** between current corner locations and desired image coordinates (`u_des`).  
- Calculates the **camera-space velocity twist**  
  \[
  v_c = -λ L^+ e
  \]  
  using the IBVS interaction matrix **L**.  
- Sends the 6D velocity vector  
  \[
  v_c = [v_x, v_y, v_z, ω_x, ω_y, ω_z]
  \]  
  to the robot controller via TCP (`127.0.0.1:5566`).

---

### 🔵 The UR3 Controller
- Connects to **rosbridge** (`192.168.27.1:9090`) to interface with the robot.  
- Subscribes to `/ur/joint_states` or `/joint_states` for the current joint positions.  
- Receives `v_c`, converts it to tool velocity (`v_e = Ad⁻¹ v_c`), and computes joint rates `q̇ = J⁺ v_e`.  
- Integrates small time steps (≈15–20 Hz) to create **micro-step trajectories**, which are published to  
  `/ur/scaled_pos_joint_traj_controller/command`.

---

## 🚀 Running the System

### 1️⃣ Start the ROS & UR3 Environment

