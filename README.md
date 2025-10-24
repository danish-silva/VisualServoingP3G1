# UR3 IBVS RealSense Visual Servoing - Project 3 GROUP 1

This project implements an **eye-in-hand Image-Based Visual Servoing (IBVS)** system using an **Intel RealSense** RGB-D camera and a **UR3 robotic arm** controlled via ROS (through `rosbridge`).  

The robot continuously tracks a **checkerboard target** by keeping it centred and scaled in the image plane. When the checkerboard moves, the UR3 automatically reacts to maintain alignment using a visual feedback loop.

---

## 📂 Project Structure

| File | Description |
|------|-------------|
| `realsense_robotpublisher.py` | RealSense publisher — detects a checkerboard, computes the image error and visual velocity (`v_c`), and streams this data over TCP. |
| `ur3_subscriber.py` | UR3 controller — connects to `rosbridge`, subscribes to joint states, receives `v_c`, computes joint velocities using the UR3 Jacobian, and commands micro-step trajectories in real time. |

---

## 🧰 System Requirements

- **Intel RealSense D4xx** or similar depth camera  
- **Universal Robots UR3** (connected to ROS machine)  
- **ROS bridge** running on the UR3 Pi

---

## ⚙️ How It Works

### The RealSense Publisher
- Detects a **6×5 checkerboard pattern** in the RGB image.  
- Uses the depth map to estimate each corner’s **3D distance (Z)**.  
- Computes the **image-based error** between current corner locations and desired image coordinates (`u_des`).  
- Calculates the **camera-space velocity twist**  
  `[v_c = -λ L^+ e]`
  using the IBVS interaction matrix **L**.  
- Sends the 6D velocity vector  
  [`v_c = [v_x, v_y, v_z, ω_x, ω_y, ω_z]`]  
  to the robot controller via TCP (`127.0.0.1:5566`).

---

### The UR3 Controller
- Connects to **rosbridge** (`192.168.27.1:9090`) to interface with the robot.  
- Subscribes to `/ur/joint_states` or `/joint_states` for the current joint positions.  
- Receives `v_c`, converts it to tool velocity (`v_e = Ad⁻¹ v_c`), and computes joint rates `q̇ = J⁺ v_e`.  
- Integrates small time steps (≈15–20 Hz) to create **micro-step trajectories**, which are published to  
  `/ur/scaled_pos_joint_traj_controller/command`.

---

## 🚀 Running the System

### 1️⃣ Start the ROS & UR3 Environment
### 2️⃣ Run the RealSense Publisher

Plug in the RealSense camera and run (make sure to have the Windows RealSense SDK with python dependencies installed):
`python realsense_robotpublisher.py`

### 3️⃣ Run the UR3 Controller
In a second terminal run:
`python ur3_subscriber.py`

Expected output should look like this:
```bash
[ROS] Connected.
[ROS] Subscribing to /joint_states
[VC] Connecting to 127.0.0.1:5566 ...
[VC] Connected.
[CTRL] Streaming micro-steps. Ctrl+C to stop.
```

---

## 📸 Key Parameters (Publisher)

| Parameter | Meaning | Typical Range / Effect |
|------------|----------|------------------------|
| `LAMBDA` | IBVS gain λ (responsiveness) | 0.3–1.0 → higher = faster response |
| `DESIRED_HALF_W`, `DESIRED_HALF_H` | Desired half-width/height (px) of the checkerboard corners in the image | Larger = robot moves further back; smaller = moves closer |
| `SUBPIX_WIN` | Corner refinement window (px) | (5, 5) → good accuracy |
| `DEPTH_MEDIAN_KSIZE` | Depth smoothing kernel size | 3–5 → reduce depth noise |
| `MIN_VALID_DEPTH_M` | Minimum valid distance (m) | 0.15–0.25 |
| `SMOOTH_WINDOW` | Velocity smoothing window (frames) | 3–5 → smoother but slower |

---

## ⚙️ Key Parameters (Controller)

| Parameter | Meaning | Typical Value |
|------------|----------|---------------|
| `CTRL_RATE_HZ` | Update rate for control loop | 15–20 Hz |
| `MAX_QD` | Maximum joint speed (rad/s) | ~0.7–1.0 rad/s (40–60°/s) |
| `VC_SCALE` | Scale factor applied to `v_c` | 1.0–3.0 |
| `STALE_TIMEOUT` | Time without updates before pausing (s) | 0.3–0.5 |
| `T_ce` | Tool-to-camera transform | Adjust to your camera mount orientation |

Move the checkerboard — the UR3 will follow it, maintaining the target’s position and scale in the image.

