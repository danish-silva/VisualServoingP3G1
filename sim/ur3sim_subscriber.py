import socket, json, threading, math
import numpy as np
from time import time

from spatialmath import SE3
from spatialmath.base import tr2adjoint
from roboticstoolbox import models
from machinevisiontoolbox import CentralCamera
from scipy import linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# =========================
# Config
# =========================
HOST = "127.0.0.1"
PORT = 5566              # must match your vision publisher

FPS = 25.0               # sim rate
DT  = 1.0 / FPS
MAX_QD = np.deg2rad(50)  # joint-rate limit (rad/s)
STALE_TIMEOUT = 0.3      # if no new vc in this many seconds: stop

# Eye-in-hand transform: tool->camera (update later to your real mount)
# Camera ~10 cm ahead of tool, optical axis forward (flip about Y)
T_ce = SE3.Trans(0, 0, 0.10) * SE3.Ry(math.pi)
Ad_ce = tr2adjoint(T_ce.A)  # maps tool twist -> camera twist; v_c = Ad_ce v_e

# Camera intrinsics (like your lab)
cam = CentralCamera(
    f=0.08,           # focal length (m)
    rho=10e-5,        # pixel size (m/pixel)
    imagesize=[1024, 1024],
    pp=[512, 512],
    name="UR3camera"
)

# Optional: a few fixed points in space to visualise projection
P_world = np.array([
    [0.8,  0.8,  0.8,  0.8],
    [-0.25, 0.25, 0.25, -0.25],
    [0.35, 0.35, 0.15, 0.15],
])

# =========================
# TCP listener (NDJSON lines with {"vc":[...], ...})
# =========================
latest_vc = np.zeros((6,1))
last_rx_time = 0.0

def tcp_listener():
    global latest_vc, last_rx_time
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[SIM] Listening for v_c on {HOST}:{PORT}")
    conn, addr = srv.accept()
    print(f"[SIM] Vision connected from {addr}")
    with conn:
        buf = b""
        while True:
            data = conn.recv(4096)
            if not data:
                print("[SIM] Vision disconnected")
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                    vc = np.array(msg.get("vc", [0,0,0,0,0,0]), dtype=float).reshape(6,1)
                    latest_vc = vc
                    last_rx_time = time()
                except Exception as e:
                    print("[SIM] Parse error:", e)

# =========================
# Camera redraw helper (same trick as your lab file)
# =========================
def cam_redraw(cam_obj, ax):
    for artist in ax.get_children():
        if isinstance(artist, Poly3DCollection):
            artist.remove()
    cam_obj.plot(scale=0.25, solid=True, alpha=0.8, ax=ax)

# =========================
# Main
# =========================
def main():
    # UR3 model and initial pose
    r = models.DH.UR3()
    q = np.array([0, -math.pi/2,  math.pi/2, 0,  math.pi/2, 0.0], dtype=float)

    # Plot robot + camera in 3D
    workspace = [-2, 2, -2, 2, 0, 2]
    fig = r.plot(q, limits=workspace)
    fig.ax.set_box_aspect([workspace[i+1] - workspace[i] for i in range(0, len(workspace), 2)])

    # Mount camera on tool
    cam.pose = r.fkine(q) * T_ce
    cam.plot(scale=0.25, solid=True, alpha=0.8, ax=fig.ax)

    # Plot some 3D markers
    try:
        from spatialmath.base import plot_sphere
        [plot_sphere(radius=0.03, centre=P_world[:, i], color='b') for i in range(P_world.shape[1])]
    except Exception:
        pass

    # Start TCP listener
    threading = __import__("threading")
    threading.Thread(target=tcp_listener, daemon=True).start()

    print("[SIM] Running. Start your RealSense publisher and move the checkerboard.")
    print("      The simulator will follow using the received v_c values.")

    # Basic history (optional)
    k = 0
    try:
        while plt.fignum_exists(fig.fig.number):
            # Grab latest commanded camera twist
            if time() - last_rx_time > STALE_TIMEOUT:
                v_c = np.zeros((6,1))
            else:
                v_c = latest_vc

            # Map camera twist -> tool twist: v_e = Ad_ce^{-1} v_c
            v_e = np.linalg.solve(Ad_ce, v_c)

            # Joint rates from body Jacobian at the tool
            J = r.jacobe(q)          # 6x6
            qd = linalg.pinv(J) @ v_e
            qd = np.clip(qd.ravel(), -MAX_QD, MAX_QD)

            # Integrate
            q = (q + qd * DT).astype(float)
            r.q = q

            # Update camera pose (mounted to tool)
            cam.pose = r.fkine(q) * T_ce

            # Step viz
            fig.step(DT)
            cam_redraw(cam, fig.ax)

            # (Optional) show current projection of reference points
            try:
                cam.plot_point(P_world, "o", pose=cam.pose, color="magenta")
            except Exception:
                pass

            k += 1
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
