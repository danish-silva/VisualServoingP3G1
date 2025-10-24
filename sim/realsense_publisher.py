import socket, json
import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
from time import time

# ====== Config ======
HOST = "127.0.0.1"
PORT = 5566                 # the sim/robot will listen on this
CB_COLS, CB_ROWS = 6, 5     # inner corners (cols, rows)
LAMBDA = 0.5
DESIRED_HALF_W, DESIRED_HALF_H = 80, 60
SUBPIX_WIN = (5, 5)
SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
DEPTH_MEDIAN_KSIZE = 3
MIN_VALID_DEPTH_M = 0.15
SMOOTH_WINDOW = 5

def ibvs_interaction_matrix(uv, Z, fx, fy, cx, cy):
    N = uv.shape[1]
    L = np.zeros((2 * N, 6), dtype=np.float64)
    for i in range(N):
        u, v, z = float(uv[0, i]), float(uv[1, i]), float(Z[i])
        if z <= 0: z = 1e9
        x = (u - cx) / fx
        y = (v - cy) / fy
        L_i = np.array([
            [-1.0/z,     0.0,  x/z,  x*y, -(1+x*x),    y],
            [    0.0, -1.0/z,  y/z, 1+y*y,   -x*y,   -x],
        ])
        L[2*i:2*i+2, :] = L_i
    return L

def get_depth_m(depth, u, v, depth_scale, ksize=3):
    h, w = depth.shape
    ui, vi = int(round(u)), int(round(v))
    u0 = max(0, ui - ksize//2); u1 = min(w, ui + ksize//2 + 1)
    v0 = max(0, vi - ksize//2); v1 = min(h, vi + ksize//2 + 1)
    patch = depth[v0:v1, u0:u1]
    if patch.size == 0: return 0.0
    return float(np.median(patch) * depth_scale)

def pick_four_corners(corners, cols, rows):
    tl = 0; tr = cols-1; bl = (rows-1)*cols; br = rows*cols - 1
    return corners[[tl, tr, bl, br]].reshape(-1, 2)

def draw_cross(img, p, size=6, thick=2, color=(0,255,0)):
    u, v = int(round(p[0])), int(round(p[1]))
    cv2.line(img, (u-size, v), (u+size, v), color, thick, cv2.LINE_AA)
    cv2.line(img, (u, v-size), (u, v+size), color, thick, cv2.LINE_AA)

# ----- TCP connect to subscriber -----
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
print(f"[INFO] Connecting to {HOST}:{PORT} ...")
sock.connect((HOST, PORT))
print("[INFO] Connected.")

# ----- RealSense setup -----
pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipe.start(cfg)
align = rs.align(rs.stream.color)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
print(f"[INFO] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} depth_scale={depth_scale:.6f}")

u_des = np.array([
    [cx - DESIRED_HALF_W, cy - DESIRED_HALF_H],
    [cx + DESIRED_HALF_W, cy - DESIRED_HALF_H],
    [cx - DESIRED_HALF_W, cy + DESIRED_HALF_H],
    [cx + DESIRED_HALF_W, cy + DESIRED_HALF_H],
], dtype=np.float64)             # 4x2
u_des_vec = u_des.reshape(-1, 1) # 8x1
vc_hist = deque(maxlen=SMOOTH_WINDOW)

try:
    while True:
        frames = pipe.wait_for_frames()
        frames = align.process(frames)
        cf, df = frames.get_color_frame(), frames.get_depth_frame()
        if not cf or not df: continue

        color = np.asanyarray(cf.get_data())
        depth = np.asanyarray(df.get_data())
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray, (CB_COLS, CB_ROWS),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        info = "Checkerboard: NOT FOUND"
        if found and corners is not None:
            corners = cv2.cornerSubPix(gray, corners, SUBPIX_WIN, (-1,-1), SUBPIX_CRIT).reshape(-1, 2)
            pts4 = pick_four_corners(corners, CB_COLS, CB_ROWS)
            Z = np.array([get_depth_m(depth, u, v, depth_scale, DEPTH_MEDIAN_KSIZE) for (u, v) in pts4], dtype=np.float64)
            valid = (Z > MIN_VALID_DEPTH_M) & np.isfinite(Z)

            if valid.sum() >= 3:
                uv = pts4.T
                L  = ibvs_interaction_matrix(uv, Z, fx, fy, cx, cy)
                e  = uv.reshape(-1,1) - u_des_vec
                vc = -LAMBDA * np.linalg.pinv(L) @ e  # (6x1)
                vc_hist.append(vc)
                vc_smooth = np.mean(np.dstack(vc_hist), axis=2).reshape(6,1)

                # send as NDJSON line
                payload = {
                    "t": time(),
                    "vc": [float(x) for x in vc.ravel()],
                    "err_px": float(np.linalg.norm(e)),
                    "valid": int(valid.sum())
                }
                sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))

                info = f"FOUND | ||e||={payload['err_px']:.1f}px | v_c={np.round(vc_smooth.ravel(),3)}"
                for p in pts4: draw_cross(color, p, color=(0,255,0))
                for p in u_des: draw_cross(color, p, color=(0,165,255))
                for i in range(4):
                    cpt = (int(round(pts4[i,0])), int(round(pts4[i,1])))
                    dpt = (int(round(u_des[i,0])), int(round(u_des[i,1])))
                    cv2.line(color, cpt, dpt, (255,0,0), 1, cv2.LINE_AA)
            else:
                info = "DEPTH INVALID (move board into range)"

        cv2.putText(color, info, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (15,235,15), 2, cv2.LINE_AA)
        cv2.imshow("IBVS: RealSense publisher (RGB)", color)
        if (cv2.waitKey(1) & 0xFF) == 27: break

except KeyboardInterrupt:
    pass
finally:
    try: sock.close()
    except: pass
    pipe.stop()
    cv2.destroyAllWindows()
