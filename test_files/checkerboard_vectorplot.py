import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque

# ===== User parameters =====
# Checkerboard (inner corners!)
CB_COLS, CB_ROWS = 7, 6           # e.g., a 7x6 board has 6x5 inner corners -> set (cols=6, rows=5)
SUBPIX_WIN = (5, 5)
SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)

# IBVS control
LAMBDA = 0.5                      # gain
DEPTH_MEDIAN_KSIZE = 3            # median filter window around each point (pixels)
MIN_VALID_DEPTH_M = 0.15          # ignore too-near invalid readings

# Desired pixel layout (relative to principal point)
DESIRED_HALF_W = 80               # px
DESIRED_HALF_H = 60               # px

# For smoothing noisy depth/points (optional)
SMOOTH_WINDOW = 5                 # average v_c over last N frames for display

# ===== Helpers =====
def ibvs_interaction_matrix(uv, Z, fx, fy, cx, cy):
    """
    uv: 2xN pixel coords (u,v) in pixels
    Z:  N depths (m)
    Returns L (2N x 6) for point features using normalized coords.
    """
    N = uv.shape[1]
    L = np.zeros((2 * N, 6), dtype=np.float64)
    for i in range(N):
        u, v, z = float(uv[0, i]), float(uv[1, i]), float(Z[i])
        if z <= 0:
            z = 1e9  # avoid div by zero; this will zero the translational terms
        # Normalize by intrinsics
        x = (u - cx) / fx
        y = (v - cy) / fy
        L_i = np.array([
            [-1.0 / z,       0.0,      x / z,   x * y, -(1 + x * x),       y],
            [     0.0, -1.0 / z,      y / z, 1 + y * y,        -x * y,     -x],
        ])
        L[2*i:2*i+2, :] = L_i
    return L

def get_depth_m(depth_image, u, v, depth_scale, ksize=3):
    """
    Fetch depth at pixel (u,v) in metres with a small median filter window.
    """
    h, w = depth_image.shape
    u_i = int(round(u)); v_i = int(round(v))
    u0 = max(0, u_i - ksize//2); u1 = min(w, u_i + ksize//2 + 1)
    v0 = max(0, v_i - ksize//2); v1 = min(h, v_i + ksize//2 + 1)
    patch = depth_image[v0:v1, u0:u1]
    if patch.size == 0:
        return 0.0
    d_mm = np.median(patch.astype(np.float32))     # raw units
    d_m = float(d_mm * depth_scale)                # to metres
    return d_m if d_m > 0 else 0.0

def pick_four_corners(corners, cols, rows):
    """
    OpenCV returns corners in row-major order (left->right, top->bottom).
    Pick TL, TR, BL, BR.
    """
    tl = 0
    tr = cols - 1
    bl = (rows - 1) * cols
    br = rows * cols - 1
    idx = [tl, tr, bl, br]
    pts = corners[idx].reshape(-1, 2)  # 4x2
    return pts

def draw_cross(img, p, size=6, thick=2, color=(0,255,0)):
    u, v = int(round(p[0])), int(round(p[1]))
    cv2.line(img, (u-size, v), (u+size, v), color, thick, cv2.LINE_AA)
    cv2.line(img, (u, v-size), (u, v+size), color, thick, cv2.LINE_AA)

# ===== RealSense setup =====
pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipe.start(cfg)
align = rs.align(rs.stream.color)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()  # metres per unit
print(f"[INFO] Depth scale: {depth_scale:.6f} m/unit")

# Camera intrinsics (color)
color_stream = profile.get_stream(rs.stream.color)  # rs.video_stream_profile
intr = color_stream.as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=np.float64)
print(f"[INFO] Intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")

# Desired 4 target pixels around principal point
u_des = np.array([
    [cx - DESIRED_HALF_W, cy - DESIRED_HALF_H],  # TL
    [cx + DESIRED_HALF_W, cy - DESIRED_HALF_H],  # TR
    [cx - DESIRED_HALF_W, cy + DESIRED_HALF_H],  # BL
    [cx + DESIRED_HALF_W, cy + DESIRED_HALF_H],  # BR
], dtype=np.float64)  # 4x2
u_des_vec = u_des.reshape(-1, 1)                 # 8x1

# Smoother for display
vc_hist = deque(maxlen=SMOOTH_WINDOW)

print("[INFO] Running. Move the checkerboard; script prints v_c. Press ESC to quit.")
try:
    while True:
        frames = pipe.wait_for_frames()
        frames = align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, (CB_COLS, CB_ROWS),
                                                   flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        vc = None
        info_text = "Checkerboard: NOT FOUND"

        if found and corners is not None:
            # Refine to subpixel
            corners = cv2.cornerSubPix(gray, corners, SUBPIX_WIN, (-1, -1), SUBPIX_CRIT)
            corners = corners.reshape(-1, 2)  # (CB_COLS*CB_ROWS, 2)

            # Take 4 stable corners
            pts4 = pick_four_corners(corners, CB_COLS, CB_ROWS)  # 4x2 (u,v)

            # Depths for each selected corner
            Z = np.array([get_depth_m(depth, u, v, depth_scale, DEPTH_MEDIAN_KSIZE) for (u, v) in pts4], dtype=np.float64)

            # Guard: ensure depths look valid
            valid = (Z > MIN_VALID_DEPTH_M) & np.isfinite(Z)
            if valid.sum() >= 3:  # need at least 3 points for a stable L+
                # Stack uv as 2xN
                uv = pts4.T  # 2x4

                # Build interaction matrix
                L = ibvs_interaction_matrix(uv, Z, fx, fy, cx, cy)  # (8x6)

                # Error in pixels
                uv_vec = uv.reshape(-1, 1)   # (8x1)
                e = uv_vec - u_des_vec       # (8x1)

                # Solve for camera twist
                try:
                    vc = -LAMBDA * np.linalg.pinv(L) @ e  # (6x1)
                    vc = vc.reshape(6, 1)
                    vc_hist.append(vc)
                    vc_disp = np.mean(np.dstack(vc_hist), axis=2) if len(vc_hist) > 0 else vc
                    info_text = ("Checkerboard: FOUND | "
                                 f"||e||={float(np.linalg.norm(e)):.1f}px | "
                                 f"v_c: [{vc_disp[0,0]:+.3f}, {vc_disp[1,0]:+.3f}, {vc_disp[2,0]:+.3f}, "
                                 f"{vc_disp[3,0]:+.3f}, {vc_disp[4,0]:+.3f}, {vc_disp[5,0]:+.3f}]")
                    # Print raw (unsmoothed) v_c so you can feed it to your controller
                    print("v_c (camera frame) =", vc.ravel())
                except np.linalg.LinAlgError:
                    info_text = "Numerics issue: interaction matrix ill-conditioned"

                # Draw current and desired points
                for p in pts4:
                    draw_cross(color, p, color=(0, 255, 0))
                for p in u_des:
                    draw_cross(color, p, color=(0, 165, 255))  # orange target

                # Draw lines current->desired
                for i in range(4):
                    cpt = (int(round(pts4[i,0])), int(round(pts4[i,1])))
                    dpt = (int(round(u_des[i,0])), int(round(u_des[i,1])))
                    cv2.line(color, cpt, dpt, (255, 0, 0), 1, cv2.LINE_AA)

            else:
                info_text = "Checkerboard: DEPTH INVALID (move board into view/range)"

        # HUD text
        cv2.putText(color, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 230, 20), 2, cv2.LINE_AA)
        cv2.putText(color, "Green=corners | Orange=targets | Blue=line to target", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)

        cv2.imshow("IBVS (RealSense RGB + overlay)", color)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

except Exception as e:
    print("[ERROR]", e)
finally:
    pipe.stop()
    cv2.destroyAllWindows()
