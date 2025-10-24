# UR3 IBVS controller for the physical robot
# - Subscribes to joint states from rosbridge
# - Receives v_c from your RealSense publisher over TCP
# - Sends incremental JointTrajectory commands to scaled_pos_joint_traj_controller
# - Keeps your /onrobot/* services unchanged (optional to call manually)
 
import time, socket, json, math, threading
import numpy as np
import roslibpy
import roboticstoolbox as rtb
from roboticstoolbox import models
from spatialmath import SE3
from spatialmath.base import tr2adjoint
 
# ----------------------------
# CONFIG
# ----------------------------
ROS_HOST = '192.168.27.1'   # ROS bridge (Pi)
ROS_PORT = 9090
VC_HOST = '127.0.0.1'       # Vision publisher (RealSense IBVS)
VC_PORT = 5566
 
CTRL_RATE_HZ = 5.0
DT = 1.0 / CTRL_RATE_HZ
STALE_TIMEOUT = 0.4
MAX_QD = np.deg2rad(40.0)
VC_SCALE = 3.0
 
# Hand-eye: tool->camera (update to your mount)
T_ce = SE3.Trans(0, 0, 0.10) * SE3.Ry(math.pi)
Ad_ce = tr2adjoint(T_ce.A)
 
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
 
# ----------------------------
# GLOBAL STATE
# ----------------------------
current_pos = None
last_vc = np.zeros((6,1))
last_vc_time = 0.0
 
# ----------------------------
# ROS HELPERS
# ----------------------------
def call_service(client, name):
    srv = roslibpy.Service(client, name, 'std_srvs/Trigger')
    req = roslibpy.ServiceRequest({})
    print(f"[ROS] Calling {name}")
    res = srv.call(req)
    print(f"[ROS] {res}")
    return res
 
def joint_state_cb(msg):
    global current_pos
    pos = msg.get('position', [])
    if len(pos) >= 6:
        current_pos = list(pos[:6])
 
def list_topics(client):
    """Return list of topics via rosapi (if available)."""
    try:
        srv = roslibpy.Service(client, '/rosapi/topics', 'rosapi/Topics')
        res = srv.call(roslibpy.ServiceRequest({}))
        return res.get('topics', [])
    except Exception as e:
        print("[ROS] rosapi unavailable:", e)
        return []
 
def publish_joint_positions(client, joints, duration):
    secs = int(duration)
    nsecs = int((duration - secs) * 1e9)
    traj = {
        'joint_names': JOINT_NAMES,
        'points': [{
            'positions': joints,
            'time_from_start': {'secs': secs, 'nsecs': nsecs}
        }]
    }
    topic = roslibpy.Topic(
        client,
        '/ur/scaled_pos_joint_traj_controller/command',
        'trajectory_msgs/JointTrajectory'
    )
    topic.advertise()
    topic.publish(roslibpy.Message(traj))
    topic.unadvertise()
 
# ----------------------------
# VC READER
# ----------------------------
def vc_reader():
    global last_vc, last_vc_time
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    retries = 0
    while True:
        try:
            print(f"[VC] Connecting to {VC_HOST}:{VC_PORT} ...")
            s.connect((VC_HOST, VC_PORT))
            print("[VC] Connected.")
            break
        except Exception as e:
            wait = min(5, 0.5 * (2 ** retries))
            print(f"[VC] {e}. Retrying in {wait:.1f}s ...")
            time.sleep(wait)
            retries += 1
    
    # Once connected to the publisher, continue reading
    buf = b""
    try:
        while True:
            data = s.recv(4096)
            if not data:
                print("[VC] Disconnected.")
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                    vc = np.array(msg.get("vc", [0,0,0,0,0,0]), dtype=float).reshape(6,1)
                    last_vc = vc
                    last_vc_time = time.time()
                except Exception as e:
                    print("[VC] Parse error:", e)
    finally:
        try: s.close()
        except: pass
 
# ----------------------------
# MAIN
# ----------------------------
def main():
    global current_pos
    threading.Thread(target=vc_reader, daemon=True).start()
 
    print(f"[ROS] Connecting to rosbridge at {ROS_HOST}:{ROS_PORT} ...")
    client = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
    client.run()
    print("[ROS] Connected.")
 
    # List available topics
    topics = list_topics(client)
    if topics:
        print("[ROS] Topics on bridge:")
        for t in topics:
            if 'joint_state' in t:
                print("   ", t)
 
    # Subscribe to /ur/joint_states or /joint_states
    candidates = ['/ur/joint_states', '/joint_states']
    listeners = []
    for t in candidates:
        if not topics or t in topics:
            print(f"[ROS] Subscribing to {t}")
            l = roslibpy.Topic(client, t, 'sensor_msgs/JointState')
            l.subscribe(joint_state_cb)
            listeners.append(l)
 
    # Wait up to 20s for first joint state
    print("[ROS] Waiting for joint states ...")
    t0 = time.time()
    while current_pos is None and (time.time() - t0) < 20.0:
        time.sleep(0.05)
    if current_pos is None:
        raise RuntimeError("No joint state received from /ur/joint_states or /joint_states")
    print(f"[ROS] Initial q: {np.round(current_pos,3)}")
 
    # UR3 model
    robot = models.DH.UR3() 
 
    print("[CTRL] Ready. Streaming micro-steps. Ctrl+C to stop.")
    try:
        while client.is_connected:
            q = np.array(current_pos[:6], dtype=float)
 
            # Get latest v_c (zero if stale)
            if time.time() - last_vc_time > STALE_TIMEOUT:
                vc = np.zeros((6,1))
            else:
                vc = last_vc * VC_SCALE
 
            # Map to tool twist and joint velocities
            v_e = np.linalg.solve(Ad_ce, vc)
            J = robot.jacobe(q)
            qd = np.linalg.pinv(J) @ v_e
            qd = np.clip(qd.ravel(), -MAX_QD, MAX_QD)
 
            q_cmd = (q + qd * DT).tolist()
            publish_joint_positions(client, q_cmd, duration=DT)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        for l in listeners:
            try: l.unsubscribe()
            except: pass
        client.terminate()
        print("[ROS] Disconnected.")
 
# ----------------------------
if __name__ == '__main__':
    main()
