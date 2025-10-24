# realsense_view.py
import cv2
import numpy as np
import pyrealsense2 as rs

def main():
    pipe = rs.pipeline()
    cfg = rs.config()

    # Enable streams (adjust resolutions if you like)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    profile = pipe.start(cfg)

    # Align depth to color so both frames line up
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()  # to make depth human-viewable

    try:
        while True:
            frames = pipe.wait_for_frames(timeout_ms=5000)
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue  # try next frame

            # Convert to NumPy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            # (Optional) stack side-by-side for a single preview window
            preview = np.hstack((color_image, depth_color))

            cv2.imshow("RealSense (color | depth)", preview)

            # Quit on Esc
            if cv2.waitKey(1) & 0xFF == 27:
                break

    except Exception as e:
        print("Stream error:", e)
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
