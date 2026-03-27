import cv2
import os
import pyrealsense2 as rs
import numpy as np

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    img_dir = os.path.join(base_dir, 'images')
    os.makedirs(img_dir, exist_ok=True)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)
    
    cont_img = 1
    print(f"Saving images to: {img_dir}")
    print("Controls: [SPACE] to save, [ESC] to exit.")

    try:
        while True:
            frames = pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            frame = np.asanyarray(color_frame.get_data())
            cv2.imshow('Live Video - Calibration', frame)

            key = cv2.waitKey(1) & 0xFF

            if key == 32:
                img_name = f"calib_{cont_img:03d}.jpg"
                img_path = os.path.join(img_dir, img_name)
                cv2.imwrite(img_path, frame)
                print(f"[{cont_img}] Image saved: {img_name}")
                cont_img += 1

            elif key == 27:
                print("Exiting data acquisition...")
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()