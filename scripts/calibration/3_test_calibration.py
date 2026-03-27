import cv2
import os
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 11
SQUARES_HORIZONTALLY = 11
SQUARE_LENGTH = 0.1
MARKER_LENGTH = 0.08

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    img_dir = os.path.join(base_dir, 'images')
    out_dir = os.path.join(base_dir, 'output')

    try:
        camera_matrix = np.load(os.path.join(out_dir, 'camera_matrix.npy'))
        dist_coeffs = np.load(os.path.join(out_dir, 'dist_coeffs.npy'))
    except FileNotFoundError:
        print("Calibration files not found. Run 2_calibrate.py first.")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    try:
        board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
        params = cv2.aruco.DetectorParameters()
    except AttributeError:
        board = cv2.aruco.CharucoBoard_create(SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH, dictionary)
        params = cv2.aruco.DetectorParameters_create()

    image_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith(".jpg")]
    image_files.sort()

    print("Controls: Press any key to view the next image. Press [ESC] to exit.")

    for image_file in image_files:
        image = cv2.imread(image_file)
        undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs)
        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

        if marker_ids is not None and len(marker_ids) > 0:
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board)

            if charuco_retval > 0:
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None)

                if retval:
                    cv2.drawFrameAxes(undistorted_image, camera_matrix, dist_coeffs, rvec, tvec, length=0.2, thickness=3)
        
        cv2.imshow("Pose Detection Validation", undistorted_image)
        key = cv2.waitKey(0) & 0xFF
        if key == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()