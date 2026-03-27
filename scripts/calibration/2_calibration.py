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
    os.makedirs(out_dir, exist_ok=True)

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    try:
        board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
        params = cv2.aruco.DetectorParameters()
    except AttributeError:
        board = cv2.aruco.CharucoBoard_create(SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH, dictionary)
        params = cv2.aruco.DetectorParameters_create()

    image_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith(".jpg")]
    image_files.sort()

    if not image_files:
        print(f"Error: No .jpg images found in {img_dir}")
        return

    all_charuco_corners = []
    all_charuco_ids = []
    image_shape = None

    print(f"Processing {len(image_files)} images...")

    for image_file in image_files:
        image = cv2.imread(image_file)
        if image_shape is None:
            image_shape = image.shape[:2]

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        
        if marker_ids is not None and len(marker_ids) > 0:
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board)
            
            if charuco_retval > 0:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                print(f"Found {len(charuco_corners)} corners in {os.path.basename(image_file)}")
            else:
                print(f"Failed to interpolate CharUco in {os.path.basename(image_file)}")
        else:
            print(f"No ArUco markers found in {os.path.basename(image_file)}")

    if not all_charuco_corners:
        print("Not enough valid data to calibrate.")
        return

    print("\nCalibrating camera...")
    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, image_shape, None, None)

    np.save(os.path.join(out_dir, 'camera_matrix.npy'), camera_matrix)
    np.save(os.path.join(out_dir, 'dist_coeffs.npy'), dist_coeffs)

    print("--- CALIBRATION SUCCESSFUL ---")
    print(f"RMS Error: {retval:.4f}")
    print(f"Saved matrices to: {out_dir}")

if __name__ == '__main__':
    main()