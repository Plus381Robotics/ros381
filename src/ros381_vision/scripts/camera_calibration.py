import cv2
import numpy as np

# ===================== USER SETTINGS =====================
CAMERA_INDEX = 0

SQUARES_X = 8          # chessboard squares in X
SQUARES_Y = 11          # chessboard squares in Y
SQUARE_LENGTH = 0.02   # meters (or any unit, be consistent)
MARKER_LENGTH = 0.015   # same unit as square length

DICT_ID = cv2.aruco.DICT_4X4_50
MIN_CORNERS = 4        # minimum charuco corners per frame
# =========================================================

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y),
    SQUARE_LENGTH,
    MARKER_LENGTH,
    aruco_dict
)

cap = cv2.VideoCapture(CAMERA_INDEX)

all_charuco_corners = []
all_charuco_ids = []
image_size = None

print("Press 'c' to capture frame, 'q' to calibrate & quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        ok, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )

        if ok and ch_corners is not None and len(ch_corners) >= MIN_CORNERS:
            cv2.aruco.drawDetectedCornersCharuco(frame, ch_corners, ch_ids)

    cv2.imshow("ChArUco Calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        if ids is not None:
            ok, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board
            )
            print("markers:", 0 if ids is None else len(ids),
            "charuco:", 0 if ch_corners is None else len(ch_corners))
            if ok and ch_corners is not None and len(ch_corners) >= MIN_CORNERS:
                all_charuco_corners.append(ch_corners)
                all_charuco_ids.append(ch_ids)
                print(f"Captured frame {len(all_charuco_corners)}")
            else:
                print("Not enough ChArUco corners")
        else:
            print("No markers detected")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(all_charuco_corners) < 5:
    raise RuntimeError("Not enough valid frames for calibration")

ret, K, D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_charuco_corners,
    all_charuco_ids,
    board,
    image_size,
    None,
    None
)

print("\n=== CALIBRATION RESULT ===")
print("Reprojection error:", ret)
print("Camera matrix:\n", K)
print("Distortion coeffs:\n", D.ravel())

np.savez(
    "charuco_calibration.npz",
    camera_matrix=K,
    dist_coeffs=D,
    reprojection_error=ret
)

print("\nSaved to charuco_calibration.npz")
