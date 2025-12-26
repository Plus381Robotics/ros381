import numpy as np

data = np.load("../params/charuco_calibration.npz")

K = data["camera_matrix"]
D = data["dist_coeffs"]
err = data["reprojection_error"]

print("=== Calibration data ===")
print("Reprojection error:")
print(err)

print("\nCamera matrix (K):")
print(K)

print("\nDistortion coefficients (D):")
print(D)