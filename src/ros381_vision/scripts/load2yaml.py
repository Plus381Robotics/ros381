import numpy as np
import yaml

# Load calibration
data = np.load("../params/charuco_calibration.npz")
K = data["camera_matrix"].flatten().tolist()
D = data["dist_coeffs"].flatten().tolist()

# Minimal YAML dictionary
minimal_yaml = {
    "camera_matrix": K,
    "dist_coeffs": D
}

# Save to file
with open("camera_params.yaml", "w") as f:
    yaml.dump(minimal_yaml, f, default_flow_style=False)

print("Saved minimal camera_params.yaml:")
print(yaml.dump(minimal_yaml, default_flow_style=False))