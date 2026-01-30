import numpy as np
import yaml
import sys
import cv2

# ---------------------- OpenCV YAML loader (supports !!opencv-matrix) ---------------------- #
def _opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    rows = int(mapping.get("rows", 0))
    cols = int(mapping.get("cols", 0))
    data = mapping.get("data", None)
    if data is None:
        return mapping
    mat = np.array(data, dtype=float).reshape(rows, cols)
    return mat

class OpenCVLoader(yaml.SafeLoader):
    pass

OpenCVLoader.add_constructor("tag:yaml.org,2002:opencv-matrix", _opencv_matrix_constructor)

def load_opencv_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    # OpenCV FileStorage often starts with "%YAML:1.0" which vanilla YAML parsers dislike.
    lines = [ln for ln in txt.splitlines() if not ln.startswith("%YAML")]
    cleaned = "\n".join(lines)
    return yaml.load(cleaned, Loader=OpenCVLoader)

def main():
    data = load_opencv_yaml('/workspaces/src/Kimera-VIO/params/S3EAlphaStereoXfeat/alpha.yaml')

    # Extract Matrices
    # Note: data[key] is already a numpy array due to the loader
    P_left = data['LEFT.P']
    P_right = data['RIGHT.P']
    
    # Standard Decomposition P = K_new * [R_rect | t_rect]
    # Ideally for rectified stereo:
    # P_left  = K_new * [I | 0]
    # P_right = K_new * [I | tx]
    
    print("=== LEFT P Analysis ===")
    K_new_left, R_new_left, t_new_left, _, _, _, _ = cv2.decomposeProjectionMatrix(P_left)
    t_new_left = (t_new_left / t_new_left[3])[:3].flatten()
    print(f"Computed t_left: {t_new_left}")
    
    print("\n=== RIGHT P Analysis ===")
    K_new_right, R_new_right, t_new_right, _, _, _, _ = cv2.decomposeProjectionMatrix(P_right)
    t_new_right = (t_new_right / t_new_right[3])[:3].flatten()
    print(f"Computed t_right: {t_new_right}")

    # Relative Translation
    rel_t = t_new_right - t_new_left
    dist = np.linalg.norm(rel_t)
    print(f"\nRelative Translation Vector: {rel_t}")
    print(f"Baseline Distance (from P): {dist:.5f} m")

    # Provided bf check
    bf_provided = float(data['Camera.bf'])
    fx_provided = float(data['Camera.fx'])
    baseline_provided = bf_provided / fx_provided
    print(f"\nProvided in YAML:")
    print(f"Camera.bf: {bf_provided}")
    print(f"Camera.fx: {fx_provided}")
    print(f"implied baseline (bf/fx): {baseline_provided:.5f} m")

    print(f"\nDiscrepancy: {abs(dist - baseline_provided):.5f} m")
    
    # Check if P matrices share intrinsics (K_new should be identical)
    print("\n=== Intrinsic Consistency CHECK ===")
    print("K from P_left (upper 3x3):\n", P_left[:3,:3])
    print("K from P_right (upper 3x3):\n", P_right[:3,:3])
    
if __name__ == "__main__":
    main()
