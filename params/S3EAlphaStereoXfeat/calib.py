#!/usr/bin/env python3
"""
Compute rigid transform from RIGHT camera frame to IMU frame using an OpenCV-style YAML.

This script is designed for YAMLs like your uploaded file, which contains:
  - Tic: (comment says) "Transformation from left camera to imu"  -> T_imu_left
  - Camera.fx and Camera.bf (bf = fx * baseline)
  - Optional RIGHT.P / LEFT.P (rectified stereo projection matrices)

ASSUMPTION (common for rectified stereo):
  - Left and right camera frames are rectified with R = I between them
  - Right camera is translated by +baseline on the x-axis relative to left camera
  - Therefore, transform RIGHT -> LEFT is:
        T_left_right = [ I | +baseline ]
    and LEFT -> RIGHT is:
        T_right_left = [ I | -baseline ]

Then:
  T_imu_right = T_imu_left @ T_left_right

If your Tic is actually IMU->LEFT (opposite direction), use --tic_is_imu_to_left
to invert it first.
"""

import argparse
import sys
import yaml
import numpy as np
from typing import Any, Dict

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

def load_opencv_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    # OpenCV FileStorage often starts with "%YAML:1.0" which vanilla YAML parsers dislike.
    lines = [ln for ln in txt.splitlines() if not ln.startswith("%YAML")]
    cleaned = "\n".join(lines)
    return yaml.load(cleaned, Loader=OpenCVLoader)

# ---------------------- SE(3) helpers ---------------------- #
def invert_T(T: np.ndarray) -> np.ndarray:
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4, got {T.shape}")
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def pretty_T(T: np.ndarray) -> str:
    return np.array2string(T, formatter={"float_kind": lambda x: f"{x: .8f}"})

def quat_wxyz_from_R(Rm: np.ndarray):
    # Returns quaternion as (w, x, y, z)
    # Robust conversion
    tr = np.trace(Rm)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (Rm[2, 1] - Rm[1, 2]) / S
        y = (Rm[0, 2] - Rm[2, 0]) / S
        z = (Rm[1, 0] - Rm[0, 1]) / S
    else:
        if (Rm[0, 0] > Rm[1, 1]) and (Rm[0, 0] > Rm[2, 2]):
            S = np.sqrt(1.0 + Rm[0, 0] - Rm[1, 1] - Rm[2, 2]) * 2
            w = (Rm[2, 1] - Rm[1, 2]) / S
            x = 0.25 * S
            y = (Rm[0, 1] + Rm[1, 0]) / S
            z = (Rm[0, 2] + Rm[2, 0]) / S
        elif Rm[1, 1] > Rm[2, 2]:
            S = np.sqrt(1.0 + Rm[1, 1] - Rm[0, 0] - Rm[2, 2]) * 2
            w = (Rm[0, 2] - Rm[2, 0]) / S
            x = (Rm[0, 1] + Rm[1, 0]) / S
            y = 0.25 * S
            z = (Rm[1, 2] + Rm[2, 1]) / S
        else:
            S = np.sqrt(1.0 + Rm[2, 2] - Rm[0, 0] - Rm[1, 1]) * 2
            w = (Rm[1, 0] - Rm[0, 1]) / S
            x = (Rm[0, 2] + Rm[2, 0]) / S
            y = (Rm[1, 2] + Rm[2, 1]) / S
            z = 0.25 * S
    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    return q

# ---------------------- Core computation ---------------------- #
def baseline_from_fx_bf(fx: float, bf: float) -> float:
    if fx == 0:
        raise ValueError("Camera.fx is zero; cannot compute baseline from bf/fx.")
    return bf / fx



def make_T_left_right(baseline: float) -> np.ndarray:
    # Transform from RIGHT cam coords to LEFT cam coords (RIGHT -> LEFT)
    T = np.eye(4)
    T[0, 3] = baseline
    return T

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml", help="Path to OpenCV-style calibration YAML (contains Tic, Camera.fx, Camera.bf, etc.)")
    ap.add_argument("--tic_is_imu_to_left", action="store_true",
                    help="If set, interpret Tic as IMU->LEFT and invert it to get LEFT->IMU.")
    ap.add_argument("--output", default="", help="Optional output YAML path to save T_imu_right.")
    args = ap.parse_args()

    cfg = load_opencv_yaml(args.yaml)

    if "Tic" not in cfg:
        print("ERROR: YAML missing 'Tic' key.", file=sys.stderr)
        sys.exit(2)

    Tic = np.array(cfg["Tic"], dtype=float)
    if Tic.shape != (4, 4):
        print(f"ERROR: Tic must be 4x4, got {Tic.shape}", file=sys.stderr)
        sys.exit(2)

    # Tic meaning per your file comment: LEFT -> IMU
    T_imu_left = invert_T(Tic) if args.tic_is_imu_to_left else Tic

    # Baseline estimation
    fx = float(cfg.get("Camera.fx", 0.0))
    bf = float(cfg.get("Camera.bf", 0.0))

    if fx == 0.0 or bf == 0.0:
        print("ERROR: Need Camera.fx and Camera.bf in YAML to compute stereo baseline.", file=sys.stderr)
        sys.exit(2)

    baseline_fx_bf = baseline_from_fx_bf(fx, bf)



    # Build RIGHT -> LEFT transform (rectified stereo)
    T_left_right = make_T_left_right(baseline_fx_bf)

    # Compose: RIGHT -> IMU
    T_imu_right = T_imu_left @ T_left_right

    # Print results
    print("\n=== Inputs ===")
    print(f"Camera.fx = {fx}")
    print(f"Camera.bf = {bf}")
    print(f"baseline (bf/fx) = {baseline_fx_bf:.8f}")


    print("\n=== Transform: LEFT -> IMU (T_imu_left) ===")
    print(pretty_T(T_imu_left))

    print("\n=== Transform: RIGHT -> LEFT (T_left_right) ===")
    print(pretty_T(T_left_right))

    print("\n=== Transform: RIGHT -> IMU (T_imu_right) ===")
    print(pretty_T(T_imu_right))

    q = quat_wxyz_from_R(T_imu_right[:3, :3])
    t = T_imu_right[:3, 3]
    print("\nAs (qw qx qy qz) and translation [tx ty tz]:")
    print(f"q = [{q[0]: .8f}, {q[1]: .8f}, {q[2]: .8f}, {q[3]: .8f}]")
    print(f"t = [{t[0]: .8f}, {t[1]: .8f}, {t[2]: .8f}]")

    # Save (optional)
    if args.output:
        out = {
            "T_imu_right": {
                "rows": 4,
                "cols": 4,
                "dt": "d",
                "data": T_imu_right.reshape(-1).tolist(),
            }
        }
        # Write in an OpenCV-like style (without the %YAML header)
        with open(args.output, "w", encoding="utf-8") as f:
            f.write("# Saved by compute_right_cam_to_imu.py\n")
            f.write("T_imu_right: !!opencv-matrix\n")
            f.write("  rows: 4\n  cols: 4\n  dt: d\n")
            f.write("  data: [")
            flat = out["T_imu_right"]["data"]
            for i, v in enumerate(flat):
                if i % 4 == 0:
                    f.write("\n    ")
                f.write(f"{v:.15f}")
                if i != len(flat) - 1:
                    f.write(", ")
            f.write("\n  ]\n")
        print(f"\nWrote: {args.output}")

if __name__ == "__main__":
    main()
