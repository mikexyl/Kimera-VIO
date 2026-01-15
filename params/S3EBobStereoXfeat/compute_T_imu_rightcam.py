#!/usr/bin/env python3
"""
Compute rectified stereo baseline and IMU<->camera transforms from an OpenCV YAML.

Key fixes vs your script:
- Baseline must use RECTIFIED fx (from LEFT.P[0,0] or RIGHT.P[0,0]), not Camera.fx.
- Do NOT invert T_left->right unless you truly want right->left.
- Make Tic direction explicit with a flag (cam->imu vs imu->cam). Your original code/comment disagreed.

Usage:
  python3 calib_compute.py --yaml bob.yaml --tic cam2imu
  python3 calib_compute.py --yaml bob.yaml --tic imu2cam
"""

import argparse
import numpy as np
import cv2


def load_calibration(yaml_file: str):
    fs = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Cannot open calibration file: {yaml_file}")

    def read_real(key):
        node = fs.getNode(key)
        return float(node.real()) if not node.empty() else None

    def read_mat(key):
        node = fs.getNode(key)
        return node.mat() if not node.empty() else None

    calib = {
        "Camera.fx": read_real("Camera.fx"),
        "Camera.fy": read_real("Camera.fy"),
        "Camera.bf": read_real("Camera.bf"),
        "Tic": read_mat("Tic"),
        "LEFT.R": read_mat("LEFT.R"),
        "LEFT.P": read_mat("LEFT.P"),
        "RIGHT.R": read_mat("RIGHT.R"),
        "RIGHT.P": read_mat("RIGHT.P"),
    }

    fs.release()

    # Basic presence checks
    for k in ["Tic", "LEFT.P", "RIGHT.P"]:
        if calib[k] is None:
            raise KeyError(f"Missing required key '{k}' in {yaml_file}")

    return calib


def as_4x4(T: np.ndarray) -> np.ndarray:
    """Ensure 4x4 homogeneous transform."""
    T = np.asarray(T, dtype=float)
    if T.shape == (4, 4):
        return T
    if T.shape == (3, 4):
        out = np.eye(4)
        out[:3, :4] = T
        return out
    if T.shape == (3, 3):
        out = np.eye(4)
        out[:3, :3] = T
        return out
    raise ValueError(f"Unsupported transform shape: {T.shape}")


def inv_T(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform."""
    T = as_4x4(T)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def baseline_from_rectified_P(P1: np.ndarray, P2: np.ndarray):
    """
    OpenCV stereoRectify-style projection matrices:
      P1 = [fx  0  cx  0
            0  fy cy  0
            0   0  1  0]

      P2 = [fx  0  cx  fx*Tx
            0  fy cy   0
            0   0  1   0]

    Baseline magnitude B is:
      B = -Tx  (usually)
      and Tx = P2(0,3)/fx

    So:
      B = -P2(0,3) / fx

    IMPORTANT: fx here is rectified fx (P1[0,0]), not raw Camera.fx.
    """
    P1 = np.asarray(P1, dtype=float)
    P2 = np.asarray(P2, dtype=float)
    fx_rect = float(P1[0, 0])
    Tx = float(P2[0, 3]) / fx_rect
    B = -Tx
    return B, fx_rect, Tx


def T_left_to_right_rectified(baseline: float) -> np.ndarray:
    """
    In rectified frame, rotations align -> R = I.
    With OpenCV convention, right camera is translated by [-B, 0, 0] in left frame.
    """
    T = np.eye(4)
    T[0, 3] = -float(baseline)
    return T


def pretty_transform(T: np.ndarray, name: str):
    T = as_4x4(T)
    R = T[:3, :3]
    t = T[:3, 3]
    print(f"\n{name}")
    print("=" * len(name))
    print("R:\n", R)
    print("t:", t)
    print("T:\n", T)


def print_yaml_opencv_matrix(T: np.ndarray, name: str):
    T = as_4x4(T)
    print(f"\n{name}: !!opencv-matrix")
    print("   rows: 4")
    print("   cols: 4")
    print("   dt: d")
    data_str = ", ".join(f"{v: .11f}" for v in T.flatten())
    print(f"   data: [{data_str}]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True, help="Path to calibration YAML (OpenCV FileStorage format).")
    ap.add_argument(
        "--tic",
        choices=["cam2imu", "imu2cam"],
        default="cam2imu",
        help="Interpretation of Tic in YAML. "
             "'cam2imu' means T_cam->imu. 'imu2cam' means T_imu->cam.",
    )
    ap.add_argument(
        "--output",
        choices=["imu2right", "right2imu", "both"],
        default="both",
        help="Which transform(s) to print in YAML format.",
    )
    args = ap.parse_args()

    calib = load_calibration(args.yaml)

    P1 = np.asarray(calib["LEFT.P"], dtype=float)
    P2 = np.asarray(calib["RIGHT.P"], dtype=float)
    Tic = as_4x4(calib["Tic"])

    # --- Rectified baseline and rectified fx ---
    B, fx_rect, Tx = baseline_from_rectified_P(P1, P2)

    print("\nRectified intrinsics from LEFT.P")
    print("===============================")
    print(f"fx_rect (P1[0,0]): {fx_rect:.6f} px")
    print(f"P2[0,3]: {P2[0,3]:.6f}")
    print(f"Tx = P2[0,3]/fx_rect: {Tx:.6f} m (often negative)")
    print(f"Baseline B = -Tx:      {B:.6f} m")

    bf_file = calib.get("Camera.bf")
    fx_file = calib.get("Camera.fx")

    print("\nSanity checks")
    print("=============")
    if bf_file is not None:
        print(f"Camera.bf (file):      {bf_file:.6f} (px*m)")
        print(f"B * fx_rect:           {(B * fx_rect):.6f} (px*m)")
        if fx_file is not None:
            print(f"Camera.fx (file):      {fx_file:.6f} px")
            print(f"B * Camera.fx:         {(B * fx_file):.6f} (px*m)")
            print("(If Camera.fx != fx_rect, it's normal when using rectified/cropped/resized images.)")
    else:
        print("Camera.bf not found in YAML; skipping bf comparison.")

    # --- Build rectified left->right transform (in rectified camera frame) ---
    T_LR = T_left_to_right_rectified(B)

    # --- Interpret Tic and construct T_imu_left ---
    # We want T_imu_left (IMU -> LeftCam) for chaining with T_LR (LeftCam -> RightCam).
    if args.tic == "cam2imu":
        # YAML gives T_left->imu, so invert to get T_imu->left
        T_left_imu = Tic
        T_imu_left = inv_T(T_left_imu)
        pretty_transform(T_left_imu, "T_leftcam_imu (from Tic, cam2imu)")
        pretty_transform(T_imu_left, "T_imu_leftcam (inverted)")
    else:
        # YAML gives T_imu->left directly
        T_imu_left = Tic
        pretty_transform(T_imu_left, "T_imu_leftcam (from Tic, imu2cam)")

    # --- Chain to get IMU -> RightCam (rectified right camera frame) ---
    # T_imu_right = T_imu_left * T_left_right
    T_imu_right = T_imu_left @ T_LR
    T_right_imu = inv_T(T_imu_right)

    pretty_transform(T_LR, "T_leftcam_rightcam (rectified)")
    pretty_transform(T_imu_right, "T_imu_rightcam (RESULT)")
    pretty_transform(T_right_imu, "T_rightcam_imu (inverse)")

    # YAML-format outputs
    print("\nYAML OUTPUT")
    print("===========")
    if args.output in ["imu2right", "both"]:
        print_yaml_opencv_matrix(T_imu_right, "T_imu_rightcam")
    if args.output in ["right2imu", "both"]:
        print_yaml_opencv_matrix(T_right_imu, "T_rightcam_imu")


if __name__ == "__main__":
    main()
