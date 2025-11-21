#!/usr/bin/env python3
import numpy as np
import cv2

def load_calibration(yaml_file):
    """Load calibration parameters from YAML file using OpenCV FileStorage"""
    fs = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Cannot open calibration file: {yaml_file}")
    
    calib = {}
    # Camera parameters
    calib['Camera.fx'] = fs.getNode('Camera.fx').real()
    calib['Camera.fy'] = fs.getNode('Camera.fy').real()
    calib['Camera.bf'] = fs.getNode('Camera.bf').real()

    # Transformation matrices
    calib['Tic'] = fs.getNode('Tic').mat()

    # Stereo rectification matrices
    calib['LEFT.R'] = fs.getNode('LEFT.R').mat()
    calib['LEFT.P'] = fs.getNode('LEFT.P').mat()
    calib['RIGHT.R'] = fs.getNode('RIGHT.R').mat()
    calib['RIGHT.P'] = fs.getNode('RIGHT.P').mat()

    fs.release()
    return calib

def matrix_to_transform(R, t):
    """Convert rotation matrix and translation vector to 4x4 transformation matrix"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def compute_T_leftcam_rightcam_from_P(left_P, right_P):
    """
    Compute T_leftcam_rightcam directly from rectified projection matrices.

    For typical OpenCV rectification:
        P_left  = K [I | 0]
        P_right = K [I | [Tx, 0, 0]^T]

    Tx encodes the baseline (in units of meters, with sign).
    Baseline B is conventionally:
        B = - Tx = - P_right[0,3] / fx

    Then T_left->right is:
        R = I
        t = [-B, 0, 0]^T
    """
    fx = left_P[0, 0]
    # Tx in camera coords:
    Tx = right_P[0, 3] / fx
    # Baseline magnitude
    baseline = -Tx

    T = np.eye(4)
    T[0, 3] = -baseline  # t = [-B, 0, 0]
    return T, baseline

def print_transform(T, name):
    """Pretty print a transformation matrix"""
    print(f"\n{name}:")
    print("=" * 60)
    print("Rotation matrix:")
    print(T[:3, :3])
    print("\nTranslation vector:")
    print(T[:3, 3])
    print("\nFull 4x4 matrix:")
    print(T)

    # Try to show Euler angles if scipy is available
    try:
        from scipy.spatial.transform import Rotation
        R = T[:3, :3]
        r = Rotation.from_matrix(R)
        euler = r.as_euler('xyz', degrees=True)
        print(f"\nEuler angles (xyz, degrees): {euler}")
    except ImportError:
        print("\n[scipy not available, skipping Euler angle computation]")

    print(f"Translation (meters): {T[:3, 3]}")
    print("=" * 60)

def print_yaml_format(T, name):
    """Print transformation in YAML format for easy copy-paste"""
    print(f"\n{name} (YAML format):")
    print(f"{name}: !!opencv-matrix")
    print("   rows: 4")
    print("   cols: 4")
    print("   dt: d")
    data_str = ", ".join([f"{val:15.11f}" for val in T.flatten()])
    print(f"   data: [{data_str}]")

def main():
    yaml_file = "bob.yaml"
    print(f"Loading calibration from {yaml_file}...")
    calib = load_calibration(yaml_file)

    # Tic assumed to be T_leftcam_imu (left camera -> IMU)
    Tic = calib['Tic']
    print_transform(Tic, "Tic (T_leftcam_imu)")

    # If Tic = T_leftcam_imu, then T_imu_leftcam = inv(Tic)
    # If your YAML defines Tic as T_imu_leftcam, then REMOVE the inverse.
    T_imu_leftcam = Tic
    # T_imu_leftcam = Tic
    print_transform(T_imu_leftcam, "T_imu_leftcam (inv(Tic))")

    left_P = calib['LEFT.P']
    right_P = calib['RIGHT.P']

    # Compute transformation from left camera to right camera in rectified frame
    T_leftcam_rightcam, baseline = compute_T_leftcam_rightcam_from_P(left_P, right_P)
    T_leftcam_rightcam = np.linalg.inv(T_leftcam_rightcam)
    print_transform(T_leftcam_rightcam, "T_leftcam_rightcam")

    # Final transformation IMU -> right camera
    T_imu_rightcam = T_imu_leftcam @ T_leftcam_rightcam
    T_rightcam_imu = np.linalg.inv(T_imu_rightcam)
    print_transform(T_imu_rightcam, "T_imu_rightcam (RESULT)")
    print_transform(T_rightcam_imu, "T_rightcam_imu (inverse of RESULT)")

    # YAML-format output
    print("\n" + "=" * 60)
    print("YAML FORMAT OUTPUT")
    print("=" * 60)
    print_yaml_format(T_imu_rightcam, "T_imu_rightcam")

    # Baseline sanity check
    fx = calib['Camera.fx']
    print(f"\nStereo baseline (from P): {baseline:.6f} meters")
    print(f"Camera.bf from file:       {calib['Camera.bf']:.6f}")
    print(f"Computed baseline * fx:    {baseline * fx:.6f}")

if __name__ == "__main__":
    main()
