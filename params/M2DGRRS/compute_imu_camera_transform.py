#!/usr/bin/env python3
import numpy as np
import os

def parse_calibration_file(filepath):
    """
    Parses the calibration file to extract extrinsic matrices.
    Returns a dictionary mapping sensor name to its 4x4 extrinsic matrix (T_Lidar_Sensor).
    """
    extrinsics = {}
    current_sensor = None
    current_tag = ""
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        if line.startswith('%%'):
            current_sensor = line[2:].strip()
            i += 1
            continue
            
        if line.startswith('%') and not line.startswith('%%'):
            current_tag = line
            i += 1
            continue
            
        if line.startswith('data:') and current_sensor and ('Extrinsic' in current_tag):
            # Parse data
            matrix_data = []
            
            # Loop to collect tokens from this line and subsequent lines
            while i < len(lines):
                curr_line_content = lines[i].strip()
                if not curr_line_content and len(matrix_data) == 0:
                     # Skip empty lines if we haven't started (though we started at 'data:')
                     i += 1
                     continue
                
                # If we hit a new tag or sensor, stop
                if (curr_line_content.startswith('%') or 'data:' in curr_line_content) and len(matrix_data) > 0:
                     # If we see "data:" again, it's a new block (shouldn't happen if we consume it)
                     # But if we see "data:" on the FIRST line, we process it.
                     pass
                if curr_line_content.startswith('%'):
                    break

                # Remove 'data:'
                clean_content = curr_line_content.replace('data:', '')
                # Clean brackets/commas
                clean_content = clean_content.replace('[', ' ').replace(']', ' ').replace(',', ' ')
                
                tokens = clean_content.split()
                
                all_valid = True
                for token in tokens:
                    try:
                        val = float(token)
                        matrix_data.append(val)
                    except ValueError:
                        all_valid = False
                        break
                
                if not all_valid:
                    # hit something like gyr_n, stop
                    break
                    
                if len(matrix_data) >= 16:
                    i += 1 
                    break
                
                # Check if we have 12 elements (3x4 matrix) and the data block effectively ended (closed bracket)
                if len(matrix_data) == 12 and ']' in curr_line_content:
                     i += 1
                     break
                
                i += 1
            
            if len(matrix_data) == 16:
                extrinsics[current_sensor] = np.array(matrix_data).reshape(4, 4)
            elif len(matrix_data) == 12:
                # Append last row for 4x4 homogeneous matrix
                mat3x4 = np.array(matrix_data).reshape(3, 4)
                last_row = np.array([0., 0., 0., 1.]).reshape(1, 4)
                extrinsics[current_sensor] = np.vstack((mat3x4, last_row))
            
            continue

        i += 1
        
    return extrinsics

def compute_relative_transform(T_Lidar_Source, T_Lidar_Target):
    """
    Computes T_Target_Source = inv(T_Lidar_Target) * T_Lidar_Source
    """
    T_Lidar_Target_inv = np.linalg.inv(T_Lidar_Target)
    T_Target_Source = np.dot(T_Lidar_Target_inv, T_Lidar_Source)
    return T_Target_Source

def main():
    # Path to the calibration file
    # Assuming the script is run from Kimera-VIO/scripts or similar, but using absolute path for robustness
    # based on user workspace context
    calib_file = "/workspaces/src/Kimera-VIO/params/M2DGRStereo/calibration_results.txt"
    
    if not os.path.exists(calib_file):
        print(f"Error: Calibration file not found at {calib_file}")
        return

    print(f"Reading calibration from: {calib_file}")
    extrinsics = parse_calibration_file(calib_file)
    
    # Identify keys
    # Based on the file content viewed:
    # "Handsfree IMU"
    # "Cam-left 8823"
    # "Cam-right 8828"
    
    key_imu = "Handsfree IMU"
    key_cam_left = "Cam-pinhole-color realsense d435i"
    key_cam_right = "Cam-right 8828"
    
    if key_imu not in extrinsics:
        print(f"Error: Could not find extrinsic for {key_imu}")
        print("Available keys:", extrinsics.keys())
        return
    if key_cam_left not in extrinsics:
        print(f"Error: Could not find extrinsic for {key_cam_left}")
        return
    if key_cam_right not in extrinsics:
        print(f"Error: Could not find extrinsic for {key_cam_right}")
        return

    T_Lidar_IMU = extrinsics[key_imu]
    T_Lidar_CamLeft = extrinsics[key_cam_left]
    T_Lidar_CamRight = extrinsics[key_cam_right]

    # Compute T_CamLeft_IMU
    T_CamLeft_IMU = compute_relative_transform(T_Lidar_IMU, T_Lidar_CamLeft)
    T_IMU_CamLeft = compute_relative_transform(T_Lidar_CamLeft, T_Lidar_IMU)
    
    # Compute T_CamRight_IMU
    T_CamRight_IMU = compute_relative_transform(T_Lidar_IMU, T_Lidar_CamRight)
    T_IMU_CamRight = compute_relative_transform(T_Lidar_CamRight, T_Lidar_IMU)

    def print_opencv_matrix(name, matrix):
        print(f"{name}:")
        print(f"  cols: {matrix.shape[1]}")
        print(f"  rows: {matrix.shape[0]}")
        # Format data list with some checking for readability
        print("  data: [", end="")
        params = matrix.flatten()
        for i, val in enumerate(params):
            if i > 0:
                print(", ", end="")
            # improved formatting to avoid super long lines
            if i > 0 and i % 4 == 0: 
                print("\n         ", end="")
            print(f"{val}", end="")
        print("]")

    print("\n%YAML:1.0")
    print("# Transformation from Handsfree IMU to Cam-left") 
    print_opencv_matrix("T_CamLeft_IMU", T_CamLeft_IMU)
    print_opencv_matrix("T_IMU_CamLeft", T_IMU_CamLeft)

    print("\n# Transformation from Handsfree IMU to Cam-right")
    print_opencv_matrix("T_CamRight_IMU", T_CamRight_IMU)
    print_opencv_matrix("T_IMU_CamRight", T_IMU_CamRight)

if __name__ == "__main__":
    main()
