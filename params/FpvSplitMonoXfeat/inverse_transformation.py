import numpy as np

def invert_transform(T):
    """
    Inverts a 4x4 homogeneous transformation matrix.
    
    Parameters:
    T (np.ndarray): A 4x4 transformation matrix.
    
    Returns:
    np.ndarray: The inverted 4x4 transformation matrix.
    """
    if T.shape != (4, 4):
        raise ValueError("Input matrix must be 4x4")

    R = T[:3, :3]
    t = T[:3, 3]
    
    R_inv = R.T
    t_inv = -R_inv @ t
    
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    
    return T_inv

if __name__ == "__main__":
    # Example transformation matrix (T_BS from the user's data)
    T_BS = np.array([
        -0.006708021451800439,  0.9999264200621176,  -0.01010727015365992, -0.04586422589354697,
         0.002425643641803532,  0.010091197021688258,  0.9999461405473755,   0.012631813183337478,
         0.9999745590269407,   -0.00673217679702115,  -0.0023577731969991467, -0.05098782892861867,
         0.0,                   0.0,                    0.0,                   1.0
    ]).reshape((4, 4))
    
    T_inv = invert_transform(T_BS)
    print("Original T_cam_imu:\n", T_BS)
    print("\nT_BS:\n", T_inv)
