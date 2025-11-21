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
       0.99997545, 0.00384914, 0.00585471, 0.02031279,
         -0.00382868, 0.99998654, -0.00350194, -0.00510325,
         -0.00586812, 0.00347944, 0.99997673, -0.01120139,
          0.0, 0.0, 0.0, 1.0
    ]).reshape((4, 4))
    
    T_inv = invert_transform(T_BS)
    print("Original T_cam_imu:\n", T_BS)
    print("\nT_BS:\n", T_inv)
