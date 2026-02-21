import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.linalg import rq

def create_3d_homogeneous_points(num_points: int) -> np.ndarray:
    """
    Generate random 3D points in homogeneous coordinates.
    
    Args:
        num_points: Number of 3D points to generate.
        
    Returns:
        A 4xN numpy array of 3D points in homogeneous coordinates.
    """
    points = np.random.randint(-480, 480, size=(num_points, 3))
    ones = np.ones((num_points, 1))
    return np.concatenate((points, ones), axis=1).T

def compute_2d_homogeneous_coordinates(P: np.ndarray, points_3d: np.ndarray) -> np.ndarray:
    """
    Project 3D homogeneous points to 2D image plane using projection matrix P.
    
    Args:
        P: 3x4 projection matrix.
        points_3d: 4xN array of 3D homogeneous coordinates.
        
    Returns:
        3xN array of 2D homogeneous coordinates.
    """
    return P @ points_3d

def to_cartesian(homogeneous_points: np.ndarray) -> np.ndarray:
    """
    Convert homogeneous coordinates to Cartesian coordinates.
    
    Args:
        homogeneous_points: (N+1)xM array of homogeneous points.
        
    Returns:
        NxM array of Cartesian coordinates.
    """
    return homogeneous_points[:-1, :] / homogeneous_points[-1:, :]

def add_gaussian_noise(points: np.ndarray, mean: float = 0.0, std: float = 0.5) -> np.ndarray:
    """
    Add Gaussian noise to the coordinates.
    
    Args:
        points: Array of points.
        mean: Mean of the Gaussian noise.
        std: Standard deviation of the Gaussian noise.
        
    Returns:
        Array of noisy points.
    """
    noise = np.random.normal(mean, std, points.shape)
    return points + noise

def dlt_calibration(points_3d: np.ndarray, points_2d: np.ndarray) -> np.ndarray:
    """
    Estimate the projection matrix P using the Direct Linear Transformation (DLT) method.
    
    Args:
        points_3d: 3xN array of 3D Cartesian coordinates.
        points_2d: 2xN array of 2D Cartesian coordinates.
        
    Returns:
        3x4 estimated projection matrix.
    """
    n = points_3d.shape[1]
    Q = np.zeros((2 * n, 11))
    B = np.zeros((2 * n, 1))

    for i in range(n):
        X, Y, Z = points_3d[:, i]
        x, y = points_2d[:, i]
        
        Q[2 * i] = [X, Y, Z, 1, 0, 0, 0, 0, -x * X, -x * Y, -x * Z]
        B[2 * i] = x
        
        Q[2 * i + 1] = [0, 0, 0, 0, X, Y, Z, 1, -y * X, -y * Y, -y * Z]
        B[2 * i + 1] = y

    # Least squares solution for Q * A = B
    A, _, _, _ = np.linalg.lstsq(Q, B, rcond=None)
    
    # Append P34 = 1 to the solution
    A = np.append(A, [1])
    A = A.reshape(-1, 1)

    # Reconstruct the 3x4 P matrix
    P_recovered = np.zeros((3, 4))
    for i in range(3):
        P_recovered[i, :] = A[i * 4:(i + 1) * 4].T

    return P_recovered / P_recovered[2, 3]

def get_intrinsics_extrinsics(P: np.ndarray):
    """
    Extract intrinsic (K) and rotation (R) matrices from the projection matrix using RQ decomposition.
    
    Args:
        P: 3x4 projection matrix.
        
    Returns:
        K: 3x3 intrinsic matrix.
        R: 3x3 rotation matrix.
    """
    M = P[:, :3]
    R_matrix, Q_matrix = rq(M)
    
    if np.linalg.det(Q_matrix) < 0:
        Q_matrix = -Q_matrix
        R_matrix = -R_matrix
        
    K = R_matrix / R_matrix[2, 2]
    return K, Q_matrix

def main():
    # --------------------------------------------------------------------------
    # 1. Ground Truth Parameters
    # --------------------------------------------------------------------------
    a_u, a_v = 557.0943, 712.9824
    u_0, v_0 = 326.3819, 298.6679
    gamma = 0
    Tx, Ty, Tz = 100, 0, 1500
    Phix, Phiy, Phix1 = 0.8 * np.pi / 2, -1.8 * np.pi / 2, np.pi / 5

    K = np.array([
        [a_u, gamma, u_0],
        [0, a_v, v_0],
        [0, 0, 1]
    ])

    translation = np.array([[Tx], [Ty], [Tz]])

    # Coordinate frame rotation
    R_x = np.array([[1, 0, 0], [0, np.cos(Phix), -np.sin(Phix)], [0, np.sin(Phix), np.cos(Phix)]])
    R_y = np.array([[np.cos(Phiy), 0, np.sin(Phiy)], [0, 1, 0], [-np.sin(Phiy), 0, np.cos(Phiy)]])
    R_x1 = np.array([[1, 0, 0], [0, np.cos(Phix1), -np.sin(Phix1)], [0, np.sin(Phix1), np.cos(Phix1)]])
    
    rotation = R_x @ R_y @ R_x1
    
    extrinsic = np.hstack((rotation, translation))
    extrinsic = np.vstack((extrinsic, [0, 0, 0, 1]))

    # Projection matrix P = K [I|0] E
    I_0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
    P_gt = K @ I_0 @ extrinsic
    P_gt = P_gt / P_gt[2, 3]

    # --------------------------------------------------------------------------
    # 2. Experimental Evaluation with Different Point Counts
    # --------------------------------------------------------------------------
    point_counts = [6, 10, 50]
    errors = []

    np.random.seed(42)  # For reproducibility

    for n_points in point_counts:
        # Generate 3D points
        points_3d_homo = create_3d_homogeneous_points(n_points)
        points_3d_cart = to_cartesian(points_3d_homo)

        # Project to 2D
        points_2d_homo_gt = compute_2d_homogeneous_coordinates(P_gt, points_3d_homo)
        points_2d_cart_gt = to_cartesian(points_2d_homo_gt)

        # Add Gaussian Noise
        points_2d_cart_noisy = add_gaussian_noise(points_2d_cart_gt, std=0.5)

        # DLT Calibration
        P_recovered = dlt_calibration(points_3d_cart, points_2d_cart_noisy)

        # Reproject 3D points using recovered P
        points_2d_homo_recovered = compute_2d_homogeneous_coordinates(P_recovered, points_3d_homo)
        points_2d_cart_recovered = to_cartesian(points_2d_homo_recovered)

        # Calculate Reprojection Error (Euclidean distance)
        reprojection_error = np.linalg.norm(points_2d_cart_gt - points_2d_cart_recovered, axis=0)
        avg_error = np.mean(reprojection_error)
        errors.append(avg_error)

        print(f"--- Results for {n_points} Points ---")
        print(f"Average Reprojection Error: {avg_error:.4f} pixels")
        
        K_rec, R_rec = get_intrinsics_extrinsics(P_recovered)
        k_error = np.linalg.norm(K - K_rec)
        r_error = np.linalg.norm(rotation - R_rec)
        print(f"Intrinsic Matrix (K) Error: {k_error:.4f}")
        print(f"Rotation Matrix (R) Error: {r_error:.4f}\n")

    # --------------------------------------------------------------------------
    # 3. Validation Plot
    # --------------------------------------------------------------------------
    plt.figure(figsize=(8, 5))
    plt.plot(point_counts, errors, 'bo-', linewidth=2, markersize=8)
    plt.xlabel('Number of Calibration Points', fontsize=12)
    plt.ylabel('Average Reprojection Error (pixels)', fontsize=12)
    plt.title('Camera Calibration: Reprojection Error vs. Points', fontsize=14, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    
    for i, (x, y) in enumerate(zip(point_counts, errors)):
        plt.annotate(f'{y:.4f}', (x, y), xytext=(0, 10), textcoords='offset points', ha='center', fontsize=10)
        
    plt.xlim(min(point_counts) - 5, max(point_counts) + 10)
    plt.ylim(0, max(errors) * 1.2)
    plt.tight_layout()
    plt.savefig('calibration_results.png', dpi=300)
    print("Saved calibration results plot to 'calibration_results.png'")

if __name__ == "__main__":
    main()
