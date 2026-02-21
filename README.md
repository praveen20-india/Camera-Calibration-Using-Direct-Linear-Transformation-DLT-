# 📷 Camera Calibration using DLT

## 📌 Overview

Camera calibration is a fundamental problem in computer vision that involves determining the internal and external parameters of a camera. This project implements and evaluates camera calibration from 2D-3D point correspondences using the Direct Linear Transformation (DLT) method. The implementation includes generating synthetic camera data, simulating the projection of 3D points onto a 2D image plane, adding realistic Gaussian noise, and robustly recovering the original projection matrices through constrained least squares optimization. 

## 🎯 Objectives

* **Implement** the Direct Linear Transformation (DLT) algorithm for camera calibration from scratch.
* **Recover** intrinsic parameters (focal length, principal point) and extrinsic parameters (rotation, translation) utilizing RQ decomposition.
* **Analyze** the accuracy of the calibration under varying numbers of 3D-2D point correspondences (6, 10, and 50 points).
* **Evaluate** the robustness of the DLT algorithm against realistic measurement noise (e.g., sensor imperfection simulated via Gaussian noise).

## 🧠 Mathematical Foundations

* **Projection Matrix Formulation**: The pinhole camera model relates 3D world coordinates to 2D image coordinates via a $3 \times 4$ projection matrix $P$. This matrix is the product of the camera's intrinsic matrix $K$ and its extrinsic parameters (rotation $R$ and translation $t$): $P = K[R|t]$.
* **Direct Linear Transformation (DLT)**: Given a set of $n$ known 3D world points and their corresponding 2D image projections, DLT sets up a homogeneous system of linear equations $Q \cdot A = 0$ (or non-homogeneous $Q \cdot A = B$ when normalizing $P_{34} = 1$). Each point correspondence provides 2 linear constraints.
* **Least Squares Solution**: Since the system is over-determined for $n \ge 6$ points, an algebraic least squares solution is computed to estimate the elements of the projection matrix $P$.
* **RQ Decomposition**: Once $P$ is recovered, the left $3 \times 3$ submatrix $M = KR$ is factored using RQ decomposition into an upper-triangular matrix $K$ (intrinsics) and an orthogonal matrix $R$ (extrinsics).
* **Reprojection Error**: To measure calibration accuracy, the original 3D points are re-projected onto the 2D plane using the estimated matrix $P$. The Euclidean distance between the ground truth 2D coordinates and the re-projected coordinates constitutes the reprojection error.

## 🛠️ Technologies Used

- **Python**: Core programming language.
- **NumPy**: Linear algebra, array manipulation, and least squares optimization.
- **SciPy (`scipy.linalg.rq`)**: RQ decomposition for parameter extraction.
- **Matplotlib**: Data visualization and error plotting.

## 📊 Experimental Results

![Calibration Results](calibration_results.png)

The algorithm was evaluated computationally with zero-mean Gaussian noise ($\sigma = 0.5$ pixels) added to 2D observations. The noise tolerance was directly evaluated across different numbers of 3D-2D correspondences. 

| Points | Additive Noise ($\sigma$) | Average Reprojection Error (pixels) | K Matrix Estimation Error | Rotation Matrix Error |
|--------|---------------------------|-------------------------------------|---------------------------|-----------------------|
| 6      | None                      | $1.35 \times 10^{-8}$               | Near zero                 | Near zero             |
| **6**  | **0.5 px**                | **0.581**                           | High (~144.59)*           | High (~0.199)*        |
| **10** | **0.5 px**                | **0.323**                           | Moderate (~10.94)*        | Moderate (~0.004)*    |
| **50** | **0.5 px**                | **0.223**                           | Low (~3.69)*              | Low (~0.003)*         |

*\*Errors represent the Frobenius norm difference between parameter matrices obtained from noise-free vs noisy reconstructions.*

### Key Insights:
1. **Mathematical Conditioning**: Ill-conditioned minimal point configurations (6 points) lead to severe parameter estimation drift under noise. 
2. **Robustness with Redundancy**: Supplying redundant geometric constraints (50 points) significantly smooths the estimation process, drastically mitigating parameter drift and producing accurate rotation structures despite spatial noise.
3. **Axis Skew Sensitivity**: The parameter optimization assigns structural perturbations from uniform Gaussian noise to the intrinsic skew parameter $\gamma$. While physically zero in modern digital sensors, the numerical minimization exploits this unconstrained degree of freedom.

## 🚀 How to Run

1. Clone the repository and navigate to the project directory.
2. Ensure you have the required dependencies installed:
   ```bash
   pip install -r requirements.txt
   ```
3. Run the calibration script:
   ```bash
   python camera_calibration_dlt.py
   ```
4. The script will output the structural errors in the terminal and generate a validation plot `calibration_results.png`.

## 🤖 Applications in Robotics and Computer Vision

* **Simultaneous Localization and Mapping (SLAM)**: Accurate intrinsic parameter extraction is fundamental for visual odometry and tracking pipelines.
* **3D Reconstruction**: Reliable camera matrices map pixel measurements to structural rays.
* **Augmented Reality (AR)**: Sub-pixel calibration accuracy guarantees rigid anchoring of virtual objects within real-world video arrays.

## 📈 Key Takeaways

* While DLT is highly efficient and offers a closed-form algebraic solution, adding more point correspondences linearly constrains the system, massively reducing sensitivity to measurement noise.
* Real-world implementations should utilize dozens of well-distributed feature points to effectively "anchor" the parameter estimation procedure.
