import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce.values import Values
from symforce import codegen
from symforce.codegen import CppConfig
from symforce.codegen import geo_package_codegen
from pathlib import Path
import os
import re


def predict_states(
    pos: sf.V3, vel: sf.V3, quat: sf.Rot3, ab: sf.V3, gb: sf.V3,
    P: sf.Matrix, delta_vel: sf.V3, delta_ang: sf.V3, 
    delta_vel_dt: sf.Scalar, delta_angle_dt: sf.Scalar,
    accel_noise_var: sf.Scalar, gyro_noise_var: sf.Scalar, accel_walk_var: sf.Scalar, gyro_walk_var: sf.Scalar,
    epsilon: sf.Scalar):
    """
    ESKF prediction step based on: https://arxiv.org/pdf/1711.02508
    Uses delta angle and delta vel formulation for proper high-rate IMU integration.
    """
    
    GRAVITY = 9.81
    
    # correct biases
    delta_angle_corr = delta_ang - gb * delta_angle_dt
    delta_vel_corr = delta_vel - ab * delta_vel_dt
    
    # attitude propagation
    delta_quat = sf.Rot3.from_tangent(delta_angle_corr, epsilon=epsilon)
    pred_quat = quat * delta_quat
    

    C_n_b = quat.to_rotation_matrix()  # Body to nav rotation (current)
    R_delta = delta_quat.to_rotation_matrix()  # Delta rotation matrix
    
    # velocity and position
    delta_vel_nav = C_n_b * delta_vel_corr
    delta_vel_nav = delta_vel_nav + sf.V3(0, 0, GRAVITY) * delta_vel_dt
    
    pred_pos = pos + vel * delta_vel_dt + 0.5 * delta_vel_nav * delta_vel_dt
    pred_vel = vel + delta_vel_nav
    
    # biases
    pred_ab = ab
    pred_gb = gb
    
    # state transition matrix
    F = sf.Matrix.eye(15)
    
    # dp/dv: position depends on velocity
    F[0, 3] = delta_vel_dt
    F[1, 4] = delta_vel_dt
    F[2, 5] = delta_vel_dt
    
    # dv/dtheta: velocity depends on attitude error
    # -C_n_b * skew(delta_vel_corr)
    skew_dv = sf.Matrix([
        [0, -delta_vel_corr[2], delta_vel_corr[1]],
        [delta_vel_corr[2], 0, -delta_vel_corr[0]],
        [-delta_vel_corr[1], delta_vel_corr[0], 0]
    ])
    dv_dtheta = -C_n_b * skew_dv
    for i in range(3):
        for j in range(3):
            F[3 + i, 6 + j] = dv_dtheta[i, j]
    
    # dv/da_b: velocity depends on accel bias
    # -C_n_b * dt
    dv_dab = -C_n_b * delta_vel_dt
    for i in range(3):
        for j in range(3):
            F[3 + i, 9 + j] = dv_dab[i, j]

    # dtheta/dtheta: attitude error transition
    # R_delta^T
    R_delta_T = R_delta.T
    for i in range(3):
        for j in range(3):
            F[6 + i, 6 + j] = R_delta_T[i, j]
    
    # dtheta/dw_b: attitude error depends on gyro bias
    # -I * dt
    F[6, 12] = -delta_angle_dt
    F[7, 13] = -delta_angle_dt
    F[8, 14] = -delta_angle_dt

    # Process noise
    Q = sf.Matrix.zeros(15, 15)
    # Velocity noise
    Q[3, 3] = accel_noise_var * delta_vel_dt * delta_vel_dt
    Q[4, 4] = accel_noise_var * delta_vel_dt * delta_vel_dt
    Q[5, 5] = accel_noise_var * delta_vel_dt * delta_vel_dt
    # Attitude noise
    Q[6, 6] = gyro_noise_var * delta_angle_dt * delta_angle_dt
    Q[7, 7] = gyro_noise_var * delta_angle_dt * delta_angle_dt
    Q[8, 8] = gyro_noise_var * delta_angle_dt * delta_angle_dt
    # Accel bias random walk
    Q[9, 9] = accel_walk_var * delta_vel_dt
    Q[10, 10] = accel_walk_var * delta_vel_dt
    Q[11, 11] = accel_walk_var * delta_vel_dt
    # Gyro bias random walk
    Q[12, 12] = gyro_walk_var * delta_angle_dt
    Q[13, 13] = gyro_walk_var * delta_angle_dt
    Q[14, 14] = gyro_walk_var * delta_angle_dt


    # update covariance
    P_pred = F * P * F.T + Q

    return pred_pos, pred_vel, pred_quat, pred_ab, pred_gb, P_pred

def generate_prediction_code():
    cpp_config = CppConfig()

    # Define symbolic inputs
    inputs = Values(
        pos=sf.V3.symbolic("pos"),
        vel=sf.V3.symbolic("vel"), 
        quat=sf.Rot3.symbolic("quat"),
        ab=sf.V3.symbolic("ab"),
        gb=sf.V3.symbolic("gb"),
        P=sf.Matrix([[sf.Symbol(f"P_{i}_{j}") for j in range(15)] for i in range(15)]),
        delta_vel=sf.V3.symbolic("delta_vel"),
        delta_ang=sf.V3.symbolic("delta_ang"),
        delta_vel_dt=sf.Symbol("delta_vel_dt"),
        delta_angle_dt=sf.Symbol("delta_angle_dt"),
        accel_noise_var=sf.Symbol("accel_noise_var"),
        gyro_noise_var=sf.Symbol("gyro_noise_var"),
        accel_walk_var=sf.Symbol("accel_walk_var"),
        gyro_walk_var=sf.Symbol("gyro_walk_var"),
        epsilon=sf.Symbol("epsilon"),
    )

    # Call the predict_states function with symbolic inputs
    outputs = predict_states(
        pos=inputs["pos"],
        vel=inputs["vel"],
        quat=inputs["quat"],
        ab=inputs["ab"],
        gb=inputs["gb"],
        P=inputs["P"],
        delta_vel=inputs["delta_vel"],
        delta_ang=inputs["delta_ang"],
        delta_vel_dt=inputs["delta_vel_dt"],
        delta_angle_dt=inputs["delta_angle_dt"],
        accel_noise_var=inputs["accel_noise_var"],
        gyro_noise_var=inputs["gyro_noise_var"],
        accel_walk_var=inputs["accel_walk_var"],
        gyro_walk_var=inputs["gyro_walk_var"],
        epsilon=inputs["epsilon"],
    )

    # Package outputs
    output_vals = Values(
        pred_pos=outputs[0],
        pred_vel=outputs[1], 
        pred_quat=outputs[2],
        pred_ab=outputs[3],
        pred_gb=outputs[4],
        P_pred=outputs[5],
    )

    gen = codegen.Codegen(
        inputs=inputs, 
        outputs=output_vals, 
        config=cpp_config,
        name="predict_states",
    )

    output_dir = Path("src/gnc/state_estimation/math/generated_code")
    lib_dir = Path("lib/symforce")

    gen.generate_function(
        output_dir=output_dir,
        namespace="gnc",
    )

    # Post-process: replace <Eigen/...> with <ArduinoEigen/Eigen/...> and change sym includes
    patch_eigen_includes(output_dir) 

    print("Done! Generated code is in:")
    print(f"  Function: {output_dir}")
    print(f"  Sym package: {lib_dir}")

def compute_attitude_innov_var(
    P: sf.Matrix,
    H: sf.Matrix,
    R: sf.Scalar):
    """
    Compute scalar innovation variance for each axis of a 3D attitude measurement.
    H is 3x15, but we only use the attitude portion
    P is 15x15 covariance matrix.
    """
    H_sub = H[:, 6:9]
    P_theta = P[6:9, 6:9]
    
    innov_var = sf.V3.zeros(3, 1)
    
    for i in range(3):
        H_row = H_sub[i, :]
        s_val = (H_row * P_theta * H_row.T)[0, 0] + R
        innov_var[i] = s_val
        
    return innov_var

def generate_compute_attitude_innov_var_code():
    cpp_config = CppConfig()

    inputs = Values(
        P=sf.Matrix([[sf.Symbol(f"P_{i}_{j}") for j in range(15)] for i in range(15)]),
        H=sf.Matrix([[sf.Symbol(f"H_{i}_{j}") for j in range(15)] for i in range(3)]),  
        R=sf.Symbol("R"),
    )

    innov_var = compute_attitude_innov_var(
        P=inputs["P"],
        H=inputs["H"],
        R=inputs["R"],
    )

    output_vals = Values(
        innov_var=innov_var,
    )

    gen = codegen.Codegen(
        inputs=inputs,
        outputs=output_vals,
        config=cpp_config,
        name="compute_attitude_innov_var",
    )

    output_dir = Path("src/gnc/state_estimation/math/generated_code")

    gen.generate_function(
        output_dir=output_dir,
        namespace="gnc",
    )

    patch_eigen_includes(output_dir)


def patch_eigen_includes(output_dir: Path):
    """
    Post-process generated C++ files:
    1. Replace desktop Eigen includes with ArduinoEigen
    2. Replace sym includes
    """
    for root, _, files in os.walk(output_dir):
        for fname in files:
            if not fname.endswith('.h'):
                continue
            
            fpath = os.path.join(root, fname)
            with open(fpath, 'r') as f:
                content = f.read()
            original_content = content
            
            content = re.sub(
                r'#include <Eigen/(\w+)>',
                r'#include <ArduinoEigen/Eigen/\1>',
                content
            )

            content = re.sub(
                r'#include <sym/rot3.h>',
                r'#include "sym_minimal.h"',
                content
            )

            if content != original_content:
                with open(fpath, 'w') as f:
                    f.write(content)
                print(f"  Patched includes in: {fpath}")


if __name__ == "__main__":
    generate_prediction_code()
    generate_compute_attitude_innov_var_code()

