#!/usr/bin/env python3
"""
Numeric forward kinematics for Kinova Gen3 Lite (6-DOF) using classical DH parameters.

This is a pure-numeric (math/numpy) version suitable for real-time use in ROS2 nodes.

Joint order assumed: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
Angles in radians.
"""

import math
import numpy as np


def dh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """
    Classical DH transform:
      Rot(z, theta) * Trans(z, d) * Trans(x, a) * Rot(x, alpha)
    """
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,      ca,      d],
        [0.0,    0.0,     0.0,    1.0],
    ], dtype=float)


def fk_gen3_lite_numpy(q):
    """
    Forward kinematics for the Kinova Gen3 Lite.

    Parameters
    ----------
    q : iterable of 6 floats
        Joint angles [q1..q6] in radians, in the order:
        [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]

    Returns
    -------
    T : (4,4) numpy.ndarray
        Homogeneous transform from base frame to tool frame.
    """
    if len(q) != 6:
        raise ValueError(f"Expected 6 joint values, got {len(q)}")

    q1, q2, q3, q4, q5, q6 = q

    # Classical DH parameters (same as in the SymPy script), in meters
    alphas = [
        math.pi / 2.0,   # i = 1
        math.pi,         # i = 2
        math.pi / 2.0,   # i = 3
        math.pi / 2.0,   # i = 4
        math.pi / 2.0,   # i = 5
        0.0              # i = 6
    ]

    a_list = [
        0.0,       # i = 1
        0.280,     # i = 2  (280 mm)
        0.0,       # i = 3
        0.0,       # i = 4
        0.0,       # i = 5
        0.0        # i = 6
    ]

    d_list = [
        0.2433,    # i = 1  (128.3 + 115.0) mm -> 0.2433 m
        0.03,      # i = 2  (30.0 mm)
        0.02,      # i = 3  (20.0 mm)
        0.245,     # i = 4  (140.0 + 105.0) mm -> 0.245 m
        0.057,     # i = 5  (28.5 + 28.5) mm -> 0.057 m
        0.235      # i = 6  (105.0 + 130.0) mm -> 0.235 m
    ]

    # Joint angle offsets from Kinova’s DH table
    theta_list = [
        q1,                   # θ1 = q1
        q2 + math.pi / 2.0,   # θ2 = q2 + π/2
        q3 + math.pi / 2.0,   # θ3 = q3 + π/2
        q4 + math.pi / 2.0,   # θ4 = q4 + π/2
        q5 + math.pi,         # θ5 = q5 + π
        q6 + math.pi / 2.0    # θ6 = q6 + π/2
    ]

    T = np.eye(4, dtype=float)
    for i in range(6):
        T = T @ dh_transform(alphas[i], a_list[i], d_list[i], theta_list[i])

    return T


if __name__ == "__main__":
    # Quick self-test: q = [0]*6 should match the SymPy result
    q_zero = [0.0] * 6
    T = fk_gen3_lite_numpy(q_zero)
    print("T(q=0):")
    np.set_printoptions(precision=6, suppress=True)
    print(T)
    print("Position [x, y, z]:", T[0:3, 3])
    print()
    
    # Ideal home position - validated to be horizontal and parallel to table
    print("="*60)
    print("IDEAL HOME POSITION: [0.0, -1.0, -2.05, -1.615, 0.55, 0.0]")
    print("="*60)
    print()
    
    q_home = [0.0, 1.0, 2.05, 1.615, 0.55, 0.0]
    T_home = fk_gen3_lite_numpy(q_home)
    
    print("Homogeneous Transformation Matrix T:")
    print(T_home)
    print()
    
    print("Position [x, y, z]:", T_home[0:3, 3])
    print()
    
    print("Rotation Matrix R:")
    print(T_home[0:3, 0:3])
    print()
    
    # Extract roll, pitch, yaw
    r11, r12, r13 = T_home[0, 0:3]
    r21, r22, r23 = T_home[1, 0:3]
    r31, r32, r33 = T_home[2, 0:3]
    
    pitch = math.atan2(-r31, math.sqrt(r11**2 + r21**2))
    yaw = math.atan2(r21, r11)
    roll = math.atan2(r32, r33)
    
    print(f"Roll-Pitch-Yaw (rad): [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]")
    print(f"Roll-Pitch-Yaw (deg): [{math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}]")

