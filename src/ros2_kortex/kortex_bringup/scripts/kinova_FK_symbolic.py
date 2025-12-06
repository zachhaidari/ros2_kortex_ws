#!/usr/bin/env python3
"""
Symbolic forward kinematics for Kinova Gen3 Lite (6-DOF).
Based on the exact symbolic equations derived from DH parameters.

This is the most accurate FK implementation - uses the exact symbolic expressions.
"""

import math
import numpy as np


def fk_gen3_lite_symbolic(q):
    """
    Forward kinematics using exact symbolic equations.
    
    Parameters
    ----------
    q : iterable of 6 floats
        Joint angles [q1..q6] in radians
    
    Returns
    -------
    T : (4,4) numpy.ndarray
        Homogeneous transformation matrix from base to tool frame
    """
    if len(q) != 6:
        raise ValueError(f"Expected 6 joint values, got {len(q)}")
    
    q1, q2, q3, q4, q5, q6 = q
    
    # Precompute sin/cos for efficiency
    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s3, c3 = math.sin(q3), math.cos(q3)
    s4, c4 = math.sin(q4), math.cos(q4)
    s5, c5 = math.sin(q5), math.cos(q5)
    s6, c6 = math.sin(q6), math.cos(q6)
    
    # Compute (q2 - q3) terms
    s23 = math.sin(q2 - q3)
    c23 = math.cos(q2 - q3)
    
    # Build transformation matrix using symbolic equations
    T = np.zeros((4, 4), dtype=float)
    
    # Row 1
    T[0,0] = -((s1*c4 + s4*c1*c23)*c5 + s5*s23*c1)*s6 - (s1*s4 - c1*c4*c23)*c6
    T[0,1] = -((s1*c4 + s4*c1*c23)*c5 + s5*s23*c1)*c6 + (s1*s4 - c1*c4*c23)*s6
    T[0,2] = (s1*c4 + s4*c1*c23)*s5 - s23*c1*c5
    T[0,3] = (-0.057*s1*s4 + 0.235*s1*s5*c4 + 0.01*s1 - 0.28*s2*c1 + 
              0.235*s4*s5*c1*c23 - 0.235*s23*c1*c5 - 0.245*s23*c1 + 0.057*c1*c4*c23)
    
    # Row 2
    T[1,0] = -((s1*s4*c23 - c1*c4)*c5 + s1*s5*s23)*s6 + (s1*c4*c23 + s4*c1)*c6
    T[1,1] = -((s1*s4*c23 - c1*c4)*c5 + s1*s5*s23)*c6 - (s1*c4*c23 + s4*c1)*s6
    T[1,2] = (s1*s4*c23 - c1*c4)*s5 - s1*s23*c5
    T[1,3] = (-0.28*s1*s2 + 0.235*s1*s4*s5*c23 - 0.235*s1*s23*c5 - 0.245*s1*s23 + 
              0.057*s1*c4*c23 + 0.057*s4*c1 - 0.235*s5*c1*c4 - 0.01*c1)
    
    # Row 3
    T[2,0] = -(s4*s23*c5 - s5*c23)*s6 + s23*c4*c6
    T[2,1] = -(s4*s23*c5 - s5*c23)*c6 - s6*s23*c4
    T[2,2] = s4*s5*s23 + c5*c23
    T[2,3] = (0.235*s4*s5*s23 + 0.057*s23*c4 + 0.28*c2 + 
              0.235*c5*c23 + 0.245*c23 + 0.2433)
    
    # Row 4 (homogeneous)
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0
    
    return T


if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)
    
    # Test with q = [0]*6
    print("="*60)
    print("TEST: q = [0, 0, 0, 0, 0, 0]")
    print("="*60)
    q_zero = [0.0] * 6
    T_zero = fk_gen3_lite_symbolic(q_zero)
    print("Transformation Matrix:")
    print(T_zero)
    print(f"Position [x, y, z]: {T_zero[0:3, 3]}")
    print()
    
    # Original validated home position (positive values)
    print("="*60)
    print("ORIGINAL HOME: [0.0, 1.0, 2.05, 1.615, 0.55, 0.0]")
    print("="*60)
    q_orig = [0.0, 1.0, 2.05, 1.615, 0.55, 0.0]
    T_orig = fk_gen3_lite_symbolic(q_orig)
    print("Transformation Matrix:")
    print(T_orig)
    print(f"Position [x, y, z]: {T_orig[0:3, 3]}")
    
    # Extract RPY
    r31 = T_orig[2, 0]
    r11 = T_orig[0, 0]
    r21 = T_orig[1, 0]
    r32 = T_orig[2, 1]
    r33 = T_orig[2, 2]
    
    pitch = math.atan2(-r31, math.sqrt(r11**2 + r21**2))
    yaw = math.atan2(r21, r11)
    roll = math.atan2(r32, r33)
    
    print(f"Roll-Pitch-Yaw (deg): [{math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}]")
    print()
    
    # Corrected home position (negative values)
    print("="*60)
    print("CORRECTED HOME: [0.0, -1.0, -2.05, -1.615, 0.55, 0.0]")
    print("="*60)
    q_corr = [0.0, -1.0, -2.05, -1.615, 0.55, 0.0]
    T_corr = fk_gen3_lite_symbolic(q_corr)
    print("Transformation Matrix:")
    print(T_corr)
    print(f"Position [x, y, z]: {T_corr[0:3, 3]}")
    
    # Extract RPY
    r31 = T_corr[2, 0]
    r11 = T_corr[0, 0]
    r21 = T_corr[1, 0]
    r32 = T_corr[2, 1]
    r33 = T_corr[2, 2]
    
    pitch = math.atan2(-r31, math.sqrt(r11**2 + r21**2))
    yaw = math.atan2(r21, r11)
    roll = math.atan2(r32, r33)
    
    print(f"Roll-Pitch-Yaw (deg): [{math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}]")
    print()
    
    print("="*60)
    print("COMPARISON")
    print("="*60)
    print(f"Original position: {T_orig[0:3, 3]}")
    print(f"Corrected position: {T_corr[0:3, 3]}")
    print(f"Difference: {T_orig[0:3, 3] - T_corr[0:3, 3]}")
