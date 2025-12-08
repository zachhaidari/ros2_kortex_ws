#!/usr/bin/env python3
"""
Complete symbolic FK from the full transformation matrix equations.
"""

from math import sin, cos, pi
import numpy as np

def fk_complete(q1, q2, q3, q4, q5, q6):
    """
    Complete forward kinematics using the full symbolic expressions.
    Returns 4x4 homogeneous transformation matrix.
    """
    # Row 1
    T11 = -((sin(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2 - q3))*cos(q5) + sin(q5)*sin(q2 - q3)*cos(q1))*sin(q6) - (sin(q1)*sin(q4) - cos(q1)*cos(q4)*cos(q2 - q3))*cos(q6)
    T12 = -((sin(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2 - q3))*cos(q5) + sin(q5)*sin(q2 - q3)*cos(q1))*cos(q6) + (sin(q1)*sin(q4) - cos(q1)*cos(q4)*cos(q2 - q3))*sin(q6)
    T13 = (sin(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2 - q3))*sin(q5) - sin(q2 - q3)*cos(q1)*cos(q5)
    T14 = -0.057*sin(q1)*sin(q4) + 0.235*sin(q1)*sin(q5)*cos(q4) + 0.01*sin(q1) - 0.28*sin(q2)*cos(q1) + 0.235*sin(q4)*sin(q5)*cos(q1)*cos(q2 - q3) - 0.235*sin(q2 - q3)*cos(q1)*cos(q5) - 0.245*sin(q2 - q3)*cos(q1) + 0.057*cos(q1)*cos(q4)*cos(q2 - q3)
    
    # Row 2
    T21 = -((sin(q1)*sin(q4)*cos(q2 - q3) - cos(q1)*cos(q4))*cos(q5) + sin(q1)*sin(q5)*sin(q2 - q3))*sin(q6) + (sin(q1)*cos(q4)*cos(q2 - q3) + sin(q4)*cos(q1))*cos(q6)
    T22 = -((sin(q1)*sin(q4)*cos(q2 - q3) - cos(q1)*cos(q4))*cos(q5) + sin(q1)*sin(q5)*sin(q2 - q3))*cos(q6) - (sin(q1)*cos(q4)*cos(q2 - q3) + sin(q4)*cos(q1))*sin(q6)
    T23 = (sin(q1)*sin(q4)*cos(q2 - q3) - cos(q1)*cos(q4))*sin(q5) - sin(q1)*sin(q2 - q3)*cos(q5)
    T24 = -0.28*sin(q1)*sin(q2) + 0.235*sin(q1)*sin(q4)*sin(q5)*cos(q2 - q3) - 0.235*sin(q1)*sin(q2 - q3)*cos(q5) - 0.245*sin(q1)*sin(q2 - q3) + 0.057*sin(q1)*cos(q4)*cos(q2 - q3) + 0.057*sin(q4)*cos(q1) - 0.235*sin(q5)*cos(q1)*cos(q4) - 0.01*cos(q1)
    
    # Row 3
    T31 = -(sin(q4)*sin(q2 - q3)*cos(q5) - sin(q5)*cos(q2 - q3))*sin(q6) + sin(q2 - q3)*cos(q4)*cos(q6)
    T32 = -(sin(q4)*sin(q2 - q3)*cos(q5) - sin(q5)*cos(q2 - q3))*cos(q6) - sin(q6)*sin(q2 - q3)*cos(q4)
    T33 = sin(q4)*sin(q5)*sin(q2 - q3) + cos(q5)*cos(q2 - q3)
    T34 = 0.235*sin(q4)*sin(q5)*sin(q2 - q3) + 0.057*sin(q2 - q3)*cos(q4) + 0.28*cos(q2) + 0.235*cos(q5)*cos(q2 - q3) + 0.245*cos(q2 - q3) + 0.2433
    
    # Row 4
    T41 = 0.0
    T42 = 0.0
    T43 = 0.0
    T44 = 1.0
    
    T = np.array([
        [T11, T12, T13, T14],
        [T21, T22, T23, T24],
        [T31, T32, T33, T34],
        [T41, T42, T43, T44]
    ])
    
    return T

if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True, linewidth=120)
    
    print("="*70)
    print("COMPLETE SYMBOLIC FK TEST")
    print("="*70)
    
    print("\n1. All zeros [0, 0, 0, 0, 0, 0]:")
    T = fk_complete(0, 0, 0, 0, 0, 0)
    print("Transformation Matrix:")
    print(T)
    print(f"\nPosition: x={T[0,3]:.4f}, y={T[1,3]:.4f}, z={T[2,3]:.4f}")
    print(f"Z-axis (gripper direction): [{T[0,2]:.4f}, {T[1,2]:.4f}, {T[2,2]:.4f}]")
    print(f"X-axis (gripper opening): [{T[0,0]:.4f}, {T[1,0]:.4f}, {T[2,0]:.4f}]")
    
    print("\n" + "="*70)
    print("\n2. Home position [0, -1.0, -2.05, -1.615, 0.55, 0.0]:")
    T = fk_complete(0, -1.0, -2.05, -1.615, 0.55, 0.0)
    print("Transformation Matrix:")
    print(T)
    print(f"\nPosition: x={T[0,3]:.4f}, y={T[1,3]:.4f}, z={T[2,3]:.4f}")
    print(f"Z-axis (gripper direction): [{T[0,2]:.4f}, {T[1,2]:.4f}, {T[2,2]:.4f}]")
    print(f"X-axis (gripper opening): [{T[0,0]:.4f}, {T[1,0]:.4f}, {T[2,0]:.4f}]")
    
    print("\n" + "="*70)
    print("\n3. Alternative [0, -1.15, 0.40, 0, 0, 0]:")
    T = fk_complete(0, -1.15, 0.40, 0, 0, 0)
    print("Transformation Matrix:")
    print(T)
    print(f"\nPosition: x={T[0,3]:.4f}, y={T[1,3]:.4f}, z={T[2,3]:.4f}")
    print(f"Z-axis (gripper direction): [{T[0,2]:.4f}, {T[1,2]:.4f}, {T[2,2]:.4f}]")
    print(f"X-axis (gripper opening): [{T[0,0]:.4f}, {T[1,0]:.4f}, {T[2,0]:.4f}]")
    
    print("\n" + "="*70)
