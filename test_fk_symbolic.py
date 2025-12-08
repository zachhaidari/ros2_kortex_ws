from math import sin, cos, pi
import numpy as np

def fk_full(q1, q2, q3, q4, q5, q6):
    # Full rotation matrix elements
    r00 = (sin(q1)*sin(q4) - cos(q1)*cos(q4)*cos(q2 - q3))*sin(q5)*sin(q6) + (sin(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2 - q3))*cos(q6) - sin(q2 - q3)*sin(q6)*cos(q1)*cos(q5)
    r01 = -(sin(q1)*sin(q4) - cos(q1)*cos(q4)*cos(q2 - q3))*sin(q5)*cos(q6) + (sin(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2 - q3))*sin(q6) + sin(q2 - q3)*cos(q1)*cos(q5)*cos(q6)
    r02 = (sin(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2 - q3))*sin(q5) - sin(q2 - q3)*cos(q1)*cos(q5)
    r03 = -0.057*sin(q1)*sin(q4) + 0.235*sin(q1)*sin(q5)*cos(q4) + 0.01*sin(q1) - 0.28*sin(q2)*cos(q1) + 0.235*sin(q4)*sin(q5)*cos(q1)*cos(q2 - q3) - 0.235*sin(q2 - q3)*cos(q1)*cos(q5) - 0.245*sin(q2 - q3)*cos(q1) + 0.057*cos(q1)*cos(q4)*cos(q2 - q3)
    
    r10 = (sin(q1)*cos(q4)*cos(q2 - q3) - sin(q4)*cos(q1))*sin(q5)*sin(q6) + (sin(q1)*sin(q4)*cos(q2 - q3) - cos(q1)*cos(q4))*cos(q6) - sin(q1)*sin(q2 - q3)*sin(q6)*cos(q5)
    r11 = -(sin(q1)*cos(q4)*cos(q2 - q3) - sin(q4)*cos(q1))*sin(q5)*cos(q6) + (sin(q1)*sin(q4)*cos(q2 - q3) - cos(q1)*cos(q4))*sin(q6) + sin(q1)*sin(q2 - q3)*cos(q5)*cos(q6)
    r12 = (sin(q1)*sin(q4)*cos(q2 - q3) - cos(q1)*cos(q4))*sin(q5) - sin(q1)*sin(q2 - q3)*cos(q5)
    r13 = -0.28*sin(q1)*sin(q2) + 0.235*sin(q1)*sin(q4)*sin(q5)*cos(q2 - q3) - 0.235*sin(q1)*sin(q2 - q3)*cos(q5) - 0.245*sin(q1)*sin(q2 - q3) + 0.057*sin(q1)*cos(q4)*cos(q2 - q3) + 0.057*sin(q4)*cos(q1) - 0.235*sin(q5)*cos(q1)*cos(q4) - 0.01*cos(q1)
    
    r20 = -sin(q4)*sin(q5)*sin(q6)*sin(q2 - q3) + sin(q4)*cos(q6)*sin(q2 - q3) - sin(q6)*cos(q4)*cos(q2 - q3)*cos(q5)
    r21 = sin(q4)*sin(q5)*sin(q2 - q3)*cos(q6) + sin(q4)*sin(q6)*sin(q2 - q3) + cos(q4)*cos(q5)*cos(q6)*cos(q2 - q3)
    r22 = sin(q4)*sin(q5)*sin(q2 - q3) + cos(q5)*cos(q2 - q3)
    r23 = 0.235*sin(q4)*sin(q5)*sin(q2 - q3) + 0.057*sin(q2 - q3)*cos(q4) + 0.28*cos(q2) + 0.235*cos(q5)*cos(q2 - q3) + 0.245*cos(q2 - q3) + 0.2433
    
    T = np.array([
        [r00, r01, r02, r03],
        [r10, r11, r12, r13],
        [r20, r21, r22, r23],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    return T


print("=== FK Results ===\n")

print("1. All zeros [0, 0, 0, 0, 0, 0]:")
T = fk_full(0, 0, 0, 0, 0, 0)
print("Transformation Matrix:")
print(T)
print(f"\nPosition: x={T[0,3]:.4f}, y={T[1,3]:.4f}, z={T[2,3]:.4f}")
print(f"Z-axis (gripper direction): [{T[0,2]:.4f}, {T[1,2]:.4f}, {T[2,2]:.4f}]")
print(f"-> Gripper pointing: {'UP' if T[2,2] > 0.9 else 'DOWN' if T[2,2] < -0.9 else 'HORIZONTAL'}")

print("\n" + "="*60)
print("\n2. The position I suggested [0.0, 1.0, 2.05, 1.615, 0.55, 0.0]:")
T = fk_full(0.0, 1.0, 2.05, 1.615, 0.55, 0.0)
print("Transformation Matrix:")
print(T)
print(f"\nPosition: x={T[0,3]:.4f}, y={T[1,3]:.4f}, z={T[2,3]:.4f}")
print(f"Z-axis (gripper direction): [{T[0,2]:.4f}, {T[1,2]:.4f}, {T[2,2]:.4f}]")
print(f"-> Gripper pointing: {'UP' if T[2,2] > 0.9 else 'DOWN' if T[2,2] < -0.9 else 'HOME'}")

