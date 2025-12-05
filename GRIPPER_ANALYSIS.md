# Gripper TF and EE Link Dimension Analysis

## Transform Tree Structure
```
end_effector_link (joint_6 output)
  ↓ (fixed joint, 0 offset)
dummy_link  
  ├→ tool_frame (0 0 0.130) - 130mm above
  └→ gripper_base_link (0 0 0)
      ├→ right_finger_prox_link (0 -0.0305 0.070003) - 70mm above gripper_base
      └→ left_finger_prox_link (0 +0.0305 0.070003) - 70mm above gripper_base
```

## Gripper Geometry
- **Finger gap when attached to gripper_base**: ±30.5mm from center = **61mm total separation**
- **Finger proximal joint height above gripper_base**: **70mm (0.070003m)**
- **Red cube dimensions**: 50mm × 50mm × 50mm (collision = visual in this case)

## Frame Conversion (base_link frame)
- Robot base_link is at world z = 0.365m (offset above table)
- Table top at world z = 0.35m → base_link z = -0.015m
- Red cube center at world z = 0.375m → **base_link z = 0.01m**
- Red cube collision extends: z = [0.01 - 0.025, 0.01 + 0.025] = **[−0.015m, +0.035m]**
- **Red cube top surface at base_link z ≈ 0.035m**

## Current Grasp Poses (base_link frame, end_effector_link)
- **Red cube grasp**: z = 0.18m
  - Implied fingertip height: 0.18 - 0.070 = **0.11m** ← **76mm above cube top!**
  - Gripper **cannot close** on object at this height
  
- **Other objects grasp**: z = 0.085m
  - Implied fingertip height: 0.085 - 0.070 = **0.015m** ← **20mm above cube top**
  - More reasonable but may still be too high for solid grip

## Correct Grasp Calculation
For end_effector_link to position fingertips at object top:
- Object top at base_link z ≈ 0.035m
- Fingertips need to be slightly above top for approach: z ≈ 0.04-0.05m
- Required end_effector_link z = 0.04 + 0.070 = **0.11m** (minimum to contact top)
- For grasping center of 50mm cube (z=0.01): 
  - end_effector_link z = 0.01 + 0.07 + 0.025 = **0.105m** (grasp center)

## Recommendation
Update red_cube grasp pose from z=0.18 to z=0.10-0.11m to allow fingers to actually close on the object. The 0.18 value places the gripper opening too high above the object.
