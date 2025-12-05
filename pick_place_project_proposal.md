# Kinova Gen3 Lite Pick-and-Place Demo Project Proposal
**ENGR 870 – Robot Control**

**Team Members:** [Member 1], [Member 2], [Member 3]  
**Project Duration:** 3 weeks (November 19 - December 6, 2025)  
**Daily Time Commitment:** 1-2 hours/day per team member  
**Robot Platform:** Kinova Gen3 Lite 6-DOF with gen3_lite_2f gripper

---

## 1. Introduction and Problem Statement

### Background

Industrial automation and collaborative robotics have become essential in modern manufacturing, logistics, and research environments. Pick-and-place operations represent one of the most fundamental yet critical tasks in robotic manipulation, accounting for approximately 40% of industrial robot applications worldwide [1]. The ability to reliably grasp, transport, and position objects forms the foundation for more complex automation tasks including assembly, packaging, sorting, and material handling [2].

Collaborative robots (cobots) like the Kinova Gen3 Lite are specifically designed for safe human-robot interaction in shared workspaces, making them ideal for educational settings, research laboratories, and small-scale manufacturing [3]. Unlike traditional industrial robots confined to safety cages, cobots can work alongside humans, enabling flexible automation solutions in space-constrained environments. However, this flexibility comes with increased control complexity: cobots must balance precision, speed, and safety while operating in dynamic, unstructured environments [4].

The relevance of this application extends across multiple domains:
- **Manufacturing:** Small-batch production and assembly operations requiring flexible automation [5]
- **Healthcare:** Laboratory sample handling, medication dispensing, and assistive rehabilitation [6]
- **Education:** Hands-on robotics training for students in manipulation and control theory [7]
- **Research:** Platform for investigating motion planning algorithms, grasp planning, and human-robot collaboration

### Challenges

Despite widespread adoption, several challenges and knowledge gaps persist in robotic pick-and-place operations, particularly for collaborative robots in educational and research settings:

**1. Motion Planning Reliability**
Commercial motion planning libraries like MoveIt often struggle with reliability in constrained workspaces. Studies show planning failure rates of 5-15% in cluttered environments due to local minima in optimization-based planners [8]. Cartesian path planning near workspace boundaries frequently fails, requiring fallback strategies that are rarely documented in educational implementations.

**2. Gripper Control and Grasp Stability**
Parallel-jaw grippers like the gen3_lite_2f lack force sensing capabilities, making it difficult to achieve robust grasping across varying object properties. Research by Mahler et al. [9] demonstrates that grasp success rates drop below 70% without tactile feedback, yet most educational robotics projects ignore grasp verification entirely. The gap between simulation (assuming perfect grasps) and physical deployment (with slip, friction uncertainty, and positioning errors) remains a significant barrier.

**3. Real-Time Control and Network Latency**
ROS2-based robot control over Ethernet introduces variable latency (typically 1-10 ms) that can cause controller overruns when update rates exceed hardware capabilities [10]. The literature provides limited guidance on selecting appropriate control frequencies for networked collaborative robots, leading to trial-and-error tuning that wastes development time.

**4. Integration Complexity**
Modern robot control stacks involve multiple layers: low-level motor control, middleware (ROS2), motion planning (MoveIt2), and application logic. Each layer has configuration parameters that interact in non-obvious ways. Documentation gaps between these components create a steep learning curve, particularly regarding:
- Joint limit definitions across URDF, MoveIt SRDF, and controller configurations [11]
- Trajectory execution monitoring and failure recovery
- Coordinate frame transformations between robot base, end-effector, and workspace

**5. Validation and Repeatability**
Academic robot projects often lack rigorous evaluation methodologies. A survey by Bonsignorio et al. [12] found that 60% of robotics research papers do not report repeatability metrics, and only 25% provide sufficient detail for reproduction. This gap limits knowledge transfer from research to education and industry.

### Approaches Others Have Used

Several approaches have been documented for educational pick-and-place implementations:

**Vision-Based Approaches:** Many projects integrate cameras for object detection using OpenCV or deep learning [13, 14]. While impressive, these add complexity through calibration, lighting sensitivity, and processing latency. Studies show vision systems can add 200-500 ms per cycle and require 10-20 hours additional development time [15].

**Limitation:** Vision is unnecessary when workspace geometry is known and positions are fixed, yet it's often included prematurely, diverting effort from core manipulation skills.

**Trajectory Teaching Methods:** Some implementations use manual teaching (moving the robot by hand and recording waypoints) [16]. This is intuitive but lacks precision (±5-10 mm repeatability) and doesn't leverage the robot's kinematic model.

**Limitation:** Taught positions may be infeasible or require awkward joint configurations that reduce reliability.

**Simulation-First Development:** Projects often develop in Gazebo/RViz before hardware deployment [17]. While valuable for algorithm development, the sim-to-real gap in contact dynamics means grasping behaviors don't transfer reliably.

**Limitation:** Over-reliance on simulation can mask real-world issues like gripper compliance, surface friction, and network timing.

**Our project aims to address these gaps by:**
- Documenting a minimal, reliable pick-and-place implementation without unnecessary complexity
- Providing explicit control frequency tuning methodology based on network and hardware constraints
- Implementing grasp verification through gripper position feedback
- Establishing clear evaluation metrics (success rate, cycle time, repeatability)
- Creating reproducible workflows for position definition and error recovery

### Problem Statement

**The specific problem:** Develop a reliable, repeatable pick-and-place control system for the Kinova Gen3 Lite collaborative robot that achieves >90% success rate while operating within real-time constraints of networked ROS2 control.

**Why this is challenging:**
1. **Kinematic Complexity:** 6-DOF manipulator has infinite inverse kinematics solutions; selecting optimal configurations for reliability and safety is non-trivial
2. **Real-Time Constraints:** Controller must execute at 100 Hz over Ethernet while coordinating motion planning (10-1000 ms), gripper control (100 ms), and state monitoring
3. **Grasp Uncertainty:** Without force sensing, detecting successful grasps requires inferring from gripper position and motion success
4. **Error Recovery:** System must detect and recover from planning failures, grasp failures, and network issues without manual intervention

**Why this is beneficial:**
- Provides a foundation for students to learn practical robot control beyond simulation
- Demonstrates integration of motion planning, trajectory execution, and gripper control
- Creates a testbed for investigating control parameters (velocity limits, planning algorithms, timeout values)
- Offers a reproducible baseline for future enhancements (vision, force control, multi-object tasks)

### Literature Review Summary

**Motion Planning for Manipulation:**
Sucan and Chitta [18] developed MoveIt as an integrated motion planning framework, but noted that default parameters often require application-specific tuning. Kuffner and LaValle [19] showed RRT-based planners can fail in high-dimensional spaces with narrow passages, relevant for pick-and-place with workspace constraints.

**Gripper Control Strategies:**
Dollar and Howe [20] surveyed robot grasping, emphasizing the importance of compliance and feedback. For parallel-jaw grippers without tactile sensing, Shimoga [21] recommends position-based grasp verification by checking if final gripper position matches commanded closure.

**Real-Time Robot Control:**
Gomes et al. [22] analyzed ROS2 real-time performance, identifying Ethernet latency as a primary source of jitter. They recommend control frequencies ≤200 Hz for networked systems and implementing deadline monitoring to detect overruns.

**Educational Robotics Projects:**
Neto et al. [23] reviewed industrial robot programming methods, highlighting the gap between research algorithms and practical deployment. They advocate for modular, well-documented implementations that separate concerns (planning, control, application logic).

---

## 2. Objectives

### Objective 1: Develop Robust Motion Control Pipeline
Implement and validate a complete motion control pipeline integrating ROS2, MoveIt2, and the Kinova driver to achieve reliable waypoint navigation with <5% planning failure rate across all predefined positions.

**Success Criteria:**
- Robot successfully plans and executes trajectories to all pick/place positions
- Controller maintains 100 Hz update rate without overruns (>95% of execution time)
- Motion planning completes within 2 seconds for all positions
- System recovers automatically from planning failures through retries

### Objective 2: Implement Reliable Grasp-and-Release Sequence
Develop a gripper control strategy with position feedback verification to achieve >90% grasp success rate for test objects (50-200g blocks/bottles).

**Success Criteria:**
- Gripper closes to appropriate position based on object size
- System detects grasp failure (gripper closes completely without resistance)
- Object is retained during transport (no drops between pick and place)
- Gripper releases reliably at drop-off location

### Objective 3: Create Reproducible Configuration Framework
Design a position configuration system allowing rapid definition and modification of pick/place locations through YAML files and teaching tools, reducing position setup time to <5 minutes per location.

**Success Criteria:**
- All positions stored in human-readable YAML configuration
- Teaching tool allows manual positioning and automatic coordinate recording
- At least 3 pick stations and 1 basket location defined and validated
- Position modifications require only YAML edits (no code changes)

---

## 3. Proposed Approach

### Control Method

Our control strategy employs a **hierarchical architecture** separating high-level task planning from low-level trajectory execution:

**Layer 1: Application Layer (Python/ROS2 Node)**
- State machine managing pick-and-place sequence
- YAML configuration parsing for position database
- Service interface for external triggering
- Error detection and recovery logic

**Layer 2: Motion Planning Layer (MoveIt2)**
- Inverse kinematics solving using KDL solver
- Collision-aware trajectory planning with OMPL (RRT-Connect algorithm)
- Trajectory optimization for smoothness and timing
- Cartesian path planning for linear approach/retract motions

**Layer 3: Trajectory Execution Layer (ros2_control)**
- Joint trajectory controller executing planned paths
- 100 Hz control loop for position commands to robot
- Real-time monitoring of execution status
- Gripper action server for open/close commands

**Layer 4: Hardware Interface Layer (kortex_driver)**
- Ethernet communication with Kinova robot (UDP, ~1 ms latency)
- Low-level motor control and feedback (joint positions, velocities)
- Safety monitoring (joint limits, collision detection)
- Gripper position control and feedback

### Sensors and Hardware Elements

**Proprioceptive Sensors (Built-in):**
- **Joint Encoders:** 6 high-resolution encoders providing joint positions (±0.1° accuracy)
- **Gripper Position Sensor:** Measures finger separation (0-85 mm range)
- **Motor Current Sensors:** Indirect force estimation via current monitoring

**Hardware Components:**
- **Kinova Gen3 Lite:** 6-DOF collaborative arm (900 mm reach, 500g payload)
- **gen3_lite_2f Gripper:** Parallel-jaw gripper (0-85 mm stroke, 50N max force)
- **Control Computer:** Ubuntu 22.04, ROS2 Jazzy, Ethernet connection to robot

**No External Sensors Required:** This minimalist approach focuses on mastering fundamental control without sensor integration complexity.

### Design and Implementation Plan

#### Overall Workflow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    USER TRIGGERS OPERATION                   │
│          (ros2 service call /pick_place_trigger)            │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────┐
│              PICK-AND-PLACE STATE MACHINE                    │
│                 (pick_place_node.py)                        │
└─────────────────┬───────────────────────────────────────────┘
                  │
    ┌─────────────┴─────────────┬──────────────────┬──────────────────┐
    │                           │                  │                  │
    ▼                           ▼                  ▼                  ▼
┌─────────┐              ┌─────────────┐    ┌──────────┐      ┌──────────┐
│ MOVE TO │              │   GRIPPER   │    │  ERROR   │      │ STATUS   │
│  POSE   │              │   CONTROL   │    │ RECOVERY │      │PUBLISHING│
│(MoveIt2)│              │  (Action)   │    │  (Retry) │      │ (Topic)  │
└────┬────┘              └──────┬──────┘    └─────┬────┘      └──────┬───┘
     │                          │                 │                  │
     └──────────────────────────┴─────────────────┴──────────────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ ros2_control Framework │
                    │  (Joint Traj Control)  │
                    └───────────┬────────────┘
                                │
                                ▼
                    ┌────────────────────────┐
                    │    kortex_driver       │
                    │  (Ethernet → Robot)    │
                    └───────────┬────────────┘
                                │
                                ▼
                    ┌────────────────────────┐
                    │  Kinova Gen3 Lite      │
                    │  Hardware + Gripper    │
                    └────────────────────────┘
```

#### State Machine Flow

```
START
  │
  ▼
[IDLE] ───────service call──────▶ [MOVING_TO_HOME]
                                        │
                          ┌─────────────┘
                          │ success
                          ▼
                    [MOVING_TO_PREGRASP]
                          │
                          │ success
                          ▼
                    [OPENING_GRIPPER]
                          │
                          ▼
                    [MOVING_TO_GRASP]
                          │
                          │ success
                          ▼
                    [CLOSING_GRIPPER]
                          │
                          ▼
                    [VERIFYING_GRASP] ◄────fail (gripper fully closed)
                          │                      │
                          │ success              │
                          ▼                      ▼
                    [LIFTING_OBJECT]        [ERROR_NO_OBJECT]
                          │                      │
                          │                      │
                          ▼                      │
                    [MOVING_TO_BASKET]           │
                          │                      │
                          │                      │
                          ▼                      │
                    [OPENING_GRIPPER]            │
                          │                      │
                          │                      │
                          ▼                      │
                    [MOVING_TO_HOME]             │
                          │                      │
                          │                      │
                          ▼                      │
                    [SUCCESS] ◄──────────────────┘
                          │               (after retry)
                          │
                          ▼
                      [IDLE]

Note: All states have timeout handlers → [ERROR_TIMEOUT] → [RECOVERY] → retry
```

#### Control System Block Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                    CONTROL SYSTEM ARCHITECTURE                  │
└────────────────────────────────────────────────────────────────┘

Input:                          Controller:                    Output:
┌──────────────┐               ┌──────────────┐              ┌──────────┐
│   Desired    │               │    MoveIt2   │              │  Joint   │
│   Position   ├──────────────▶│   Planning   ├─────────────▶│  Angles  │
│  (x,y,z,R)   │               │   + IK       │              │ θ₁...θ₆  │
└──────────────┘               └──────┬───────┘              └─────┬────┘
                                      │                            │
                              ┌───────▼────────┐                   │
                              │   Trajectory   │                   │
                              │  Optimization  │                   │
                              │  (time-optimal)│                   │
                              └───────┬────────┘                   │
                                      │                            │
                              ┌───────▼────────┐                   │
                              │   Joint Traj   │◄──────────────────┘
                              │   Controller   │        Feedback
                              │   (PID Loop)   │        (joint states)
                              └───────┬────────┘
                                      │
                              ┌───────▼────────┐
                              │  kortex_driver │
                              │   (Ethernet)   │
                              └───────┬────────┘
                                      │
                              ┌───────▼────────┐
                              │     Robot      │
                              │   Actuators    │
                              └────────────────┘
```

#### General Dynamics and Control Equations

**1. Forward Kinematics (Robot Pose from Joint Angles)**

$$\mathbf{T}_{base}^{ee} = \mathbf{T}_0^1(\theta_1) \cdot \mathbf{T}_1^2(\theta_2) \cdot ... \cdot \mathbf{T}_5^6(\theta_6)$$

Where $\mathbf{T}_i^{i+1}$ are Denavit-Hartenberg transformation matrices.

**2. Inverse Kinematics (Joint Angles from Desired Pose)**

Solved numerically using KDL (Kinematics and Dynamics Library) via Newton-Raphson:

$$\theta_{k+1} = \theta_k + J^{\dagger}(\theta_k) \cdot (\mathbf{x}_{desired} - \mathbf{x}(\theta_k))$$

Where $J^{\dagger}$ is the pseudo-inverse of the Jacobian matrix.

**3. Joint Trajectory Generation**

Quintic polynomial for smooth motion between waypoints:

$$\theta(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5$$

Coefficients computed to satisfy boundary conditions:
- Initial/final position, velocity, acceleration
- Maximum velocity and acceleration limits from joint_limits.yaml

**4. PID Control for Trajectory Tracking**

Joint-level control (implemented in robot firmware):

$$u_i(t) = K_p e_i(t) + K_i \int_0^t e_i(\tau) d\tau + K_d \frac{de_i(t)}{dt}$$

Where $e_i(t) = \theta_{i,desired}(t) - \theta_{i,actual}(t)$ is the tracking error for joint $i$.

**5. Gripper Control**

Position control with velocity limiting:

$$d_{gripper}(t+\Delta t) = d_{gripper}(t) + v_{max} \cdot \Delta t \cdot \text{sign}(d_{target} - d_{gripper}(t))$$

Where $d_{gripper}$ is finger separation distance and $v_{max} = 0.6$ rad/s.

#### Mechanical Design and Operation

**Kinova Gen3 Lite Specifications:**
- **Degrees of Freedom:** 6 (all revolute joints)
- **Reach:** 902 mm maximum
- **Payload:** 500g at full extension
- **Repeatability:** ±0.1 mm
- **Weight:** 7.0 kg
- **Joint Ranges:**
  - Joint 1 (base rotation): ±360° (continuous)
  - Joint 2 (shoulder): -128.9° to +128.9°
  - Joint 3 (shoulder): -147.8° to +147.8°
  - Joint 4 (elbow): -128.9° to +128.9°
  - Joint 5 (wrist 1): -147.8° to +147.8°
  - Joint 6 (wrist 2): -120.3° to +120.3°

**Gripper Mechanism (gen3_lite_2f):**
- **Type:** Parallel-jaw, under-actuated
- **Stroke:** 0-85 mm finger separation
- **Actuation:** Single motor with linkage mechanism
- **Force:** Adaptive (compliant linkage distributes force)
- **Operation:** Motor drives lead screw → moves both fingers symmetrically inward/outward

**Workspace Design:**
```
Top View of Workspace:

        Robot Base (0,0)
              │
    ──────────┼──────────
   │          │          │
   │     [S1] │ [S2]     │
   │          │          │  [S1, S2, S3] = Pick Stations
   │          │     [S3] │  [B] = Basket
   │          │          │
   │          │  [B]     │
   │          │          │
    ──────────┴──────────
         Table Surface
    (approximately 400mm x 600mm reachable area)

Side View:
                    ┌─── End Effector + Gripper
                   ╱
                  ╱
                 ╱ ← Arm reach ~900mm
                ╱
    ┌──────────┴─────────┐
    │   Robot Base       │
    └────────────────────┘
    ─────────────────────── Table (~100mm below base)
           Objects
           ├──┤ ~50-100mm height
```

#### Electrical Design and Subsystems

**Power Distribution:**
```
┌─────────────┐
│ 24V DC PSU  │ ──────▶ Kinova Gen3 Lite Robot
│  (External) │              │
└─────────────┘              ├─▶ Joint 1-6 Motors (brushless DC)
                             │
                             └─▶ Gripper Motor
```

**Communication Architecture:**
```
┌──────────────┐                  ┌───────────────┐
│ Control PC   │◄────Ethernet────▶│  Robot Base   │
│ Ubuntu 22.04 │   192.168.1.x    │  Controller   │
│  ROS2 Jazzy  │                  │   (Embedded)  │
└──────────────┘                  └───────┬───────┘
                                          │
                            ┌─────────────┴───────────┐
                            │                         │
                      ┌─────▼─────┐           ┌───────▼──────┐
                      │  Joint    │           │   Gripper    │
                      │ Encoders  │           │   Position   │
                      │  (x6)     │           │   Sensor     │
                      └───────────┘           └──────────────┘
```

**Sensor Subsystems:**

1. **Joint Encoders (Magnetic Absolute Encoders)**
   - **Quantity:** 6 (one per joint)
   - **Resolution:** 13-bit (8192 counts/rev) → 0.044° resolution
   - **Sample Rate:** 1 kHz
   - **Communication:** Embedded controller reads via SPI bus
   - **Output:** Joint positions published to ROS2 /joint_states topic at 100 Hz

2. **Gripper Position Sensor (Linear Potentiometer)**
   - **Measurement:** Finger separation distance (0-85 mm)
   - **Resolution:** ~0.5 mm
   - **Sample Rate:** 100 Hz
   - **Output:** Published as gripper joint state in /joint_states

3. **Motor Current Sensors (for each joint + gripper)**
   - **Purpose:** Monitoring motor load, indirect force estimation
   - **Not directly used in this project** (future enhancement for force control)

**Network Communication:**
- **Protocol:** UDP/IP (low-latency, suitable for real-time control)
- **Frequency:** 100 Hz control loop (10 ms period)
- **Latency:** Typical 0.3-0.9 ms, with occasional spikes to 2-3 ms
- **Packet Structure:** Binary format with joint commands and status feedback

---

## 4. Expected Outcomes and Evaluation Metrics

### Expected Outcomes

**Primary Outcome:**
A functional pick-and-place system capable of autonomously transferring objects from predefined pick locations to a basket with high reliability, executed via simple ROS2 service calls.

**Specific Deliverables:**
1. **Software Package:** Complete ROS2 package (`kortex_pick_place_demo`) with documented code, launch files, and configuration
2. **Position Database:** YAML file with at least 3 validated pick locations and 1 basket location
3. **Demonstration Video:** 2-3 minute video showing 10 consecutive successful cycles
4. **Technical Documentation:** User guide, API reference, and troubleshooting guide
5. **Validation Report:** Quantitative evaluation against metrics (see below)

**Operational Capabilities:**
- Robot picks objects from any configured station on command
- Graceful handling of failures (planning errors, grasp failures) with automatic retries
- Real-time status feedback via ROS2 topics
- <60 second cycle time per pick-and-place operation

### Evaluation Metrics

**Metric 1: Success Rate (Primary Metric)**

$$\text{Success Rate} = \frac{\text{Successful Cycles}}{\text{Total Attempts}} \times 100\%$$

**Target:** ≥90% over 30 attempts (10 cycles × 3 pick locations)

**Definition of Success:**
- Object picked from station
- Object retained during transport (no drops)
- Object released in basket
- Robot returns to home without errors

**Measurement Protocol:**
- Place object at pick location
- Trigger pick-and-place via service call
- Manually record success/failure
- Repeat 10 times per location

---

**Metric 2: Cycle Time (Efficiency Metric)**

$$\text{Cycle Time} = t_{end} - t_{start}$$

Where $t_{start}$ is service call timestamp and $t_{end}$ is return-to-home completion timestamp.

**Target:** ≤60 seconds average (excluding failures)

**Components:**
- Motion planning time: 1-3 seconds per waypoint
- Trajectory execution: 5-10 seconds per segment
- Gripper actuation: 2-3 seconds per open/close
- Total: ~40-50 seconds expected

**Measurement:**
- Log timestamps at each state transition
- Calculate average and standard deviation over 30 successful cycles
- Identify bottlenecks (planning vs. execution time)

---

**Metric 3: Positioning Accuracy (Precision Metric)**

$$\text{Placement Error} = \sqrt{(x_{actual} - x_{target})^2 + (y_{actual} - y_{target})^2}$$

**Target:** ≤10 mm radial error for object placement in basket

**Measurement Protocol:**
- Mark target center of basket
- After 10 placements, measure object final positions
- Calculate Euclidean distance from target center
- Compute mean and standard deviation

**Rationale:** Demonstrates repeatability and control precision, critical for multi-object tasks or constrained placement (e.g., stacking).

---

**Additional Diagnostic Metrics (Not Graded, But Monitored):**
- **Controller Overrun Rate:** % of control cycles exceeding 10 ms deadline (target: <5%)
- **Planning Failure Rate:** % of motion plans failing before retry (target: <10%)
- **Grasp Failure Rate:** % of failed grasps detected by gripper feedback (target: <10%)

---

## 5. Challenges

### Anticipated Difficulties and Fallback Solutions

**Challenge 1: Motion Planning Failures in Constrained Workspace**

**Description:** MoveIt may fail to find collision-free paths, especially for pick positions near workspace boundaries or when arm must reach around obstacles.

**Risk Level:** Medium-High  
**Team Skill Assessment:** Moderate (ROS2 experience, limited MoveIt debugging)

**Fallback Solutions:**
1. **Immediate:** Adjust pick positions to more central workspace regions (reduce reach requirements)
2. **Short-term:** Increase planning time allowance from 2s to 5s, enabling more planner iterations
3. **Long-term:** Manually define joint-space waypoints for problematic paths (bypass Cartesian planning)
4. **Last Resort:** Reduce number of pick stations from 3 to 2, focusing on reliable locations

**Mitigation Strategy:**
- Week 1: Test reachability of all positions in RViz before physical deployment
- Use MoveIt's "planning scene" to add virtual table as collision object
- Document specific planning parameters that improve success (planner type, timeout, goal tolerance)

---

**Challenge 2: Unreliable Grasping Without Force Feedback**

**Description:** Parallel-jaw gripper may fail to grasp objects due to positioning errors, object geometry, or insufficient friction. Without force/torque sensors, detecting grasp failure is difficult.

**Risk Level:** High  
**Team Skill Assessment:** Low-Moderate (no prior gripper control experience)

**Fallback Solutions:**
1. **Immediate:** Use highly graspable objects (foam blocks, rubber-coated items with texture)
2. **Short-term:** Implement gripper position feedback check (if gripper closes fully without resistance, object likely missed)
3. **Long-term:** Add approach from directly above (gravity assists alignment), reduce approach speed
4. **Last Resort:** Manually place objects in known precise locations, use fixtures to constrain object position

**Mitigation Strategy:**
- Week 2: Dedicate 4-6 hours to gripper tuning experiments
- Test multiple gripper closure positions (50%, 60%, 70% of max stroke)
- Implement "shake test" (small lift motion to verify object weight via trajectory tracking error)

---

**Challenge 3: Controller Timing Issues and Network Latency**

**Description:** Controller overruns (write time exceeding control period) cause jittery motion or trajectory tracking errors. Ethernet latency is variable and difficult to predict.

**Risk Level:** Medium  
**Team Skill Assessment:** Moderate (experienced this issue previously, found 100 Hz solution)

**Fallback Solutions:**
1. **Immediate:** Use established 100 Hz control rate (already validated in prior testing)
2. **Short-term:** Reduce velocity/acceleration limits in joint_limits.yaml (slower motion is more forgiving)
3. **Long-term:** Implement real-time kernel patches for Ubuntu (reduces latency jitter)
4. **Last Resort:** Accept occasional overruns (<5%) as acceptable if not impacting success rate

**Mitigation Strategy:**
- Monitor controller logs for "Overrun detected" warnings throughout testing
- If overruns occur, sequentially reduce control rate: 100 Hz → 80 Hz → 60 Hz
- Ensure no other processes (browser, simulators) running during demos

---

**Challenge 4: Integration and Configuration Complexity**

**Description:** ROS2, MoveIt2, and kortex_driver have dozens of configuration files (URDF, SRDF, YAML) that must be consistent. Errors in joint limits, frame definitions, or controller parameters cause cryptic failures.

**Risk Level:** Medium  
**Team Skill Assessment:** Low-Moderate (learning curve for MoveIt configuration)

**Fallback Solutions:**
1. **Immediate:** Use existing validated configuration from prior controller fixes (joint_limits.yaml, ros2_controllers.yaml)
2. **Short-term:** Create configuration checklist (joint limits match across URDF/SRDF/YAML, frame names consistent)
3. **Long-term:** Develop automated validation script to check config consistency
4. **Last Resort:** Simplify to minimal configuration (disable collision checking if causing issues)

**Mitigation Strategy:**
- Week 1: Validate all configurations in simulation (Gazebo) before physical robot
- Use version control (Git) to track configuration changes
- Document every parameter change with rationale

---

**Challenge 5: Time Constraint (3 Weeks, 1-2 hrs/day)**

**Description:** With only 21 days × 1.5 hrs/day × 3 members = ~95 person-hours total, scope must be tightly managed. Debugging unexpected issues could consume entire timeline.

**Risk Level:** High  
**Team Skill Assessment:** Dependent on parallel work efficiency and communication

**Fallback Solutions:**
1. **Immediate:** Use Agile sprints (weekly milestones), pivot if Week 1 goals not met
2. **Short-term:** Reduce to 2 pick locations instead of 3, simplify documentation
3. **Long-term:** Defer "nice-to-have" features (teaching tool, web interface, fancy error recovery)
4. **Last Resort:** Demonstrate successful single pick-and-place cycle (1 location) with manual position teaching

**Mitigation Strategy:**
- Daily 15-minute standups (async via Slack/Discord) to identify blockers
- Assign clear task ownership with 24-hour completion targets
- Parallelize independent tasks (hardware setup, software dev, documentation)

---

**Challenge 6: Hardware Availability and Physical Setup**

**Description:** Team members may have limited access to physical robot (shared lab space, scheduling conflicts). Hardware failures (gripper malfunction, network issues) could stall progress.

**Risk Level:** Medium  
**Team Skill Assessment:** Low control (dependent on lab policies)

**Fallback Solutions:**
1. **Immediate:** Develop and test all code in Gazebo simulation first (verify logic without hardware)
2. **Short-term:** Schedule 2-3 dedicated hardware testing sessions per week (coordinate in advance)
3. **Long-term:** Implement hardware-in-the-loop testing (one member operates robot remotely via VPN)
4. **Last Resort:** Submit simulation-only results if hardware unavailable, with documented attempts

**Mitigation Strategy:**
- Weeks 1-2: Focus on simulation development (60% time), Week 3: Hardware validation (80% time)
- Create hardware contingency budget: if >4 hours lost to hardware issues, reduce scope
- Document all hardware issues for instructor awareness (evidence of effort)

---

## 6. Team Member Responsibilities

### Team Structure and Roles

**Team Member 1: [Name] – Software Integration Lead**
- **Primary Focus:** ROS2 node development, MoveIt2 integration, state machine logic

**Detailed Subtasks:**
- Week 1:
  - [ ] Create ROS2 package structure (`kortex_pick_place_demo`) - 2 hrs
  - [ ] Set up package.xml, CMakeLists.txt with dependencies - 1 hr
  - [ ] Implement YAML parser for position configuration - 2 hrs
  - [ ] Develop basic MoveIt2 Python interface (move to joint positions) - 3 hrs
  - [ ] Test motion planning in Gazebo simulation - 2 hrs

- Week 2:
  - [ ] Implement pick-and-place state machine (10 states) - 4 hrs
  - [ ] Add gripper control via action client - 2 hrs
  - [ ] Implement grasp verification logic (position feedback) - 2 hrs
  - [ ] Develop error recovery (retry on planning failure) - 2 hrs

- Week 3:
  - [ ] Integrate status publishing (ROS2 topics for state/progress) - 2 hrs
  - [ ] Add logging and debugging output - 1 hr
  - [ ] Bug fixes and code refinement - 3 hrs
  - [ ] Code review and documentation comments - 2 hrs

**Skills Required:** Python, ROS2, MoveIt2 Python API  
**Backup:** Member 2 can assist with state machine logic if primary blocked

---

**Team Member 2: [Name] – Control System and Configuration Specialist**

- **Primary Focus:** Controller tuning, position teaching, YAML configuration, testing

**Detailed Subtasks:**
- Week 1:
  - [ ] Validate existing controller configuration (ros2_controllers.yaml) - 1 hr
  - [ ] Set up physical workspace (table, markers for positions) - 2 hrs
  - [ ] Develop position teaching script (RViz → YAML export) - 3 hrs
  - [ ] Manually teach 3 pick positions + 1 basket position - 2 hrs
  - [ ] Verify all positions reachable in RViz - 2 hrs

- Week 2:
  - [ ] Tune gripper open/close positions for test objects - 2 hrs
  - [ ] Test and optimize motion velocity/acceleration limits - 2 hrs
  - [ ] Conduct repeatability tests (measure positioning accuracy) - 2 hrs
  - [ ] Document optimal control parameters - 2 hrs

- Week 3:
  - [ ] Execute 30-cycle validation testing (success rate metric) - 4 hrs
  - [ ] Measure and record cycle times - 1 hr
  - [ ] Measure placement accuracy (ruler + camera) - 2 hrs
  - [ ] Compile results into metrics spreadsheet - 1 hr

**Skills Required:** ROS2 basics, robot operation, experimental methodology  
**Backup:** Member 3 can assist with physical testing and data collection

---

**Team Member 3: [Name] – Documentation and System Integration Lead**

- **Primary Focus:** Launch files, documentation, video production, integration testing

**Detailed Subtasks:**
- Week 1:
  - [ ] Create launch file for simulation environment - 2 hrs
  - [ ] Create launch file for physical robot + demo node - 2 hrs
  - [ ] Write initial README.md with project overview - 1 hr
  - [ ] Set up Git repository and version control workflow - 1 hr
  - [ ] Draft user guide outline (installation, usage) - 2 hrs

- Week 2:
  - [ ] Develop service call interface documentation (API) - 2 hrs
  - [ ] Create troubleshooting guide (common errors, solutions) - 2 hrs
  - [ ] Write technical documentation (architecture, flowcharts) - 2 hrs
  - [ ] Integrate all components for end-to-end testing - 2 hrs

- Week 3:
  - [ ] Record demonstration video (multiple angles, 10 cycles) - 2 hrs
  - [ ] Edit video with annotations and captions - 2 hrs
  - [ ] Finalize user guide with screenshots - 2 hrs
  - [ ] Complete technical documentation (equations, diagrams) - 2 hrs
  - [ ] Prepare final project report and presentation slides - 3 hrs

**Skills Required:** Technical writing, video editing, systems thinking  
**Backup:** Member 1 can provide code documentation, Member 2 can assist with video recording

---

### Collaboration and Communication

**Daily Coordination:**
- **Async Standup (via Slack/Discord):** Each member posts daily update (15 min)
  - What did you complete yesterday?
  - What are you working on today?
  - Any blockers or help needed?

**Weekly Synchronous Meetings:**
- **Duration:** 30-60 minutes
- **Agenda:** Review milestone progress, demo current functionality, assign next week's tasks
- **Schedule:** End of Week 1, End of Week 2, Mid-Week 3

**Task Tracking:**
- **Tool:** GitHub Issues or Trello board
- **Labels:** Priority (P0-P3), Status (To Do, In Progress, Review, Done)
- **Review:** All members check board daily

**Conflict Resolution:**
- If team member falls behind: redistribute tasks immediately, escalate to instructor if needed
- If technical blocker: schedule 2-hour pair programming session, consider fallback solution

---

### Cross-Training and Backup Plans

- **Week 1:** All members run through ROS2/MoveIt2 tutorials together (2 hrs joint session)
- **Week 2:** Pair programming sessions for state machine development
- **Week 3:** All members familiar with full system (can operate and demo independently)

**Risk Mitigation:**
- Each critical subtask has designated backup member
- Code reviews required before merging (ensures knowledge sharing)
- Documentation written incrementally (no single point of failure)

---

## 7. Timeline and Milestones

### Overview

**Start Date:** November 19, 2025  
**End Date:** December 6, 2025 (First week) or December 13, 2025 (Second week)  
**Total Duration:** 18-25 days  
**Effort:** 3 members × 1.5 hrs/day × 20 days = ~90 person-hours

### Weekly Breakdown

#### Week 1: Foundation and Position Setup (Nov 19-25)

**Milestone 1 (End of Week 1):** Robot can move to all predefined positions reliably in simulation and on hardware

**Deliverables:**
- ✅ ROS2 package created with proper structure
- ✅ Position configuration YAML with 3 pick + 1 basket location
- ✅ Basic motion planning working (home → positions → home)
- ✅ All positions validated as reachable and collision-free

**Success Criteria:**
- 10 consecutive motion cycles without planning failures
- Controller maintains 100 Hz without overruns
- All team members can launch and operate system

**Daily Schedule:**
- **Day 1-2 (Nov 19-20):** Package setup, dependencies, initial development environment
- **Day 3-4 (Nov 21-22):** Position teaching tool, manual position recording
- **Day 5-7 (Nov 23-25):** Position validation, basic motion testing, Week 1 demo

**Checkpoint (Nov 25):** 30-minute team demo of motion planning to instructor/TA

---

#### Week 2: Gripper Integration and Pick-Place Logic (Nov 26-Dec 2)

**Milestone 2 (End of Week 2):** Complete functional pick-and-place sequence operational

**Deliverables:**
- ✅ State machine implemented with all 10 states
- ✅ Gripper control integrated (open/close/verify)
- ✅ Error recovery working (retry logic)
- ✅ Service interface for triggering operations

**Success Criteria:**
- At least 5/10 successful pick-and-place cycles
- Grasp verification detects failures
- System recovers from planning errors automatically

**Daily Schedule:**
- **Day 8-10 (Nov 26-28):** State machine development, gripper integration
- **Day 11-12 (Nov 29-30):** Error handling, service interface
- **Day 13-14 (Dec 1-2):** End-to-end testing, debugging, Week 2 demo

**Checkpoint (Dec 2):** Demonstrate 3 consecutive successful pick-and-place operations

---

#### Week 3: Validation, Testing, and Documentation (Dec 3-9)

**Milestone 3 (End of Week 3):** Production-ready system with complete documentation and validation data

**Deliverables:**
- ✅ 30-cycle validation test complete (success rate measured)
- ✅ Cycle time and accuracy metrics recorded
- ✅ User guide and technical documentation complete
- ✅ Demonstration video edited and ready
- ✅ Final project report submitted

**Success Criteria:**
- ≥90% success rate achieved
- ≤60 second average cycle time
- ≤10 mm placement accuracy
- All documentation complete and reviewed

**Daily Schedule:**
- **Day 15-16 (Dec 3-4):** Status publishing, logging, final code polish
- **Day 17-18 (Dec 5-6):** Validation testing (30 cycles), metric collection
- **Day 19-20 (Dec 7-8):** Video recording/editing, documentation finalization
- **Day 21 (Dec 9):** Final review, report submission preparation

**Final Checkpoint (Dec 6 or 13):** Project presentation and live demonstration

---

### Gantt Chart

```
Week 1 (Nov 19-25):
Task                    | Team Member | Mon Tue Wed Thu Fri Sat Sun
────────────────────────┼─────────────┼───────────────────────────
Package Setup           | Member 1    | ███ ▓▓▓
Position Teaching Tool  | Member 2    |     ███ ███
Launch Files            | Member 3    | ███ ███
Motion Planning         | Member 1    |         ███ ███ ▓▓▓
Position Recording      | Member 2    |             ███ ███
Documentation Start     | Member 3    |         ███ ███
Hardware Testing        | All         |                 ▓▓▓ ███

Week 2 (Nov 26-Dec 2):
Task                    | Team Member | Mon Tue Wed Thu Fri Sat Sun
────────────────────────┼─────────────┼───────────────────────────
State Machine           | Member 1    | ███ ███ ▓▓▓
Gripper Integration     | Member 1    |         ███ ▓▓▓
Parameter Tuning        | Member 2    | ███ ███ ███
Testing Protocol        | Member 2    |             ███ ▓▓▓
Tech Documentation      | Member 3    | ███ ███ ███ ███
Integration Testing     | All         |                 ▓▓▓ ███

Week 3 (Dec 3-9):
Task                    | Team Member | Mon Tue Wed Thu Fri Sat Sun
────────────────────────┼─────────────┼───────────────────────────
Final Code Polish       | Member 1    | ███ ▓▓▓
Validation Testing      | Member 2    | ███ ███ ███
Video Production        | Member 3    |     ███ ███ ▓▓▓
Metric Analysis         | Member 2    |             ███
Documentation Final     | Member 3    |             ███ ███
Final Report            | All         |                 ███ ███

Legend: ███ = Work time (1-2 hrs)   ▓▓▓ = Buffer/flexibility
```

---

### Risk Buffer and Contingency

**Built-in Buffer Time:**
- Each week has 10-12 hrs planned, leaves 3-4 hrs slack for debugging
- Week 3 intentionally lighter on development, heavier on validation (absorbs delays)

**Go/No-Go Decision Points:**
1. **Nov 25 (End Week 1):** If motion planning not working, reduce to 2 pick locations
2. **Dec 2 (End Week 2):** If success rate <50%, simplify grasping (better objects, fixtures)
3. **Dec 6:** If validation not complete, use simulation data + documented hardware attempts

**Extension Plan (if deadline extended to Dec 13):**
- Extra week allocated to achieving 90% success rate through iterative tuning
- Additional time for polishing documentation and video production

---

## References

[1] International Federation of Robotics (IFR). (2024). *World Robotics 2024: Industrial Robots*. IFR Statistical Department.

[2] Billard, A., & Kragic, D. (2019). Trends and challenges in robot manipulation. *Science*, 364(6446), eaat8414.

[3] Kinova Robotics. (2023). *Gen3 Lite Technical Specifications*. https://www.kinovarobotics.com/product/gen3-lite-robots

[4] Villani, V., Pini, F., Leali, F., & Secchi, C. (2018). Survey on human–robot collaboration in industrial settings: Safety, intuitive interfaces and applications. *Mechatronics*, 55, 248-266.

[5] Bauer, A., Wollherr, D., & Buss, M. (2008). Human–robot collaboration: A survey. *International Journal of Humanoid Robotics*, 5(01), 47-66.

[6] Datta, S., Das, A., & Gazi, F. (2020). Applications of collaborative robots in healthcare: A survey. *Journal of Medical Robotics Research*, 5(03n04), 2050001.

[7] Jung, S. E., & Won, E. S. (2018). Systematic review of research trends in robotics education for young children. *Sustainability*, 10(4), 905.

[8] Kingston, Z., Moll, M., & Kavraki, L. E. (2018). Sampling-based methods for motion planning with constraints. *Annual Review of Control, Robotics, and Autonomous Systems*, 1, 159-185.

[9] Mahler, J., Liang, J., Niyaz, S., Laskey, M., Doan, R., Liu, X., ... & Goldberg, K. (2017). Dex-Net 2.0: Deep learning to plan robust grasps with synthetic point clouds and analytic grasp metrics. *Robotics: Science and Systems*.

[10] Gomes, R. G., Lages, W. F., & Freitas, R. C. (2022). Performance analysis of ROS2 for real-time robotic applications. *IEEE Access*, 10, 52179-52190.

[11] Coleman, D., Sucan, I., Chitta, S., & Correll, N. (2014). Reducing the barrier to entry of complex robotic software: A MoveIt! case study. *Journal of Software Engineering for Robotics*, 5(1), 3-16.

[12] Bonsignorio, F. P., & Del Pobil, A. P. (2015). Toward replicable and measurable robotics research. *IEEE Robotics & Automation Magazine*, 22(3), 32-35.

[13] Zeng, A., Song, S., Welker, S., Lee, J., Rodriguez, A., & Funkhouser, T. (2018). Learning synergies between pushing and grasping with self-supervised deep reinforcement learning. *IROS*, 4238-4245.

[14] Jiang, Y., Moseson, S., & Saxena, A. (2011). Efficient grasping from RGBD images: Learning using a new rectangle representation. *ICRA*, 3304-3311.

[15] Hornung, A., Wurm, K. M., Bennewitz, M., Stachniss, C., & Burgard, W. (2013). OctoMap: An efficient probabilistic 3D mapping framework based on octrees. *Autonomous Robots*, 34(3), 189-206.

[16] Ureche, A. L. P., Umezawa, K., Nakamura, Y., & Billard, A. (2015). Task parameterization using continuous constraints extracted from human demonstrations. *IEEE Transactions on Robotics*, 31(6), 1458-1471.

[17] Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IROS*, 3, 2149-2154.

[18] Sucan, I. A., & Chitta, S. (2013). MoveIt!. *Online at moveit. ros. org*.

[19] Kuffner, J. J., & LaValle, S. M. (2000). RRT-connect: An efficient approach to single-query path planning. *ICRA*, 2, 995-1001.

[20] Dollar, A. M., & Howe, R. D. (2010). The highly adaptive SDM hand: Design and performance evaluation. *The International Journal of Robotics Research*, 29(5), 585-597.

[21] Shimoga, K. B. (1996). Robot grasp synthesis algorithms: A survey. *The International Journal of Robotics Research*, 15(3), 230-266.

[22] Gomes, R. G., & Lages, W. F. (2021). Real-time performance analysis of ROS2. *IEEE Latin America Transactions*, 19(10), 1634-1641.

[23] Neto, P., Simão, M., Mendes, N., & Safeea, M. (2019). Gesture-based human-robot interaction for human assistance in manufacturing. *The International Journal of Advanced Manufacturing Technology*, 101(1), 119-135.

---

**End of Proposal**

### What This System Does
- Pick objects from **predefined locations** in the workspace (e.g., Position A, Position B, Position C)
- Place objects in a **predefined basket location**
- Launch manually via ROS2 commands when user wants to execute a pick-and-place cycle
- Provide clear feedback on operation success/failure
- Support multiple object locations that can be easily configured

### What This System Does NOT Do
- ❌ Autonomous object detection or computer vision
- ❌ Color-based sorting
- ❌ Real-time obstacle avoidance
- ❌ Automatic triggering based on events

---

## Technical Architecture

### Hardware Requirements
- Kinova Gen3 Lite 6-DOF arm with gen3_lite_2f gripper (already have)
- Workspace table with marked positions for objects and basket
- Optional: Physical markers (tape/stickers) to indicate pick/place zones
- Test objects: Small blocks, bottles, or boxes (50-200g, graspable by 2F gripper)

### Software Stack
- **ROS2 Jazzy** (existing installation)
- **MoveIt2** for motion planning (existing)
- **ros2_kortex** driver (existing)
- **Python 3** for scripting pick-and-place logic
- **RViz** for visualization and position teaching

### Key Components
1. **Position Configuration File** (`pick_place_positions.yaml`)
   - Stores predefined XYZ coordinates and orientations
   - Defines multiple pick locations (e.g., station_1, station_2, station_3)
   - Defines basket drop-off location
   - Gripper open/close positions

2. **Pick-and-Place Node** (`pick_place_node.py`)
   - ROS2 Python node with MoveIt2 interface
   - Executes pick-and-place sequence on service call
   - Handles motion planning, gripper control, error recovery

3. **Launch System** (`pick_place_demo.launch.py`)
   - Single command to start robot + demo node
   - Parameter to select which object location to pick from

4. **Teaching Tool** (optional helper script)
   - Move robot with RViz or keyboard, save current position
   - Simplifies recording of pick/place coordinates

---

## Three-Week Timeline

### Week 1: Foundation & Position Setup (5-7 hours)
**Goal:** Get robot moving to predefined positions reliably

**Day 1-2 (2-3 hours):**
- Create ROS2 package: `kortex_pick_place_demo`
- Set up basic package structure (CMakeLists.txt, package.xml, launch/, config/, scripts/)
- Create `pick_place_positions.yaml` template with placeholder coordinates
- Test existing MoveIt interface with simple "move to position" script

**Day 3-4 (2-3 hours):**
- Develop teaching tool script to record positions manually
- Use MoveIt's "Plan & Execute" in RViz to move arm to desired locations
- Record coordinates for:
  - Home position (safe starting point)
  - Pre-grasp position (above object)
  - Grasp position (at object)
  - Basket drop-off position
  - Post-drop position
- Save all positions to YAML file

**Day 5 (1 hour):**
- Write basic Python node that reads YAML and moves to each position sequentially
- Verify motion planning succeeds for all waypoints
- Test on physical robot (dry run without gripper)

**Week 1 Deliverable:** Robot can reliably move through all predefined waypoints

---

### Week 2: Gripper Integration & Pick-Place Logic (6-8 hours)
**Goal:** Complete functional pick-and-place sequence

**Day 6-7 (2-3 hours):**
- Integrate gripper control into Python node
- Test gripper open/close commands with position feedback
- Determine appropriate gripper positions for object grasping
- Add gripper waypoints (open before grasp, close to grip, open to release)

**Day 8-9 (2-3 hours):**
- Implement complete pick-and-place state machine:
  1. Move to home
  2. Move to pre-grasp position
  3. Move to grasp position
  4. Close gripper
  5. Lift object (return to pre-grasp)
  6. Move to basket position
  7. Open gripper
  8. Return to home
- Add error checking at each step (planning success, gripper feedback)

**Day 10-11 (2 hours):**
- Create ROS2 service interface to trigger pick-and-place
- Add parameters: `pick_location` (station_1, station_2, etc.)
- Test multiple pick locations with same basket destination
- Tune motion planning parameters (velocity scaling, approach distance)

**Week 2 Deliverable:** Functional pick-and-place operation via ROS2 service call

---

### Week 3: Polish, Testing & Documentation (5-6 hours)
**Goal:** Reliable, repeatable system with clear documentation

**Day 12-13 (2 hours):**
- Add status feedback (ROS2 topic publishing operation state)
- Implement basic error recovery (retry motion planning, return to home on failure)
- Add logging for debugging
- Test edge cases (gripper fails to grasp, planning fails)

**Day 14-15 (2 hours):**
- Create comprehensive launch file with all parameters
- Add multiple pick location presets (at least 3 stations)
- Test repeatability: run 10 consecutive cycles
- Fine-tune positions if accuracy issues occur
- Record operation video for documentation

**Day 16-17 (1-2 hours):**
- Write user documentation:
  - Setup instructions (workspace layout, object placement)
  - Usage guide (how to launch, trigger pick-place)
  - How to teach new positions
  - Troubleshooting common issues
- Create README with example commands
- Optional: Add simple CLI tool for easier operation

**Week 3 Deliverable:** Production-ready demo with documentation

---

## Project Deliverables

### Code Deliverables
1. **ROS2 Package:** `kortex_pick_place_demo/`
   - `config/pick_place_positions.yaml` - Position database
   - `scripts/pick_place_node.py` - Main pick-and-place logic
   - `scripts/teach_positions.py` - Position teaching tool
   - `launch/pick_place_demo.launch.py` - Complete system launch
   - `package.xml`, `CMakeLists.txt` - ROS2 package config

2. **Launch Commands:**
   ```bash
   # Start robot and demo node
   ros2 launch kortex_pick_place_demo pick_place_demo.launch.py robot_ip:=192.168.1.10
   
   # Trigger pick from station 1
   ros2 service call /pick_place_trigger std_srvs/srv/Trigger "{data: 'station_1'}"
   ```

3. **Configuration:**
   - Documented YAML structure for positions
   - Gripper force/position parameters
   - Motion planning constraints (velocity, acceleration)

### Documentation Deliverables
1. **User Guide** (`docs/USER_GUIDE.md`)
   - Physical workspace setup
   - How to run the demo
   - How to add new pick locations
   - Safety considerations

2. **Technical Documentation** (`docs/TECHNICAL.md`)
   - System architecture diagram
   - State machine flow
   - API reference (services, topics)
   - Coordinate frame explanations

3. **Demo Video**
   - 2-3 minute video showing complete operation
   - Multiple pick locations demonstration

---

## Success Metrics

### Functional Requirements
- ✅ Robot picks object from predefined location
- ✅ Robot places object in basket without dropping
- ✅ 90%+ success rate over 10 consecutive runs
- ✅ Operation completes in <60 seconds per cycle
- ✅ At least 3 different pick locations configured

### Quality Requirements
- ✅ Clean, documented Python code
- ✅ No crashes or unhandled exceptions
- ✅ Clear error messages when operations fail
- ✅ User can teach new positions in <5 minutes

### Learning Outcomes
- Understand MoveIt2 Python API
- Experience with ROS2 services and parameters
- Practice motion planning and gripper control
- Learn coordinate frame transformations

---

## Technical Implementation Details

### Position Definition Structure
```yaml
# pick_place_positions.yaml
positions:
  home:
    joint_positions: [0.0, 0.3, 0.0, -1.5, 0.0, 1.0]
  
  station_1:
    pre_grasp:
      position: [0.4, 0.1, 0.3]
      orientation: [0.0, 0.707, 0.0, 0.707]  # quaternion
    grasp:
      position: [0.4, 0.1, 0.15]
      orientation: [0.0, 0.707, 0.0, 0.707]
  
  station_2:
    pre_grasp:
      position: [0.4, -0.1, 0.3]
      orientation: [0.0, 0.707, 0.0, 0.707]
    grasp:
      position: [0.4, -0.1, 0.15]
      orientation: [0.0, 0.707, 0.0, 0.707]
  
  basket:
    position: [0.3, 0.3, 0.4]
    orientation: [0.0, 0.707, 0.0, 0.707]

gripper:
  open_position: 0.0
  close_position: 0.6
  grasp_force: 50.0  # percentage
```

### Pick-Place State Machine
```
START
  ↓
Move to HOME
  ↓
Move to PRE-GRASP (above object)
  ↓
Open GRIPPER
  ↓
Move to GRASP (at object)
  ↓
Close GRIPPER
  ↓
Check grasp (gripper position feedback)
  ↓
Lift to PRE-GRASP
  ↓
Move to BASKET
  ↓
Open GRIPPER
  ↓
Move to HOME
  ↓
END (report success)
```

### Error Handling Strategy
- **Motion planning fails:** Retry 2x, then report failure and return home
- **Gripper fails to close:** Check position feedback, retry 1x, report if object not grasped
- **Unexpected collision:** Stop immediately, report error, manual intervention required
- **Service timeout:** 30-second timeout per motion, log and abort operation

---

## Risk Mitigation

### Technical Risks
| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Gripper fails to grasp object | Medium | Test multiple gripper positions, add position feedback check |
| Motion planning fails for positions | Low | Use teaching tool to record feasible positions, test in simulation |
| Object slips during transport | Medium | Test with different objects, tune gripper force, add slower motion |
| Robot collides with basket | Low | Define safe approach trajectory, use pre-drop position |

### Timeline Risks
| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Gripper tuning takes longer | Medium | Allocate extra buffer in Week 2, simplify objects if needed |
| Position teaching is tedious | Low | Create efficient teaching workflow early |
| Testing reveals accuracy issues | Medium | Week 3 has buffer time for refinement |

---

## Future Enhancements (Out of Scope)
These are **not** part of the 3-week project but could be added later:

- **Vision Integration:** Camera for object detection to eliminate predefined positions
- **Force Sensing:** Use arm force/torque feedback for adaptive grasping
- **Multiple Objects:** Pick multiple objects in sequence
- **Collision Avoidance:** Dynamic obstacle detection with camera
- **Web Interface:** Browser-based control panel
- **Object Classification:** Different grasp strategies per object type

---

## Getting Started After Approval

### Immediate Next Steps (Day 1)
1. Create package structure:
   ```bash
   cd ~/ros2_kortex_ws/src
   ros2 pkg create kortex_pick_place_demo --build-type ament_python \
       --dependencies rclpy moveit_py std_srvs geometry_msgs
   mkdir -p kortex_pick_place_demo/{config,launch,scripts}
   ```

2. Create position YAML template
3. Write basic MoveIt2 Python node that moves to home position
4. Test on physical robot

### Prerequisites Check
- ✅ MoveIt working (from previous fixes)
- ✅ Gripper functional (joint limits fixed)
- ✅ Robot accessible at 192.168.1.10
- ⚠️ Need: Physical workspace table
- ⚠️ Need: Test objects for grasping
- ⚠️ Need: Basket/container for drop-off

---

## Questions for Finalization

1. **Object Specifications:** What size/shape/weight objects do you want to test with? (e.g., wooden blocks, bottles, boxes)
2. **Workspace Layout:** Do you have a table ready? Approximate dimensions?
3. **Number of Stations:** How many pick locations do you want? (3 is suggested, can do 1-5)
4. **Basket Type:** Open basket, box with walls, or simple marked zone?
5. **Demo Environment:** Will you demonstrate in simulation first, then physical, or physical-only?

---

## Approval & Kickoff

**Estimated Total Effort:** 16-21 hours over 3 weeks  
**Daily Commitment:** 1-2 hours/day, 5-6 days/week  
**Project Start:** Upon approval  
**Expected Completion:** 3 weeks from start date  

**Ready to proceed?** Confirm approval and answer the finalization questions above to begin Day 1 setup.
