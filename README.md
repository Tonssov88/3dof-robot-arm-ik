# 3-DOF Robot Arm with Inverse Kinematics

A 2D planar robot arm controlled via inverse kinematics, featuring smooth interpolated movements and dual control modes. Built with Arduino and custom geometric IK algorithms for precise end-effector positioning.

![Robot Arm Demo](images/demo.gif)

## Overview

This project implements a 3-degree-of-freedom robotic manipulator with custom inverse kinematics software. The system allows precise control of the end-effector position in 3D Cartesian space, with the IK engine automatically calculating required joint angles. Linear interpolation creates smooth, predictable motion along straight-line paths.

**Key Achievement**: Successfully implemented geometric IK solution achieving ±2mm positioning accuracy across a 210mm workspace.

## Features

- **Custom Inverse Kinematics Engine**: Geometric solution using law of cosines and triangle decomposition for 3-DOF manipulation
- **Smooth Linear Interpolation**: Cartesian-space path planning with 20-step interpolation for fluid movement
- **Dual Control Modes**: 
  - **IK Mode**: Position control via 3D coordinates (x, y, z)
  - **Manual Mode**: Direct servo angle control for testing and diagnostics
- **Safety Systems**: 
  - Workspace boundary validation
  - Self-collision prevention via elbow angle constraints
  - Per-servo calibration for accurate positioning
- **Real-time Serial Interface**: Interactive command-line control via USB
- **Memory-Optimized Code**: Runs on Arduino Uno/Nano with only 2KB RAM
- **Non-blocking Architecture**: Timing-based servo updates maintain interface responsiveness during motion

## Hardware

### System Components
| Component | Model/Spec | Purpose |
|-----------|------------|---------|
| Microcontroller | Arduino Nano (ATmega328) | Main controller (16MHz, 32KB flash, 2KB RAM) |
| Servo Driver | PCA9685 16-Channel PWM | I²C servo control with 12-bit resolution |
| Actuators | 3× MG996R Servos | High-torque metal gear servos (180° rotation) |
| Power Supply | 5V/3A Regulated | External servo power (critical: servos draw 2.5A each under load) |
| Structure | Custom segments | 3D printed arm linkages |

### Physical Specifications
- **Shoulder Segment (L1)**: 105mm
- **Forearm Segment (L2)**: 105mm  
- **Maximum Reach**: 210mm (L1 + L2)
- **Workspace**: Hemispherical sector, 210mm radius
- **Base Rotation Range**: 0° to 180°
- **Degrees of Freedom**: 3 (Base, Shoulder, Elbow)

### Architecture Diagram
```
┌─────────────┐
│  Arduino    │
│  Uno/Nano   │
└──────┬──────┘
       │ I²C (SDA/SCL)
       │
┌──────▼──────┐      External 5V/3A
│   PCA9685   │◄─────────────────────
│ Servo Driver│
└──────┬──────┘
       │ PWM Signals
       │
       ├──► Base Servo (Channel 0)
       ├──► Shoulder Servo (Channel 1)
       └──► Elbow Servo (Channel 2)
```



![Physical Build](images/assembly.jpg)

## Software 

### Coordinate System
- **X-axis**: Lateral (left/right)
- **Y-axis**: Vertical (up/down)  
- **Z-axis**: Depth (forward/backward)
- **Origin**: Base servo pivot point

### Inverse Kinematics Solution

The inverse kinematics solution uses geometric decomposition to convert 3D Cartesian coordinates into joint angles. The algorithm employs the law of cosines to solve the "two-bar linkage" problem for the elbow joint, while the base and shoulder angles are determined through trigonometric projection and triangle decomposition. This analytical approach provides deterministic, real-time performance with computation times under 5ms.

### Motion Planning: Linear Interpolation

Linear interpolation generates smooth motion by creating intermediate waypoints along a straight line between the start and end positions in 3D space. Rather than interpolating joint angles (which produces curved paths), the system calculates evenly-spaced (x,y,z) coordinates and solves inverse kinematics for each point. This approach ensures the end-effector follows smooth, predictable straight-line trajectories with approximately 1-second movement times.

## Usage

### Command Interface

The robot is controlled via serial commands at 9600 baud.

#### IK Mode (Default)
Specify end-effector position in millimeters:
```
0 100 100        # Move to (x=0, y=100mm, z=100mm)
0 0 210          # Fully extended forward horizontally
0 210 0          # Straight up (vertical)
50 100 150       # Arbitrary reachable position
```

#### Manual Mode
Direct servo angle control (degrees):
```
manual           # Enter manual mode
b 90             # Base to 90°
s 45             # Shoulder to 45°
e 30             # Elbow to 30°
a 90 45 30       # Set all three servos simultaneously
```

#### Utility Commands
```
ik               # Return to IK mode
home             # All servos to 90° (safe neutral position)
help             # Display command reference
```

### Example Session
```
=== Robot Arm IK Controller ===
Type 'help' for commands
Current mode: IK

> 0 100 100
Starting movement to (0.0, 100.0, 100.0)
[IK] Angles: B=0.0° S=45.0° E=90.0°
Movement complete

> 0 0 210
Starting movement to (0.0, 0.0, 210.0)
[IK] Angles: B=0.0° S=0.0° E=0.0°
Movement complete

> manual
Switched to manual mode

> s 90
SHOULDER set to 90.0°
[MANUAL] Angles: B=0.0° S=90.0° E=0.0°
```

![Serial Interface](images/serial_demo.png)

## Future Improvements

### Stabilized Base Platform
- Current setup uses tape to secure arm to table (functional but inelegant)
- Design 3D-printed base to house electronics (Arduino, PCA9685, power distribution)
- Add weight distribution to counteract tipping when arm extends to maximum reach

###End-Effector and Wrist Joint
- Add gripper mechanism to transform from positioning system to true manipulator
- Integrate wrist rotation joint (more challenging)
- Would require upgrading to 4-DOF kinematics
- Significantly more complex IK mathematics
- Trade-off between added capability and system complexity makes this longer-term goal

###Graphical User Interface
- Current serial command-line works well for testing/debugging
- Python-based GUI would be more accessible and intuitive
- Make experimentation faster than typing coordinates manually


