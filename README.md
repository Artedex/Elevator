# Elevator Subsystem Simulation and Control

This repository contains a software simulation implementing the physical modeling, state control, and automation logic of an elevator mechanism.

---

## Architecture

The project is designed in a modular structure in accordance with the principles of the Command-Based Robotics Architecture.

### Core Components:

1. **Subsystem (ElevatorSubsystem):**
   - Represents the hardware components and actuators of the elevator mechanism.
   - Manages motor controllers, encoders, and sensor inputs/outputs.
   - Updates mechanism position during periodic loops and monitors software safety limits.

2. **Feedback and Control Loops:**
   - **PID Control:** Provides closed-loop position control to reach the target position accurately and stably.
   - **Feedforward (kS, kG, kV, kA):** Relieves the load on the PID loop by pre-calculating gravity acceleration ($kG$), static friction ($kS$), and velocity-dependent resistance.

3. **Trajectory and Profile Management:**
   - **Motion Magic / Trapezoidal Profile:** Minimizes stress and wear on mechanical components by limiting sudden velocity and acceleration changes.

4. **Safety Limits:**
   - Prevents mechanical damage through software soft limits and hardware limit switches.

---

## Physical Values and Parameters

Standard reference values used during the simulation and control of cooperating elevator parameters:

| Parameter | Description | Value / Unit | Reference |
| :--- | :--- | :--- | :--- |
| **Gravity Acceleration ($g$)** | Standard gravity constant | $9.81 \text{ m/s}^2$ | Physical Constant |
| **Gear Ratio** | Ratio between motor shaft and output drum | *Defined per project* | Hardware Design |
| **Drum Diameter** | Diameter of the drum wrapped by the rope | *Based on project CAD data* | Mechanical Model |
| **Carriage Mass** | Weight of the moving carriage platform | *Based on project CAD data* | Mechanical Model |
| **Maximum Velocity ($v_{\max}$)** | Theoretical maximum linear velocity | *Based on system limits* | Motor and Gearbox Data |
| **Maximum Acceleration ($a_{\max}$)** | Maximum acceleration threshold | *Based on system limits* | Motor Torque Curve |

### Sources and References:
- **WPILib Documentation:** Physical simulation and ElevatorSim modeling guides.
- **SysId (System Identification):** Feedforward constants ($kS, kG, kV, kA$) obtained through system characterization tests.
- **Motor Controller Manuals:** Closed-loop PID configuration standards.
