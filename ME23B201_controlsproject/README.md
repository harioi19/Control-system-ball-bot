# Ballbot Control System Project

## Overview
This project implements modeling, controller design, and state estimation for a ballbot (ball-balancing robot) system using MATLAB and Simulink. The system is modeled from first principles using Lagrangian mechanics, linearized for control design, and tested with two control strategies: LQR (optimal) and PID (cascaded).

## File Structure

### Main Files
- **ME23B201_Control_System_project.mlx** - Main MATLAB Live Script with complete analysis
- **matlab.mat** - MATLAB workspace with computed matrices

### Simulink Models (.slx)
- **ctrlr.slx** - LQR controller simulation
- **PID.slx** - Cascaded PID controller simulation
- **observer.slx** - Full-state observer (no noise)
- **noisy_observer.slx** - Full-state observer with noise
- **min_order.slx** - Reduced-order observer (no noise)
- **noisy_minorder.slx** - Reduced-order observer with noise

### Data and Analysis
- **equations_of_motion.ipynb** - SymPy symbolic derivation of system equations
- **ctrlr.xlsx, PID.xlsx, observer.xlsx, etc.** - Simulation data and results

## Quick Start

### Prerequisites
- MATLAB R2020a or later with Control Systems Toolbox and Simulink
- Python 3.8+ with SymPy (for symbolic derivation, optional)

### Running the Analysis

1. Open **ME23B201_Control_System_project.mlx** in MATLAB
2. Run all sections to:
   - Load physical parameters (mass, inertia, dimensions)
   - Compute state-space matrices (A, B, C, D)
   - Design LQR controller using Bryson's rule
   - Design cascaded PID controller
   - Design full-state and reduced-order observers
   - Compare performance metrics

3. Open individual Simulink models (.slx files) to visualize closed-loop responses

## System Model

**Generalized coordinates:** q = [x, theta]^T
- x: horizontal displacement of ball (m)
- theta: body pitch angle (rad)

**State vector:** X = [x, x_dot, theta, theta_dot]^T

**Linearized dynamics:** M1*q_ddot = M2*q + M3*u

Where:
- M1: inertia matrix (2x2)
- M2: gravity/stiffness matrix (2x2)
- M3: input matrix (2x1)

**State-space form:** Ẋ = AX + Bu, y = CX

## Controller Comparison

| Metric | LQR | PID |
|--------|-----|-----|
| Settling Time (x) | ~5 s | ~300 s |
| Control Type | Full-state feedback | Cascaded loops |
| Tuning Method | Bryson's rule | Root locus |
| Performance | 60x faster | Stable but slow |

**Key Finding:** LQR achieves ~60x faster settling than cascaded PID while maintaining safety constraints.

## Observer Design

**Full-State Observer (Kalman Filter)**
- Estimates all 4 states from theta measurement only
- Convergence time: ~90 s under noise
- Better noise filtering due to full model utilization

**Reduced-Order Observer**
- Estimates unmeasured states (x, x_dot, theta_dot) via eta-transformation
- Convergence time: ~120 s under noise
- Lower computational cost (3 states vs 4)

## Noise and Filtering

- White noise injected at measurement
- FFT analysis identifies noise spectrum
- Butterworth filter (2nd order, 20 Hz cutoff) applied
- Filter reduces actuator chatter without destabilizing system

## Key Results

1. **System is open-loop unstable** (positive eigenvalue detected)
2. **LQR controller stabilizes in ~5 seconds** with full-state feedback
3. **PID requires ~300 seconds** due to cascaded loop limitations
4. **Full-state observer** converges faster than reduced-order under noise
5. **Butterworth filtering** improves control signal smoothness

## How to Use Each File

### ME23B201_Control_System_project.mlx
Main script that runs the entire analysis in sequence. Execute sections to see:
- Parameters loaded
- State-space matrices computed
- LQR and PID controllers designed
- Performance comparison plots
- Observer convergence analysis

### ctrlr.slx
Run LQR controller simulation. Shows x(t) and theta(t) under state feedback control.

### PID.slx
Run cascaded PID simulation. Inner loop controls theta, outer loop controls x.

### observer.slx & noisy_observer.slx
Test observer performance. First without noise (perfect state estimation), then with noise (realistic scenario).

### min_order.slx & noisy_minorder.slx
Compare reduced-order observer against full-order. Note slightly slower convergence but lower computational load.

## Modifying the Design

**Change LQR weights:**
In ME23B201_Control_System_project.mlx, modify x_limit, theta_limit, xdot_max, thetadot_max, F_max

**Change PID gains:**
In PID.slx, adjust Kp, Ki, Kd for inner and outer loops

**Change observer tuning:**
In noisy_observer.slx, adjust Q_est and R_est covariances

## Troubleshooting

Q: Simulation doesn't run?
A: Ensure all .slx files are in the same folder as the .mlx script.

Q: LQR poles not calculated?
A: Check controllability with rank(ctrb(A,B)) = 4. Verify A and B computed correctly.

Q: Observer diverges?
A: Increase observer covariances (reduce Q_est or increase R_est). Check observability: rank(obsv(A,C)) = 4.

## Project Status
Complete. All simulations tested and validated.
