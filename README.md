# ACS6300_dissertation MATLAB code for LQR and MPC Controllers for Furuta Pendulum

This repository contains MATLAB implementations of two control strategies, **Linear Quadratic Regulator (LQR)** and **Model Predictive Control (MPC)**, to stabilize and control a Furuta pendulum. The controllers are designed to minimize cost functions while achieving reference tracking. Both simulations demonstrate the pendulum's dynamic response and control inputs.

## Features

- **State-Space Modeling**: Both controllers use the state-space representation of the Furuta pendulum.
- **Optimal Control Design**:
  - **LQR**: Solves the algebraic Riccati equation to compute optimal gains.
  - **MPC**: Predicts future states and minimizes a quadratic cost function over a finite horizon.
- **Reference Trajectories**: Time-varying reference signals are provided for the arm angle.
- **Simulation**: Dynamics are simulated using Euler integration.
- **Visualization**: Plots show pendulum and arm angles, control inputs, and reference trajectories.

---

## System Dynamics

The Furuta pendulum is a nonlinear system with:
- **Arm angle (\(\theta\))**: Horizontal rotation.
- **Pendulum angle (\(\alpha\))**: Vertical oscillation.
- The goal is to stabilize the pendulum in an upright position while controlling the arm to follow reference trajectories.

### State-Space Representation
The system is represented using matrices:
- **State matrix (\(A\))**
- **Input matrix (\(B\))**
- **Output matrix (\(C\))**
- **Feedforward matrix (\(D\))**

---

## Files in This Repository

### 1. `lqr_furuta_pendulum.m`
- Implements the **LQR** control strategy.
- Features:
  - Customizable weights for state and control cost matrices (\(Q\) and \(R\)).
  - Simulation over a 50-second time span with reference tracking.
  - Outputs:
    - Pendulum and arm angle trajectories.
    - Control effort (\(u\)).
    - Total cost \(J\).

### 2. `mpc_furuta_pendulum.m`
- Implements the **MPC** control strategy.
- Features:
  - Finite prediction horizon and adjustable weights.
  - Constrained optimization to minimize the cost function.
  - Outputs:
    - Predicted and actual trajectories for pendulum and arm angles.
    - Control effort (\(u\)).
    - Comparison with reference trajectories.
    - 
### 3. `plot_comparison_graphs.m`
- Plots the comparitive graphs of LQR and MPC.
- Shows the pendulum angle and arm angle in plots.

---

## Prerequisites

- Optimization Toolbox (for MPC)
- Basic understanding of control systems

---

## How to Use

1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/furuta-pendulum-controllers.git
   cd furuta-pendulum-controllers
   ```

2. Open MATLAB and load the desired script (`lqr_furuta_pendulum.m` or `mpc_furuta_pendulum.m`).

3. Run the script:
   ```matlab
   run('lqr_furuta_pendulum.m');  % For LQR simulation
   run('mpc_furuta_pendulum.m');  % For MPC simulation
   ```

4. View the output plots for:
   - Pendulum angle (\(\alpha\)) and arm angle (\(\theta\)).
   - Control input (\(u\)).
   - Reference tracking performance.
  
5. **View the comparitive results:**
   The simulation will generate plots showing the pendulum angle and arm angle over time, along with the controller outputs and reference signals for both MPC and LQR.
   ```matlab
   run('plot_comparison_graphs.m');  % For the plots
   ```
   - The plots will be displayed automatically in MATLAB.
   - The results can also be saved and analyzed for further comparison.
---

## Simulation Details

### LQR Controller
- **Weight Matrices**:
  - State weight (\(Q\)): `diag([3, 8, 1, 1])`
  - Control weight (\(R\)): `1`
- **Reference Signal**:
  - Alternates between \(\pi/4\), \(0\), and \(-\pi/4\) every 5 seconds after an initial stabilization phase.

### MPC Controller
- **Prediction Horizon**: User-defined in the script.
- **Weight Matrices**:
  - State weight (\(Q\)): `diag([10, 200, 1, 1])`
  - Control weight (\(R\)): `50`
- **Constraints**: Boundaries on control inputs can be adjusted.

---

## Output

- **Pendulum Angle (\(\alpha\))**: Tracks the vertical oscillation.
- **Arm Angle (\(\theta\))**: Tracks horizontal rotation.
- **Control Input (\(u\))**: Effort applied by the controller.
- **Total Cost**: Evaluates the controller's efficiency.

---

## Acknowledgments

- Inspired by the classic Furuta pendulum control problem.
- Special thanks to the MATLAB community for their contributions to control system design.

---
