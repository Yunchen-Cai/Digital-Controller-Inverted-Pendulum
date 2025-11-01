# Digital Controller Design for Inverted Pendulum on Cart - PID and LQR in MATLAB

This repository contains MATLAB code, simulations, and documentation for designing and comparing discrete PID and LQR controllers to stabilize an inverted pendulum on a cart system. The project models system dynamics using Lagrangian mechanics, linearizes to state-space form, discretizes for digital control, and incorporates practical constraints like sensor limits, actuator saturation, and noise. An augmented Kalman filter is used for state estimation due to the absence of velocity sensors. Performance is evaluated through settling time, overshoot, and stability analysis (e.g., Jury's test, root locus).

Based on the ME5422 project requirements, it demonstrates concepts from discrete-time control, including z-transform, bilinear transformation, and Monte Carlo simulations for robustness.

---

## 📦 Project Overview

The inverted pendulum on a cart is an unstable system used as a benchmark for control design. Key elements:
- **System Parameters**: Cart mass m_c = 1.1 kg, pendulum mass m_p = 0.1 kg, length L = 0.5 m, gravity g = 9.81 m/s².
- **Controllers**:
  - **PID**: Tuned with derivative filtering to handle noise; requires manual effort for stability.
  - **LQR with Kalman Filter**: Optimal state feedback for superior performance and robustness; handles position regulation and disturbances better.
- **Simulations**: MATLAB-based, including step responses, noise addition, and Monte Carlo analysis.
- **Outcomes**: LQR outperforms PID in settling time and overshoot; both achieve BIBO and asymptotic stability.

Challenges addressed: No velocity sensors (Kalman estimation), saturation limits (-10 ≤ f ≤ 10 N), sensor ranges (-0.25 ≤ x ≤ 0.25 m, -0.21 ≤ θ ≤ 0.21 rad).

Refer to "ME5422 Project Report_Cai Yunchen.pdf" for detailed derivations, tuning, analysis, and results.

---

## 🗂️ Project Structure

The repository is organized as follows:

- **Figures/**: Generated plots and visualizations (e.g., step responses, root locus, Bode plots).
- controller_gains.mat: Stored gains for PID and LQR controllers.
- generate_analysis_plots.m: Script to generate stability and performance plots (e.g., Jury's test, Monte Carlo).
- lqr_controller.m: Implementation of LQR controller with Kalman filter.
- model.m: System modeling script (Lagrangian derivation, linearization, state-space).
- pid_controller.m: Discrete PID controller implementation with tuning and filtering.
- README.txt: Basic project notes (if any; superseded by this README.md).
- simulation.m: Main simulation script for running controllers and testing scenarios.
- system_model.mat: Saved system model data (continuous and discrete state-space).

- **ME5422 Project Report_Cai Yunchen.pdf**: Full project report with equations, results, and discussions.
- **LICENSE**: MIT License (add if missing).
- **README.md**: This file.

Architecture diagrams (e.g., control loops, state estimation) are included in the PDF report.

---

## 🚀 Setup and Usage

### Prerequisites
- MATLAB (R2023a or later recommended).
- Control System Toolbox (for functions like `lqr`, `kalman`, `c2d`, `step`).
- Signal Processing Toolbox (for filtering and analysis).

No external libraries needed; all code uses built-in MATLAB functions.

### Installation
1. Clone the repository:
   ```
   git clone https://github.com/Yunchen-Cai/Digital-Controller-Inverted-Pendulum.git  # Replace with your repo URL
   cd Digital-Controller-Inverted-Pendulum
   ```

2. Open MATLAB and add the repository folder to your path:
   ```
   addpath(genpath(pwd));
   ```

### Running the Code
1. **Model the System**: Run `model.m` to derive and save the continuous/discrete state-space models.
   - Outputs: system_model.mat with A, B, C, D matrices.

2. **Simulate Controllers**:
   - Run `simulation.m` to test PID and LQR under various conditions (e.g., initial angle offset, disturbances).
   - Example: Compare responses with/without noise.

3. **Tune and Analyze**:
   - PID: Edit gains in `pid_controller.m` (e.g., Kp, Ki, Kd with derivative filter).
   - LQR: Compute gains via `lqr` in `lqr_controller.m`; Kalman via `kalman`.
   - Plots: Run `generate_analysis_plots.m` for root locus, Bode, Jury's test, and Monte Carlo simulations.

4. **Custom Usage**:
   - Load model: `load('system_model.mat');`
   - Simulate step response: `step(sys);`
   - Add saturation: Use `saturation` block in Simulink (if extending to graphical simulation).

Results are displayed in figures and saved to Figures/ folder.

---

## 📜 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 🙏 Acknowledgments

- Department of Mechanical Engineering, National University of Singapore.
- Based on ME5422 course project (October 2025).
- Special thanks to MATLAB for the Control System Toolbox.

For questions or contributions, open an issue on GitHub.

---

