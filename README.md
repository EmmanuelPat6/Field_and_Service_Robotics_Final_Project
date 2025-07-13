# Field and Service Robotics Final Project PATELLARO EMMANUEL P38000239 #
# ✈️🚁 Modeling and Control of UAV Quadrotor using Hierarchical Control, Geometric Control and Passivity-Based Control with External Wrench Disturbances and APF Algorithm considering also some Aerodynamic Effects 🖥️🎯 #
🗂️ This README file provides an overview of everything included in the repository and explains how it is organized.

## Features 🪐 ##
- Dynamic Model 🚀🤖 
- Hierarchical Control 🧩📊
- Geometric Control 📐🛰️
- Passivity-Based Control ⚡🧭
- Obstacle Avoidance with APF 🚧🧲
- Local Minima and Improved APF⚠️🧱
- Hovering 🛸🛑
- Ground Effect 🛬⬇️
- Ceiling Effect 🏗️⬆️

## Available Directory in this Repository 📂 ##
- Hierarchical_Control_APF
- Geometric_Control_APF
- Passivity_Based_Control_APF
- Plots
- Video

## Getting Started ⏯️

## 📁 Repository Structure & Usage

Each repository contains the necessary **MATLAB** and **Simulink** files for the corresponding implementation.  
Some files may appear in multiple repositories, as they serve the same purpose across different setups.

## 🎯 Position Goal Repository (Improved APF)

This repository is designed to test an **Improved Artificial Potential Field (APF)** Algorithm capable of overcoming the **Local Minima Problem**.
To visualize the **Local Minima Problem**, a folder named `Traditional_APF_Simulink` has been included. It contains the classic APF implementation that clearly shows how the UAV can get stuck in Local Minima.


## 📍➡️📍➡️ Waypoints Repositories with Hierarchical Control, Geometric Control & Passivity-Based Control
They includes an extended simulation scenario with **Multiple Obstacles** and **Several Waypoints**.
Three control strategy have been implemented:
- 🔗 **Hierarchical Control**
- 📐 **Geometric Control**
- 🚀 **Passivity-Based Control**

# 🛠️ Customization & Parameters

To run different tests, simply modify:
- **Obstacle Positions, Heights, and Widths**  
- **Simulation Parameters** like, for example, `ρ₀` *(influence radius)*, *gain matrices* for controllers or the order `r` of teh Estimator
- **Waypoint Positions**
- **Disturbances**

Everything else is straightforward thanks to the simplicity of the main script.

# 📊 Output & Plots

Once the main `.m` script is executed and, automatically, also the Simulink file:
- A **video** of the UAV behavior in the scenario is shown  
- Multiple plots are **automatically generated**  
- Each plot is **saved in PDF format**
- The visual outputs help evaluate:  
  - The **performance of the control strategies**
  - The **position, velocity and attitude errors**
  - The **differencies between each of them**
  - The **control input values**
  - The **tacking trajectory**
  - The **robustness to external disturbances and model uncertainties**
  - The **estimated wrench values**
  - How the **aerodynamics effects can affect the motion**
