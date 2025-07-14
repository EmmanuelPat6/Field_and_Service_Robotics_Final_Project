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
- Hierarchical_Control
- Geometric_Control
- Passivity_Based_Control
- Improved_APF
- Plots
- Video

## Getting Started ⏯️

## 📁 Repository Structure & Usage

Each repository contains the necessary **MATLAB** and **Simulink** files for the corresponding implementation.  
Some files may appear in multiple repositories, as they serve the same purpose across different setups.

## 🎯 Improved APF Repository

This repository is designed to test an **Improved Artificial Potential Field (APF)** Algorithm capable of overcoming the **Local Minima Problem**.
To visualize the **Local Minima Problem**, a folder named `Traditional_APF_Simulink` has been included. It contains the classic APF implementation that clearly shows how the UAV can get stuck in Local Minima.


## 📍➡️📍➡️ Repositories with Hierarchical Control, Geometric Control & Passivity-Based Control

They includes an extended simulation scenario with **Multiple Obstacles** and **Several Waypoints**.
Three control strategy have been implemented:
- 🔗 **Hierarchical Control**
- 📐 **Geometric Control**
- 🚀 **Passivity-Based Control**

# 🛠️ Customization & Parameters

To run different tests, simply modify:
- **Simulation Parameters** like, for example, *gain matrices* for controllers or the order `r` of teh Estimator
- **Obstacle Positions, Heights, and Widths**  
- **Waypoint Positions**
- **Disturbances**
- Simulink Time more or less equal to *260 s* for City Scenario and *115 s* for 3 Obstacles Scenario

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
