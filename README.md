# UNB-Vehicle-Dynamics
**Official Full Vehicle Simulation for UNB Formula SAE**

## 🏎️ Project Overview
This repository hosts the suspension simulation for the University of New Brunswick Formula SAE team. It serves as the primary tool for validating suspension kinematics, analyzing weight transfer, and optimizing damper settings before on-track testing.

The simulation integrates Simscape Multibody for physical modeling with Simulink control loops to predict vehicle behavior under dynamic race conditions.

## 🛠️ Key Features
* **Full Ride Model:** 7-DOF multi-body simulation.
* **Frequency Analysis:** Automated scripts to determine natural frequencies, spring stiffness and motion ratios.
* **Weight Transfer Logic:** Analysis of longitudinal and lateral load transfer during braking, acceleration, and steady-state cornering.
* **Parameter Database:** Centralized `vehicle_params.m` file ensuring all subsystems use identical mass, inertia, and stiffness values.

## 📂 Repository Structure
* **`/models`**: Contains the core `.slx` files (Full Ride, Quarter Car, etc.).
* **`/scripts`**: Initialization `.m` files, weight transfer & wheel-load analysis, undamped frequency testing.
* **`/data`**: `.mat` files for vehicle data and track inputs.
* **`/resources`**: Project definition files and path management.

### Prerequisites
* MATLAB R2024b or newer
* **Required Toolboxes:**
    * Simulink
    * Simscape
    * Simscape Multibody
    * Control System Toolbox

### Installation (for Contributors)
Clone the repository by pasting this in the MATLAB command window:
    ```
   !git clone https://github.com/camfarrell/UNB-FSAE-Vehicle-Dynamics
    ```
MATLAB will promt you for a username and password...
* Click your github profile photo (top right) > Settings.
* Scroll all the way down the left sidebar and click Developer settings.
* Click Personal access tokens > Tokens (classic).
* Click Generate new token (classic).
* You must check the box for repo (this gives full control of private repositories).
* Click Generate token at the bottom.
* Copy the long string of characters (starts with ghp_...), this is your password.

## 👨‍🔧 Maintained By:
* **Cameron Farrell**
* **Noah Stairs**
* **Alec Leblanc**
* **Riley Somerville**

---
*Created for the University of New Brunswick Formula SAE Design validation.*
