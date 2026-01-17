# UNB-Vehicle-Dynamics
**Official Full Vehicle Simulation for UNB Formula SAE**


## 🏎️ Project Overview
This repository hosts the **Full Ride Model** for the University of New Brunswick Formula SAE team. It serves as the primary tool for validating suspension kinematics, analyzing weight transfer, and optimizing damper settings before on-track testing.

The simulation integrates **Simscape Multibody** for physical modeling with **Simulink** control loops to predict vehicle behavior under dynamic race conditions.

## 🛠️ Key Features
* **Full Ride Model:** 7-DOF multi-body simulation including chassis, suspension linkages, and tire interaction.
* **Frequency Analysis:** Automated scripts to determine natural frequencies and damping ratios for ride and pitch modes.
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

### Installation
Clone the repository by pasting this in the MATLAB command window:
    ```bash
    !git clone https://github.com/YourUsername/YourRepo.git
    ```
MATLAB will promt you for a username and password...
Click your github profile photo (top right) > Settings.
Scroll all the way down the left sidebar and click Developer settings.
Click Personal access tokens > Tokens (classic).
Click Generate new token (classic).
Select Scopes (Important): You must check the box for repo (this gives full control of private repositories).
Click Generate token at the bottom.
Copy the long string of characters (starts with ghp_...), this is your password.

### How to Run a Simulation
1.  Open `Full_Ride_Model.slx` in the `/models` folder.
2.  Select a drive cycle (e.g., *Skidpad*, *Acceleration*, or *Step Steer*) from the input block.
3.  Click **Run**.
4.  Use the **Data Inspector** to view `Chassis_Pitch`, `Chassis_Roll`, `Sprung_Acceleration`, and `Damper_Velocities`.

## 👨‍🔧 Maintained By:
* **Cameron Farrell**
* **Noah Stairs**
* **Alec Leblanc**
* **UNB Formula SAE Team**

---
*Created for the University of New Brunswick Formula SAE Design validation.*
