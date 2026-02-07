# Vehicle-Dynamics🏎️
**Official Vehicle Simulation Reposoitory for UNB Formula SAE**

This repository serves as the primary tool for validating suspension kinematics, analyzing weight transfer, and optimizing ride frequencies before on-track testing.

## 🛠️ Main Features
**Parameter Database:** Centralized `vehicle_params.m` function ensuring all subsystems use identical mass, inertia, dimensions, and stiffness. Some values are estimates and subject to change.

**Simulink Full Ride Model:** `State_Space_Model.slx` - 7-DOF second-order Simulink simulation using State Space control loops to predict vehicle behavior under dynamic race conditions. 

**Simscape Full Ride Model (WIP):** Full vehicle simulation using exact vehcle properties from imported CAD files to visualize frequency response to various track inputs.

**Weight Transfer:** `Wheel_Loads.m` function that outputs contact patch forces at each wheel and a breakdown of weight transfer contributions due to longitudinal and lateral load transfer during braking, acceleration, and steady-state cornering. 

**Suspension Loads (WIP):** Calcualtes the combined load at each suspension member for several load cases using a matrix inversion of the contact patch forces from the `Wheel_Loads.m` file. 

## 📂 Repository Structure
* **`/models`**: Contains the core `.slx` files (Full Ride, Quarter Car, etc.).
* **`/scripts`**: Initialization `.m` files, weight transfer & wheel-load analysis, undamped frequency testing.
* **`/data`**: `.mat` files for vehicle data and track inputs.
* **`/resources`**: Project definition files and path management.

### Prerequisites
* MATLAB R2024b or newer
* Simulink 
* Simscape
* Simscape Multibody
* Control System Toolbox

## Contributor Information 
This link from MATLAB explains how to clone the GitHub Repository as a MATLAB Project:

https://www.mathworks.com/help/matlab/matlab_prog/clone-git-repository.html

**Best Practices:** 
* To interact with a file, right click the blank space under files in the MATLAB Project, then click `Source Control`.
* After modifying any files: **`Commit` -> `Pull` -> `Push`** to prevent overwriting other members changes.
* **`Commit`**: A local snapshot that saves your progress to your project's history.
* **`Pull`**: Downloads the latest team updates from the server and merging them into your files.
* **`Push`**: Uploads your local commits to a shared remote server for others to see.

## 👨‍🔧 Maintained By:
* **Cameron Farrell**
* **Noah Stairs**
* **Alec Leblanc**
* **Riley Somerville**

---
*Created for the University of New Brunswick Formula SAE Design validation.*
