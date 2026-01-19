# EKF-Based State Estimation and Waypoint Navigation for an AUV in ROS 2 / Gazebo
This repository implements a ROS 2 / Gazebo simulation stack for autonomous navigation of an AUV (based on the BlueROV2 model).  
The focus of the project is on **state estimation via sensor fusion (EKF)** and **waypoint-based navigation**, integrating perception, planning, and control in a reproducible simulation environment.

The goal is not to introduce a novel algorithm, but to demonstrate a complete autonomy pipeline combining:
- Noisy sensor simulation
- EKF-based state estimation
- Path generation
- Waypoint tracking

<p align="center">
<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/7cc75c01-5113-42c1-b303-d7bd068ddc61" />
</p>

## System Architecture
The autonomy stack follows a classical estimation–planning–control structure:
1. **Perception/State Estimation**
 - Sensor fusion using the ROS2 EKF (robot_localization)
 - Multiple simulated sensors with realistic noise models
2. **Planning**
 - A custom node generates a geometric path from the start position to the goal
3. **Control/Execution**
 - A custom waypoint manager node sends the current target waypoint to the low-level controller
4. **Simulation**
 - Gazebo environment with a BlueROV2-based AUV model and full sensor stack

## Sensor Simulation and Noise Modelling
To emulate realistic conditions, ground-truth odometry from Gazebo is corrupted with noise and covariance before being sent to the EKF.  
The following custom ROS2 nodes are implemented:
- **Acoustic Positioning (Trilateration) Node**  
  Simulates position estimates obtained from acoustic beacons, with added covariance.
- **IMU Noise Node**  
  Injects noise into acceleration and orientation measurements.
- **Depth Sensor Node**  
  Provides noisy depth measurements.
- **DVL (Doppler Velocity Log) Node**  
  Simulates noisy velocity measurements

These signals are fused using the ROS 2 EKF (configured via YAML) to produce a robust estimate of the AUV pose and velocity.

## Custom ROS 2 Nodes
This repository includes the following custom nodes:
1. **Path Generator Node**  
   Generates a sequence of waypoints between the start and goal positions.
2. **Waypoint Manager Node**  
   Communicates the current target waypoint to the AUV controller.
3. **Sensor Noise Simulation Nodes**  
   - Acoustic trilateration simulator  
   - IMU noise injector  
   - Depth sensor simulator  
   - DVL noise simulator  

## Controllers and Model
- AUV model: **BlueROV2**  
- Controllers and model description are based on open-source BlueROV2 ROS 2 repositories  
- These components are reused and integrated into the simulation stack

## State Estimation
State estimation is performed using the ROS 2 `robot_localization` EKF:
- Configuration via YAML file
- Fuses:
  - Noisy IMU
  - Acoustic positioning
  - DVL velocity
  - Depth

The EKF provides:
- Estimated position
- Orientation
- Linear and angular velocity

## Running the simulation
Launch the main.launch file with 
````
ros2 launch pipeline_follower pipeline_launch.py
````

## Results
### Position estimate: EKF vs Ground truth
<img width="2292" height="1287" alt="image" src="https://github.com/user-attachments/assets/785e32ba-b9d4-4718-bd2a-008749f5d509" />

### Position estimate: EKF vs Pose given by trilateration
<img width="2292" height="1287" alt="image" src="https://github.com/user-attachments/assets/992a6dd4-6924-4e02-8090-7facf313defa" />
Sensor fusion provides a correction of the z position of the bluerov2

### Velocity estimate: EKF vs Noisy DVL
<img width="2292" height="1287" alt="image" src="https://github.com/user-attachments/assets/d8039b0f-ecf3-443e-9745-941f11754529" />







