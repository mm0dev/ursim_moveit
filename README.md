# UR5e Digital Twin & Stress Test

This repository contains a fully containerized "Digital Twin" development environment for the Universal Robots UR5e. It runs entirely on Docker, simulating the hardware (URSim) and the control logic (ROS 2 Humble + MoveIt 2) in separate networked containers.

## 🏗 Architecture

The system consists of two Docker containers connected via a private bridge network (`ur_net`).

1. **The Body (`ursim`)**: Runs the official Universal Robots Simulator. It behaves exactly like the physical robot controller tablet.
   - Address: http://localhost:8080 (VNC Web Interface)

2. **The Brain (`ros2_ur_control`)**: Runs ROS 2 Humble, the UR Drivers, and MoveIt 2.
   - Development: Accessed via VS Code Remote Containers.

## 🚀 Prerequisites

- Docker Desktop installed and running
- VS Code with the "Dev Containers" extension installed

## 🛠️ Step 1: Start the Environment

Open your terminal in this folder and spin up the containers:

```bash
docker compose up
```

Wait until you see the logs settle. You can now access the Robot Simulator at: http://localhost:8080/vnc.html

## 🔌 Step 2: The "Handshake" (One-Time Setup)

Before ROS can control the robot, you must point the simulator to the ROS container.

1. Open the Simulator (localhost:8080)
2. Turn the robot ON (Bottom left button → ON → Start)
3. Go to **Installation** tab → **URCaps** → **External Control**
4. Set **Host IP** to: `ros2_ur_control` (This is the Docker DNS name)
5. Set **Custom Port** to: `50002`
6. Save the installation

**Note:** You only need to do this once; the volume mapping persists these settings.

## 🎮 Step 3: The "Safety Dance" (Launch Sequence)

To run the system, we need 3 separate terminals inside the `ros2_ur_control` container. Open VS Code, attach to the running container, and open 3 terminal tabs.

### Terminal 1: The Driver (The Listener)

This connects ROS to the robot hardware.

```bash
# 1. Source the workspace
source install/setup.bash

# 2. Launch the Driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=ursim launch_rviz:=false
```

**CRITICAL:** Once this is running, go back to the Simulator Tablet:
- Load the `ros_external` program
- Press **Play**
- Look at Terminal 1. It should say: `Robot connected to 50002`

### Terminal 2: MoveIt (The Planner)

This launches the path planning engine and RViz visualization.

```bash
# 1. Source the workspace
source install/setup.bash

# 2. Launch MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

**Check:** You should see the robot in RViz. The "Motion Planning" plugin should be active.

### Terminal 3: The Stress Test (The Brain)

This runs our custom Python script with advanced safety validation.

If you have modified code, rebuild first:

```bash
cd ~/ros2_ws
colcon build --packages-select my_ur_station
```

Run the stress test:

```bash
# 1. Source the workspace (MANDATORY)
source install/setup.bash

# 2. Run the node
ros2 run my_ur_station stress_test
```

## 🧠 What the Stress Test Does

The `stress_test.py` script performs advanced pose validation and safe motion planning:

### 1. **IK-Based Pose Validation**
Before sending any goal to MoveIt, the script:
- Calls the IK solver (`/compute_ik`) with collision avoidance enabled
- Verifies a valid joint solution exists
- Checks if the solution respects MoveIt's collision constraints

### 2. **Tool-Forearm Distance Check (Anti-Protective Stop)**
The script implements forward kinematics to compute the geometric distance between:
- The **tool flange** (end of wrist_3)
- The **forearm link** (line segment from elbow to wrist_1)

If this distance is < 3cm, the pose is rejected. This prevents the "Protective Stop" error that occurs when the wrist collides with the lower arm.

### 3. **Joint-Space Goal Sending**
Instead of sending pose constraints to MoveIt (which can fail with "position/orientation constraint violated" errors), the script:
- Uses the validated IK solution directly
- Sends **joint constraints** to MoveIt
- Ensures MoveIt plans to the exact configuration that passed validation

### 4. **Robust Error Handling**
- If a random pose fails validation, the script generates a new one (up to 500 attempts)
- If MoveIt planning fails (error code -27), it's logged but doesn't crash
- The script tracks statistics: success rate, invalid poses, tool-forearm violations

## 📊 Configuration Parameters

Edit the top of `stress_test.py` to customize:

```python
NUM_POSES = 30  # Number of random poses to test

# Workspace bounds (meters, relative to base_link)
WORKSPACE_X_MIN = 0.3
WORKSPACE_X_MAX = 0.7
WORKSPACE_Y_MIN = -0.3
WORKSPACE_Y_MAX = 0.3
WORKSPACE_Z_MIN = 0.2
WORKSPACE_Z_MAX = 0.6

# Motion parameters
MAX_VELOCITY_SCALING = 0.2  # 20% of max velocity
MAX_ACCELERATION_SCALING = 0.2  # 20% of max acceleration

# Safety distance
MIN_TOOL_FOREARM_DISTANCE = 0.03  # 3cm minimum (safety layer is 2.8cm)
```

## 📈 Understanding the Output

The stress test prints detailed statistics:

```
============================================================
STRESS TEST COMPLETE!
Total Poses: 30
Successful: 25
Failed: 5
Invalid Poses Generated: 120
Unsafe Trajectories Rejected: 0
Tool-Forearm Violations: 15
Success Rate: 83.3%
============================================================
```

- **Invalid Poses Generated**: How many random poses were rejected during validation
- **Tool-Forearm Violations**: How many poses were rejected due to distance < 3cm
- **Success Rate**: Percentage of poses that successfully executed

## ⚠️ Troubleshooting

### Error: "Connection Refused" in Terminal 1
- Ensure the Robot Simulator is running
- Ensure the `ros_external` program is playing on the tablet
- Ensure the "Host IP" in the URCap settings is exactly `ros2_ur_control`

### Error: "Goal rejected" or all poses fail
- Check that MoveIt is running (Terminal 2)
- Verify `/compute_ik` service is available: `ros2 service list | grep ik`
- Ensure the robot is in a valid start state (not in protective stop)

### Error: "Command not found: ros2"
- You forgot to source! Run `source install/setup.bash`

### Many "No IK solution found" messages
- This is normal - the script is rejecting poses that are unreachable or too close to singularities
- The script will keep trying until it finds valid poses

### All poses fail with "Error Code -27" (GOAL_IN_COLLISION)
- This means MoveIt's planner is finding collisions
- Check the collision objects in RViz
- Ensure the robot is not starting in a self-collision state

## 🔬 Technical Details

### Forward Kinematics Implementation
The script implements the Denavit-Hartenberg (DH) convention to compute forward kinematics:

```python
# UR5e DH parameters
d1 = 0.08916   # shoulder to elbow Z offset
a2 = -0.42500  # upper arm length
a3 = -0.39225  # forearm length
d4 = 0.10915   # wrist 1 offset
d5 = 0.09465   # wrist 2 offset
d6 = 0.0823    # wrist 3 to flange
```

These parameters define the robot's geometric structure and enable accurate distance calculations.

### Distance to Line Segment Algorithm
The tool-forearm distance check uses vector projection:

1. Compute the forearm vector (elbow → wrist_1)
2. Project the tool position onto the forearm line
3. Clamp the projection to the line segment bounds
4. Calculate Euclidean distance from tool to closest point

This gives the true minimum distance, not just endpoint distances.

## 📚 References

- [UR5e Technical Specifications](https://www.universal-robots.com/products/ur5-robot/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [ROS 2 Universal Robots Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

## 📝 License

This project is for educational and research purposes.