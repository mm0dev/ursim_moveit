# UR5e Digital Twin & Unchained Stress Test

This repository contains a fully containerized "Digital Twin" development environment for the Universal Robots UR5e. It runs entirely on Docker, simulating the hardware (URSim) and the control logic (ROS 2 Humble + MoveIt 2) in separate networked containers.

## 🏗 Architecture

The system consists of two Docker containers connected via a private bridge network (`ur_net`).

1. **The Body (`ursim`)**: Runs the official Universal Robots Simulator. It behaves exactly like the physical robot controller tablet.
   * *Address:* `http://localhost:8080` (VNC Web Interface)

2. **The Brain (`ros2_ur_control`)**: Runs ROS 2 Humble, the UR Drivers, and MoveIt 2.
   * *Development:* Accessed via VS Code Remote Containers.

## 🚀 Prerequisites

1. **Docker Desktop** installed and running.
2. **VS Code** with the "Dev Containers" extension installed.

## 🛠️ Step 1: Start the Environment

Open your terminal in this folder and spin up the containers:

```bash
docker compose up
````

Wait until you see the logs settle. You can now access the Robot Simulator at:
**[http://localhost:8080/vnc.html](https://www.google.com/search?q=http://localhost:8080/vnc.html)**

## 🔌 Step 2: The "Handshake" (One-Time Setup)

Before ROS can control the robot, you must point the simulator to the ROS container.

1.  Open the Simulator (**[localhost:8080](https://www.google.com/search?q=http://localhost:8080/vnc.html)**).
2.  Turn the robot **ON** (Bottom left button -\> ON -\> Start).
3.  Go to **Installation** tab -\> **URCaps** -\> **External Control**.
4.  Set **Host IP** to: `ros2_ur_control` (This is the Docker DNS name).
5.  Set **Custom Port** to: `50002`.
6.  Save the installation.

*Note: You only need to do this once; the volume mapping persists these settings.*

## 🎮 Step 3: The "Safety Dance" (Launch Sequence)

To run the system, we need **3 separate terminals** inside the `ros2_ur_control` container.
*Open VS Code, attach to the running container, and open 3 terminal tabs.*

### Terminal 1: The Driver (The Listener)

This connects ROS to the robot hardware.

```bash
# 1. Source the workspace
source install/setup.bash

# 2. Launch the Driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=ursim launch_rviz:=false
```

> **CRITICAL:** Once this is running, go back to the Simulator Tablet.
>
> 1.  Load the `ros_external` program.
> 2.  Press **Play**.
> 3.  Look at Terminal 1. It should say: `Robot connected to 50002`.

### Terminal 2: MoveIt (The Planner)

This launches the path planning engine and RViz visualization.

```bash
# 1. Source the workspace
source install/setup.bash

# 2. Launch MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

> **Check:** You should see the robot in RViz. The "Motion Planning" plugin should be active.

### Terminal 3: The Unchained Stress Test (The Brain)

This runs our custom Python script with the "Safety Bubble" logic.

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
ros2 run my_ur_station unchained_stress_test
```

## 🧠 What the Script Does

The `unchained_stress_test.py` script performs the following:

1.  **Safety Bubble:** It attaches an invisible collision object (cylinder) to the robot's wrist. This forces MoveIt to plan paths that keep the wrist 8cm away from the body, preventing the "Elbow Clamping" error.
2.  **Start State Fix:** It explicitly tells the planner to start from the *current* robot position, preventing "Invalid Start State" errors.
3.  **Robust Loop:** It generates random poses. If a pose is unreachable or unsafe, it catches the error, waits 0.5s, and retries without crashing the driver.

## ⚠️ Troubleshooting

**Error: "Connection Refused" in Terminal 1**

  * Ensure the Robot Simulator is running.
  * Ensure the `ros_external` program is playing on the tablet.
  * Ensure the "Host IP" in the URCap settings is exactly `ros2_ur_control`.

**Error: "Goal rejected" or Code 99999**

  * This usually means MoveIt is trying to plan from a collapsed state. Ensure you are running the `unchained_stress_test` (which has the fix) and not the old `moveit_mover`.

**Error: "Command not found: ros2"**

  * You forgot to source\! Run `source install/setup.bash`.

<!-- end list -->

```
```