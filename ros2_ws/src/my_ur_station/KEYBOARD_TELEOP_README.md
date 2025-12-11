# Keyboard Teleoperation for UR Robot with MoveIt Servo

This script provides real-time keyboard control of your URSim robot using MoveIt Servo for live teleoperation with Cartesian velocity commands.

## Features

- **Real-time Control**: Direct velocity control via MoveIt Servo (no motion planning delays)
- **Cartesian Space Control**: Move the end-effector up and down (Z axis)
- **MoveIt Servo**: Uses inverse Jacobian for real-time servo calculations
- **Safe Motion**: Continuous singularity checking and velocity limiting
- **Simple Interface**: Easy terminal-based control with Q/E keys
- **Command-line Tool**: Direct twist command sending for automation/testing (`servo_twist_cmd`)
- **Scriptable**: Can be integrated into bash scripts or Python automation

## Quick Start

### Build the Package
```bash
cd /root/ros2_ws && colcon build --packages-select my_ur_station && source install/setup.bash
```

### Run Everything (3 Terminals Required)

**Terminal 1 - Launch UR Robot Driver:**
```bash
cd /root/ros2_ws
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=ursim launch_rviz:=false
```
*Note: Make sure URSim is running and the External Control program is started on the teach pendant*

**Terminal 2 - Launch MoveIt Servo:**
```bash
cd /root/ros2_ws
source install/setup.bash
ros2 launch my_ur_station ur_servo.launch.py
```
*Expected warnings (safe to ignore):*
- "No kinematics plugins defined" - Servo will use inverse Jacobian instead
- "Failed to call service get_planning_scene" - Normal when MoveIt isn't running
- "No 3D sensor plugin(s)" - Normal, no depth camera configured

**Terminal 3 - Run Keyboard Teleop:**
```bash
cd /root/ros2_ws
source install/setup.bash
ros2 run my_ur_station keyboard_teleop_servo
```

## Controls

### Interactive Keyboard Mode

**Movement Commands:**
- **Q** - Move UP (Z+ axis, positive velocity)
- **E** - Move DOWN (Z- axis, negative velocity)
- **X** - Exit the program

**Usage:**
1. Type the letter (q, e, or x) and press ENTER
2. The robot will move for 1 second in the specified direction
3. Movement automatically stops after 1 second
4. Repeat as needed

### Command-Line Tool (for automation/testing)

Instead of using the interactive keyboard, you can send twist commands directly from the terminal:

**Move UP:**
```bash
ros2 run my_ur_station servo_twist_cmd up
```

**Move DOWN:**
```bash
ros2 run my_ur_station servo_twist_cmd down
```

**Stop:**
```bash
ros2 run my_ur_station servo_twist_cmd stop
```

**Custom twist (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):**
```bash
ros2 run my_ur_station servo_twist_cmd 0 0 0.15 0 0 0  # Move up faster (0.15 m/s)
ros2 run my_ur_station servo_twist_cmd 0.1 0 0 0 0 0    # Move forward (X+)
```

**Notes:**
- The command continuously publishes until you press Ctrl+C
- Automatically sends zero velocity when stopped
- Updates timestamps properly (this is why `ros2 topic pub` doesn't work)
- Publishes at 10 Hz

## How It Works

### MoveIt Servo Architecture
```
Keyboard Script → /servo_node/delta_twist_cmds (TwistStamped)
                     ↓
                MoveIt Servo Node (inverse Jacobian calculations)
                     ↓
                /scaled_joint_trajectory_controller/joint_trajectory
                     ↓
                UR Robot Controller → URSim
```

### Technical Details

**Servo Configuration:**
- **Move Group**: `ur_manipulator`
- **Planning Frame**: `base_link`
- **End-effector Frame**: `tool0`
- **Command Frame**: `tool0`
- **Linear Velocity**: 0.1 m/s (configurable in script)
- **Publish Rate**: ~30 Hz (34ms period)
- **Kinematics**: Inverse Jacobian (IKFast not required)

**Safety Features:**
- Singularity detection and avoidance
- Joint velocity limits enforced
- Automatic timeout on stale commands (0.5s)
- Joint limit margins to prevent boundary violations

### Key Files

**Launch File:** `/root/ros2_ws/src/my_ur_station/launch/ur_servo.launch.py`
- Loads robot URDF and SRDF via xacro
- Configures MoveIt Servo with proper parameters
- Sets up robot_description and kinematics

**Config File:** `/root/ros2_ws/src/my_ur_station/config/ur_servo.yaml`
- Servo parameters (velocity limits, thresholds, etc.)
- Topic names and output configuration
- Safety thresholds for singularities

**Teleop Script:** `/root/ros2_ws/src/my_ur_station/my_ur_station/keyboard_teleop_servo.py`
- Interactive keyboard control with Q/E keys
- Publishes TwistStamped messages with updated timestamps
- Auto-starts servo service on initialization
- Terminal-based input (works in Docker environments)

**Command Tool:** `/root/ros2_ws/src/my_ur_station/my_ur_station/servo_twist_cmd.py`
- Command-line twist sender
- Properly updates timestamps at 10 Hz
- Useful for scripting and automation
- Supports custom velocity values

## Customization

You can modify these parameters in `keyboard_teleop_servo.py`:

```python
# Movement speed
self.linear_speed = 0.1   # m/s for Z axis (0.1 = 10 cm/s)

# Command frame
self.current_twist.header.frame_id = 'base_link'

# Duration of movement per key press (in main loop)
for _ in range(10):  # 10 iterations at 0.1s = 1 second of movement
    self.send_twist(self.linear_speed)
    rclpy.spin_once(self, timeout_sec=0.1)
```

To modify servo parameters, edit `/root/ros2_ws/src/my_ur_station/config/ur_servo.yaml`:

```yaml
# Velocity limits
linear_velocity_limit: 0.3      # m/s (max Cartesian speed)
rotational_velocity_limit: 1.5  # rad/s
joint_velocity_limit: 2.0       # rad/s (per joint)

# Singularity thresholds
lower_singularity_threshold: 17.0       # Start slowing down
hard_stop_singularity_threshold: 30.0   # Complete stop

# Command timeout
incoming_command_timeout: 0.5  # Stop if no command for 0.5s

# Publish rate
publish_period: 0.034  # ~30 Hz
```

## Troubleshooting

### Robot doesn't move
1. **Check servo status**: 
   ```bash
   ros2 topic echo /servo_node/status --once
   # Should output: data: 0 (means OK)
   ```

2. **Verify servo is started**:
   ```bash
   ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
   # Should return: success: True
   ```

3. **Check if trajectories are being published**:
   ```bash
   ros2 topic hz /scaled_joint_trajectory_controller/joint_trajectory
   # Should show ~30 Hz when commands are sent
   ```

4. **Ensure robot program is running on URSim**:
   - On the teach pendant, load and run the "External Control" program
   - Check status: `ros2 topic echo /io_and_status_controller/robot_program_running --once`
   - Should output: `data: true`

### Servo node crashes
- **"Group 'panda_arm' not found"**: Fixed in current config, but if it occurs, check that `move_group_name: ur_manipulator` is set in `ur_servo.yaml`
- **"No kinematics plugins"**: This is just a warning, servo will use inverse Jacobian (works fine)
- **Missing robot_description**: Ensure Terminal 1 (UR driver) is running first

### Movement is too fast/slow
- Adjust `self.linear_speed` in `keyboard_teleop_servo.py`
- Or modify `linear_velocity_limit` in `ur_servo.yaml`
- When using command tool, specify custom velocities: `ros2 run my_ur_station servo_twist_cmd 0 0 0.05 0 0 0`

### Why doesn't `ros2 topic pub` work?

The key issue is **timestamps**. MoveIt Servo requires TwistStamped messages with **continuously updated timestamps** to detect stale commands. While `ros2 topic pub` publishes messages, it doesn't properly update the `header.stamp` field at each publication. This is why:

- ❌ `ros2 topic pub` - Doesn't work (static/missing timestamp)
- ✅ `ros2 run my_ur_station servo_twist_cmd` - Works (updates timestamp at 10 Hz)
- ✅ `ros2 run my_ur_station keyboard_teleop_servo` - Works (updates timestamp continuously)

### Robot stops unexpectedly
- Servo has a command timeout (0.5s default)
- Messages must have updated timestamps (already handled in script)
- Check for singularities: `ros2 topic echo /servo_node/status`

## Technical Details

### ROS2 Topics

**Input to Servo:**
- `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped)
  - Cartesian velocity commands
  - Must have frame_id and updated timestamp

**Output from Servo:**
- `/scaled_joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
  - Single-point trajectories with positions and velocities
  - Published at ~30 Hz when servo is active

**Status/Monitoring:**
- `/servo_node/status` (std_msgs/Int8)
  - 0 = No warnings
  - Non-zero = See status_codes.h for meanings
- `/joint_states` (sensor_msgs/JointState)
  - Current robot joint positions/velocities

### Dependencies
- `rclpy` - ROS2 Python client library
- `geometry_msgs` - TwistStamped messages
- `std_srvs` - Trigger service for starting servo
- `moveit_servo` - MoveIt Servo node package
- `ur_robot_driver` - UR robot controller interface

### Frame Convention
- **Base Frame**: `base_link` (robot base, fixed)
- **Planning Frame**: `base_link` (where servo does calculations)
- **End-effector Frame**: `tool0` (tool flange)
- **Command Frame**: `base_link` (frame for twist commands)
- **Coordinate System**: Right-handed (X forward, Y left, Z up)

## Example Sessions

### Interactive Keyboard Session
```bash
# Terminal 2 output (Servo node):
[moveit_servo.servo_calcs]: No kinematics solver instantiated for group 'ur_manipulator'. 
                             Will use inverse Jacobian for servo calculations instead.

# Terminal 3 output (Keyboard teleop):
============================================================
UP/DOWN CONTROL - MoveIt Servo
============================================================
  Q - Move UP (Z+)
  E - Move DOWN (Z-)
  X - Exit
============================================================
Type Q, E, or X then press ENTER

Command (q/e/x): q
▲ Moving UP...
Stopped

Command (q/e/x): e
▼ Moving DOWN...
Stopped

Command (q/e/x): x
Exiting...
```

### Command-Line Tool Session
```bash
# Move up continuously
$ ros2 run my_ur_station servo_twist_cmd up
Moving UP (Z+) at 0.1 m/s
Publishing at 10 Hz. Press Ctrl+C to stop.
^C
Stopping...

# Move down with custom speed
$ ros2 run my_ur_station servo_twist_cmd 0 0 -0.15 0 0 0
Sending custom twist: linear=(0.0, 0.0, -0.15), angular=(0.0, 0.0, 0.0)
Publishing at 10 Hz. Press Ctrl+C to stop.
^C
Stopping...
```

### Bash Script Example
```bash
#!/bin/bash
# Automated movement sequence

echo "Starting servo..."
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

echo "Moving up for 3 seconds..."
timeout 3 ros2 run my_ur_station servo_twist_cmd up

echo "Waiting 1 second..."
sleep 1

echo "Moving down for 3 seconds..."
timeout 3 ros2 run my_ur_station servo_twist_cmd down

echo "Done!"
```

## Optional: Running with RViz

For visualization, use 4 terminals instead:

**Terminal 1:** UR Driver (same as above)

**Terminal 2:** MoveIt with RViz
```bash
cd /root/ros2_ws
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

**Terminal 3:** MoveIt Servo (same as above)

**Terminal 4:** Keyboard Teleop (same as above)

This lets you see the robot model moving in RViz while controlling it.

## License

Part of the `my_ur_station` package.
