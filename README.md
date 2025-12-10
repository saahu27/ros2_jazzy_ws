# PUMA560 Cartesian Motion with Vertical Lift

6-DOF PUMA560 robotic arm mounted on a vertical prismatic joint (lift), simulated in ROS 2 Jazzy + Gazebo Harmonic. Demonstrates constant-velocity Cartesian motion using trapezoidal velocity profiles and position-based admittance control for force regulation.

**Tested on**: Ubuntu 22.04, Ubuntu 24.04

## Prerequisites

| Option | Requirements |
|--------|--------------|
| Dev Containers | VS Code, Dev Containers extension, Docker |
| Docker | Docker with X11 forwarding |
| Native | ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2 |

---

## Option 1: VS Code Dev Containers (Recommended)

1. **Install prerequisites**:
   - [VS Code](https://code.visualstudio.com/)
   - [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   - Docker

2. **Open in container**:
   ```bash
   xhost +local:docker
   code /path/to/ros2_jazzy_ws
   ```
   Press `F1` → "Dev Containers: Reopen in Container"

3. **Build workspace** (inside container terminal):
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. **Run demos**:
   ```bash
   # Cartesian motion demo
   ros2 launch puma560_py cartesian_motion.launch.py

   # Compliance control demo
   ros2 launch puma560_py compliance_control.launch.py
   ```

---

## Option 2: Docker Build and Run

1. **Build the Docker image**:
   ```bash
   cd /path/to/ros2_jazzy_ws
   ./.devcontainer/build.sh
   ```

2. **Run the container** (with X11 forwarding):
   ```bash
   xhost +local:docker

   docker run -it --rm \
     --network=host \
     --cap-add=SYS_PTRACE \
     --security-opt seccomp=unconfined \
     -e DISPLAY=$DISPLAY \
     -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
     -e LIBGL_ALWAYS_SOFTWARE=1 \
     -e QT_X11_NO_MITSHM=1 \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v $(pwd)/workspace:/root/ros2_ws/src \
     manipulation:latest \
     /bin/bash
   ```

3. **Inside the container**:
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash

   # Run cartesian motion demo
   ros2 launch puma560_py cartesian_motion.launch.py

   # Compliance control demo
   ros2 launch puma560_py compliance_control.launch.py
   ```

4. **Result Plots**:
   ```bash
   cd workspace/results
   ```
---

 **Note* :  *Untested using Option3 (native), might have some trouble with dependencies*
 
## Option 3: Native ROS 2 Jazzy Installation 

1. **Prerequisites**: ROS 2 Jazzy with Gazebo Harmonic and MoveIt 2 installed.

2. **Copy packages to your workspace**:
   ```bash
   cp -r /path/to/ros2_jazzy_ws/workspace/puma560_ros2_moveit/src/* ~/your_ros2_ws/src/
   ```

3. **Install dependencies**:
   ```bash
   cd ~/your_ros2_ws
   rosdep install -i --from-path src --rosdistro jazzy -y
   ```

4. **Build and source**:
   ```bash
   colcon build
   source install/setup.bash
   ```

5. **Run demos**:
   ```bash
   ros2 launch puma560_py cartesian_motion.launch.py
   ros2 launch puma560_py compliance_control.launch.py
   ```

---

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `cartesian_motion.launch.py` | Gazebo + MoveIt + Cartesian waypoint demo |
| `compliance_control.launch.py` | Gazebo with wall + MoveIt + force control demo |

---

## Output

Plots are saved to `results/` in the project root:
- `cartesian_motion_YYYYMMDD_HHMMSS.png` - End-effector velocities, XY trajectory, joint velocities
- `admittance_YYYYMMDD_HHMMSS.png` - Force tracking, position, force error

---

## Project Structure

```
ros2_jazzy_ws/
├── .devcontainer/
│   ├── Dockerfile          # ROS 2 Jazzy + Gazebo Harmonic image
│   ├── devcontainer.json   # VS Code container config
│   ├── build.sh            # Docker build script
│   └── entrypoint.sh       # Container startup script
├── results/                 # Plot outputs (persisted from container)
├── workspace/puma560_ros2_moveit/src/
│   ├── puma560_description/ # URDF, meshes, MoveIt config, launch files
│   │   ├── urdf/puma560_robot.urdf  # Robot with lift joint
│   │   ├── config/          # MoveIt, controllers, kinematics
│   │   └── worlds/wall_world.sdf    # Gazebo world with wall
│   └── puma560_py/          # Python control nodes
│       ├── puma560_py/
│       │   ├── cartesian_motion.py    # Cartesian waypoint controller
│       │   └── compliance_control.py  # Force regulation controller
│       └── launch/
│           ├── cartesian_motion.launch.py
│           └── compliance_control.launch.py
└── scripts/                 # Build and utility scripts
```

---

## Details

### Robot Configuration
- **Arm**: PUMA560 6-DOF manipulator
- **Lift**: Prismatic joint on Z-axis (0.0 - 1.0 m)
- **Total DOF**: 7 (lift + 6 arm joints)

### Cartesian Motion
- Trapezoidal velocity profile
- Default parameters: v_max = 0.08 m/s, a_max = 0.15 m/s²
- IK computed via MoveIt `/compute_ik` service

### Compliance Control
- Position-based admittance: `x_cmd = x_eq + Kf * (F_d - F_meas)`
- Target force: 100N
- Two-zone deadband for stable force regulation