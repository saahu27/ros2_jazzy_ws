# PUMA560 Cartesian Motion with Vertical Lift

6-DOF PUMA560 robotic arm mounted on a vertical prismatic joint (lift), simulated in ROS 2 Jazzy + Gazebo Harmonic. Demonstrates constant-velocity Cartesian motion using trapezoidal velocity profiles and position-based admittance control for force regulation.

**Tested on**: Ubuntu 22.04, Ubuntu 24.04, macOS (Apple Silicon via noVNC)

## Prerequisites

| Option | Requirements |
|--------|--------------|
| Dev Containers | VS Code, Dev Containers extension, Docker |
| Docker (Linux) | Docker with X11 forwarding |
| Docker (macOS) | Docker Desktop + Web Browser (noVNC) |
| Native | ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2 |

---

## Quick Start by Platform

### 🐧 Linux
```bash
xhost +local:docker
./.devcontainer/scripts/build.sh
./.devcontainer/scripts/run.sh
# Inside container: ros2 launch puma560_py cartesian_motion.launch.py
```

### 🍎 macOS (Recommended: noVNC)

**Using Docker CLI:**
```bash
./.devcontainer/scripts/build.sh
./.devcontainer/scripts/run-novnc.sh
# Open http://localhost:6080/vnc.html in browser
# Click Connect, then run: gz sim
```

**Using VS Code Devcontainer:**
```bash
# Open in VS Code, select "ROS 2 Jazzy + Gazebo (macOS noVNC)" config
# Then in container terminal:
/root/start-vnc.sh
# Open http://localhost:6080/vnc.html in browser
```

---

## Option 1: VS Code Dev Containers

1. **Install prerequisites**:
   - [VS Code](https://code.visualstudio.com/)
   - [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   - Docker (Linux) or Docker Desktop (macOS)

2. **Open in container**:

   **Linux:**
   ```bash
   xhost +local:docker
   code /path/to/ros2_jazzy_ws
   ```
   Press `F1` → "Dev Containers: Reopen in Container"
   
   GUI applications (Gazebo, RViz) work directly via X11 forwarding.

   **macOS:**
   ```bash
   code /path/to/ros2_jazzy_ws
   ```
   Press `F1` → "Dev Containers: Reopen in Container"
   
   When prompted, select **"ROS 2 Jazzy + Gazebo (macOS noVNC)"** configuration.
   
   After container starts, run in the VS Code terminal:
   ```bash
   /root/start-vnc.sh
   ```
   
   Then open **http://localhost:6080/vnc.html** in your browser. Click Connect to see the GUI desktop.

3. **Build workspace** (inside container terminal):
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. **Run demos**:
   ```bash
   ros2 launch puma560_py cartesian_motion.launch.py
   ros2 launch puma560_py compliance_control.launch.py
   ```

---

## Option 2: Docker Build and Run

### Build the Docker image (all platforms):
   ```bash
   cd /path/to/ros2_jazzy_ws
   ./.devcontainer/build.sh
   ```

### Run on Linux (X11 forwarding):
   ```bash
   xhost +local:docker
./.devcontainer/run.sh
```

### Run on macOS (noVNC - Web Browser):
```bash
./.devcontainer/run-novnc.sh
```
Then open **http://localhost:6080/vnc.html** in your browser and click **Connect**.

A desktop will appear with a terminal. Run your commands there:
```bash
gz sim
# or
ros2 launch puma560_py cartesian_motion.launch.py
```

### Inside the container:
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash

   # Run cartesian motion demo
   ros2 launch puma560_py cartesian_motion.launch.py

   # Compliance control demo
   ros2 launch puma560_py compliance_control.launch.py
   ```

---
 
## Option 3: Native ROS 2 Jazzy Installation 

> **Note**: Untested, may have dependency issues.

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
│   ├── devcontainer.json       # VS Code container config (Linux)
│   ├── macos/
│   │   └── devcontainer.json   # VS Code container config (macOS)
│   ├── docker/
│   │   ├── Dockerfile          # ROS 2 Jazzy + Gazebo Harmonic image
│   │   ├── entrypoint.sh       # Container startup script
│   │   └── workspace.sh        # Workspace build script
│   └── scripts/
│       ├── build.sh            # Docker build script (cross-platform)
│       ├── run.sh              # Docker run script (Linux X11)
│       ├── run-novnc.sh        # Docker run script (macOS noVNC)
│       └── start-vnc.sh        # noVNC startup script (container)
├── results/                     # Plot outputs (persisted from container)
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

---

## Troubleshooting

### macOS

**Why noVNC instead of XQuartz?**
- XQuartz only supports OpenGL 2.1, but Gazebo/RViz require OpenGL 3.3+
- noVNC runs a virtual display inside the container with full OpenGL support
- Access via web browser - no additional software needed

**noVNC shows blank or won't connect:**
1. Run the startup script in the container:
   ```bash
   /root/start-vnc.sh
   ```
2. Wait a few seconds, then open http://localhost:6080/vnc.html
3. If still blank after connecting, run `xterm &` in VS Code terminal

**Black screen after connecting:**
```bash
export DISPLAY=:99
xterm &
```

**Performance on Apple Silicon:**
- The container runs x86_64 code via Rosetta 2 emulation
- This is expected; ROS Jazzy images are only available for AMD64
- Graphics use software rendering (llvmpipe) for compatibility

### Linux

**"Cannot connect to X server" error:**
```bash
xhost +local:docker
```

**GPU rendering issues:**
- The default configuration uses software rendering (`LIBGL_ALWAYS_SOFTWARE=1`)
- For GPU acceleration on Linux, edit `run.sh` and uncomment `--device=/dev/dri`

**Display not found:**
```bash
echo $DISPLAY  # Should show :0 or :1
export DISPLAY=:0  # If empty
```
