# NAO Robot Motion Control System

A Python-based motion control system for the NAO humanoid robot that bridges Python 2 (robot communication) and Python 3 (motion processing) to stream motion capture data to the robot in real-time.

## Project Overview

This system enables you to send motion capture sequences (BVH files) to a NAO robot through an XML-RPC bridge. The workflow is:

1. **Virtual Robot (Choregraphe)** → Simulate the robot and find its port
2. **Python 2 Server** → Creates an XML-RPC bridge between Python and NAO
3. **Python 3 Client** → Parses motion data and performs inverse kinematics
4. **Robot Execution** → Sends computed joint angles to the robot asynchronously

## Important: Python Version Requirements

This project **involves two different Python versions**:

- **Python 2.7** (required for `nao_server.py`): The NAO robot SDK (naoqi) only supports Python 2.7
- **Python 3** (required for `main_ik_client.py`): Modern Python 3 for motion processing and XML-RPC client communication

This is necessary because the naoqi library is a legacy C++ binding that was never updated to Python 3.

---

## Setup Instructions

### Prerequisites

- **Choregraphe**: https://developer.softbankrobotics.com/en/downloads/choregraphe
- **Python 2.7**: Download and install from python.org or via package manager
- **Python 3.7+**: For the client-side motion processing
- **Git Bash** (on Windows): Provides POSIX shell environment and comes with Python 2.7

### Step 1: Start the Virtual Robot in Choregraphe

1. Open **Choregraphe**
2. Create a new project or open an existing one
3. Click the **Virtual Robot** button (or go to `Robot > Virtual Robot > Start`)
4. Wait for the virtual robot to boot
5. You should see output in the console showing the robot's IP and port, typically:
   ```
   [CONNECTED] Robot is running at: 127.0.0.1:31559
   ```
   **Note the port number** (default is `31559` for virtual robots, `9559` for real robots over NAT)

### Step 2: Configure and Run the Python 2 Server

The server must run in Python 2.7 because it uses the naoqi library.

#### On Windows (Git Bash):

1. Open **Git Bash** (make sure it has Python 2.7 available)
2. Navigate to your project directory:
   ```bash
   cd /c/Users/aarya/Documents/Imperial\ College\ London/Year\ 4/FYP/Robot\ Code
   ```
3. Run the setup script to configure the Python path:
   ```bash
   source setup_naoqi.sh
   ```
   This script:
   - Deactivates any active Python virtual environment
   - Verifies Python 2.7 is in use
   - Adds the naoqi library to `PYTHONPATH`

4. Start the server with the robot's port:
   ```bash
   python nao_server/nao_server.py 31559
   ```
   Replace `31559` with your robot's actual port number.

   Expected output:
   ```
   Connecting to NAO at 127.0.0.1:31559...
   NAO is initialized and listening for commands on port 8000.
   Python 2 Bridge Server running. Waiting for Python 3 client...
   ```

The server is now running on **localhost:8000** and listening for XML-RPC calls.

### Step 3: Run the Python 3 Client

In a **separate terminal** (can be PowerShell, Command Prompt, or VS Code terminal):

1. Ensure Python 3 is in use:
   ```bash
   python --version  # Should show Python 3.x
   ```

2. Navigate to the project directory and run the client:
   ```bash
   python motion_logic/main_ik_client.py
   ```

The client will:
- Load a BVH motion capture file
- Parse the skeletal structure and frame data
- Compute inverse kinematics to convert human motion to NAO joint angles
- Send trajectories to the Python 2 server via XML-RPC
- The robot will execute the motion asynchronously

---

## Project Structure

```
├── nao_server/
│   └── nao_server.py           # Python 2.7 - XML-RPC server bridging to NAO
├── motion_logic/
│   └── main_ik_client.py       # Python 3 - BVH parsing and IK computation
├── setup_naoqi.sh             # Bash script to configure environment
├── archive/                    # Older versions and utilities
└── README.md                   # This file
```

---

## NAO Server (`nao_server.py`) - Python 2.7

The server creates an XML-RPC interface to communicate with the NAO robot. All methods are exposed as remote procedure calls accessible from the Python 3 client.

### Initialization

```python
NaoBridge(robot_ip, robot_port)
```

- **`robot_ip`**: IP address of the robot (default: `127.0.0.1` for virtual)
- **`robot_port`**: NAO's port (default: `31559` for virtual, `9559` for real)

On initialization:
- Connects to NAO's `ALMotion`, `ALRobotPosture`, and `ALAutonomousLife` services
- Disables autonomous life to prevent random movements
- Calls `wakeUp()` to activate motors
- Sets the robot to "StandInit" posture (standing position)

### Server Methods

#### 1. `play_trajectory(names, angles, times)`

Executes a motion trajectory asynchronously on the robot.

**Parameters:**
- **`names`** (list of strings): Joint names to control
  - Example: `["HeadYaw", "HeadPitch", "RShoulderPitch", "RShoulderRoll", ...]`
- **`angles`** (list of lists): Joint angle sequences in radians
  - Example: `[[0.1, 0.2, 0.3], [0.1, 0.2, 0.3], ...]` (one list per frame)
- **`times`** (list of floats): Time in seconds at which each keyframe should be reached
  - Example: `[0.033, 0.066, 0.099, ...]` for 30 FPS motion capture

**Behavior:**
- Uses `.post.angleInterpolation()` to execute motion **asynchronously**
- Returns `True` immediately without waiting for motion completion
- The robot smoothly interpolates between keyframes
- All specified joints move in parallel

**Example usage (from Python 3 client):**
```python
nao = xmlrpc.client.ServerProxy('http://localhost:8000', allow_none=True)
nao.play_trajectory(
    ["HeadYaw", "RShoulderPitch"],
    [[0.0, 0.5], [0.0, -0.5]],
    [0.0, 1.0]
)
```

#### 2. `stop()`

Emergency stop command that immediately halts all motion.

**Behavior:**
- Calls `killAll()` to terminate all running motion tasks
- Returns robot to "StandInit" posture safely
- Returns `True`

**Use case:** Press Ctrl+C in the server to trigger this via KeyboardInterrupt handler.

#### 3. `rest()`

Safely transitions the robot to a resting state.

**Behavior:**
- Sets posture to "StandInit"
- Calls `motion.rest()` to deactivate motors and lower the robot if possible
- Returns `True`

**Use case:** Call this after completing motion sequences to save power and reduce heat.

### Server Details

- **Port**: Runs on `localhost:8000`
- **Protocol**: XML-RPC (language-agnostic RPC over HTTP)
- **Allow None**: `allow_none=True` is set to permit `None` return values
- **Threading**: Handles one client at a time (SimpleXMLRPCServer is single-threaded)

---

## Python 3 Client (`main_ik_client.py`)

The client processes motion capture data and communicates with the server.

### Key Components

#### BVHParser

Parses BVH (Biovision Hierarchical) motion capture files.

- **Input**: Path to a `.bvh` file
- **Output**: 
  - `joint_channels`: Maps joint names to their channel indices
  - `frames`: List of frame data (one frame per row, columns are joint rotations)
  - `frame_time`: Duration of each frame in seconds

#### Coordinate Space Conversion

The client converts between two coordinate systems:

| Axis | BVH Space | NAO Space    |
|------|-----------|--------------|
| X    | Left      | Forward      |
| Y    | Up        | Up           |
| Z    | Forward   | Left         |

Function: `bvh_to_nao_space(v)` performs this conversion.

#### Forward Kinematics

Computes the 3D position of a joint by multiplying all rotation matrices from parent to child:

```python
R_head_global = R_torso * R_neck * R_head
v_head_fwd = rotate_vector(R_head_global, [0, 0, 1])
```

#### Inverse Kinematics (IK)

Solves the inverse kinematics problem for NAO's arms:

**Function:** `solve_nao_arm_ik(V_nao, W_nao, is_left)`

**Inputs:**
- `V_nao`: Target position of the upper arm
- `W_nao`: Target position of the lower arm (hand)
- `is_left`: Boolean indicating left (`True`) or right (`False`) arm

**Outputs:** Returns `(pitch, roll, elbow_yaw, elbow_roll)` - the 4 joint angles for one arm

**Key assumptions:**
- NAO's arm has 4 DOF (degrees of freedom)
- Motion is constrained to be physiologically feasible
- Solutions are clamped to NAO's hardware joint limits

#### Joint Angle Mapping

`map_bvh_to_nao()` converts a single BVH frame to NAO joint angles:

**Input:** One frame of motion capture data

**Output:** Dictionary of all NAO joints with their target angles:
```python
{
    "HeadYaw": 0.05,
    "HeadPitch": -0.1,
    "RShoulderPitch": 1.2,
    "RShoulderRoll": 0.8,
    "RElbowYaw": -0.5,
    "RElbowRoll": 0.3,
    "LShoulderPitch": 1.1,
    "LShoulderRoll": -0.9,
    "LElbowYaw": 0.4,
    "LElbowRoll": -0.2
}
```

Angles are automatically clamped to NAO's hardware limits.

### Main Execution

The `main()` function:
1. Connects to the Python 2 server on `localhost:8000`
2. Loads a BVH file (hardcoded path in the script)
3. Iterates through each frame
4. Computes IK for head and both arms
5. Sends the resulting joint angles to the server
6. Server executes the motion on the robot

---

## Workflow Example

### Terminal 1: Choregraphe Virtual Robot
```
Choregraphe → Virtual Robot running on port 31559
```

### Terminal 2: Git Bash (Python 2.7 Server)
```bash
$ source setup_naoqi.sh
Python 2.7 detected. Proceeding...
PYTHONPATH updated. Current path includes: C:/Python27/Lib/site-packages/pynaoqi/lib
Setup complete.

$ python nao_server/nao_server.py 31559
Connecting to NAO at 127.0.0.1:31559...
NAO is initialized and listening for commands on port 8000.
Python 2 Bridge Server running. Waiting for Python 3 client...
```

### Terminal 3: PowerShell / Command Prompt (Python 3 Client)
```bash
> python motion_logic/main_ik_client.py
Connecting to Python 2 NAO Bridge on localhost:8000...
Loading BVH and computing IK in Python 3...
[Client sends motion data, robot executes...]
```

---

## Troubleshooting

### "Could not connect to the Python 2 Server"
- Ensure `nao_server.py` is running on localhost:8000
- Check that the server started successfully and shows "Waiting for Python 3 client..."

### "Error: Current Python is 3.x. Please use Git Bash with Python 2.7"
- You must run the server from a terminal with Python 2.7
- On Windows, use Git Bash configured with Python 2.7
- Verify: `python --version` should show `Python 2.7.x`

### "Warning: Naoqi directory not found"
- The naoqi library is not installed in the expected location
- Install: `pip install naoqi` (in Python 2.7)
- Or update the `NAOQI_PATH` in `setup_naoqi.sh` to match your installation

### Robot Not Moving
- Verify the robot port number matches what Choregraphe reports
- Ensure autonomous life is disabled (server does this automatically)
- Check that joint angles are within NAO's limits
- Try calling `rest()` to ensure the robot is awake

### Motion Looks Jerky or Unnatural
- Verify the BVH file's frame rate matches the expected 30 FPS
- Check that `frame_time` in BVHParser is set correctly
- Ensure inverse kinematics constraints are appropriate for the motion type

---

## Key Design Decisions

### Why Two Python Versions?
- **naoqi** (C++ extension) was only compiled for Python 2.7
- Modern **Python 3** is used for computation and motion processing
- XML-RPC bridges the two worlds seamlessly

### Why XML-RPC?
- Language-agnostic protocol (could use Java, C++, etc. for client)
- Asynchronous execution via `.post` allows non-blocking motion
- Simple HTTP-based communication with no external dependencies

### Why Asynchronous Motion?
- Multiple trajectories can be queued without waiting
- Enables smooth, continuous motion without frame drops
- Server can receive new commands while motion is executing

---

## References

- [NAO Robot Documentation](https://developer.softbankrobotics.com/en/docs/aldebaran/2-1/nao/nao-robot)
- [BVH File Format](https://www.dancetech.com/pages/bvh.html)
- [Python XML-RPC Documentation](https://docs.python.org/3/library/xmlrpc.html)

