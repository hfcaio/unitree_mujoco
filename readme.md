# Introduction
## Unitree mujoco
`unitree_mujoco` is a simulator developed based on `Unitree sdk2` and `mujoco`. Users can easily integrate the control programs developed with `Unitree_sdk2`, `unitree_ros2`, and `unitree_sdk2_python` into this simulator, enabling a seamless transition from simulation to physical development. The repository includes two versions of the simulator implemented in C++ and Python, with a structure as follows:
![](./doc/func.png)

## Directory Structure
- `simulate`: Simulator implemented based on unitree_sdk2 and mujoco (C++, recommended)
- `simulate_python`: Simulator implemented based on unitree_sdk2_python and mujoco (Python)
- `unitree_robots`: MJCF description files for robots supported by unitree_sdk2
- `terrain_tool`: Tool for generating terrain in simulation scenarios
- `example`: Example programs

## Supported Unitree sdk2 Messages:
**Current version only supports low-level development, mainly used for sim to real verification of controller**
- `LowCmd`: Motor control commands
- `LowState`: Motor state information
- `SportModeState`: Robot position and velocity data
- `IMUState`: Torso IMU state at `rt/secondary_imu` topic (G1 only)
- `rt/lf/lowstate` (`LowState`) and `rt/lf/sportmodestate` (`SportModeState`): the low-frequency variants of `rt/lowstate` and `rt/sportmodestate` (ROS 2: `/lf/lowstate`, `/lf/sportmodestate`), published for robots whose definition lists the `lf_lowstate` / `lf_sportmodestate` components (Go2, G1, G1-D), by default every 10th bridge tick
- `rt/brainco/{left,right}/cmd` (`unitree_go::MotorCmds_`), `rt/brainco/{left,right}/state` (`unitree_go::MotorStates_`) and `rt/brainco/{left,right}/touch` (`unitree_hg::PressSensorState_`, simulator only): BrainCo Revo2 hands on the G1 (`-r g1_brainco`), see [G1 with BrainCo hands](#g1-with-brainco-revo2-hands)

Note:
1. The numbering of the motors corresponds to the actual robot hardware. Specific details can be found in the [Unitree documentation](https://support.unitree.com/home/zh/developer).
2. In the actual robot hardware, the `SportModeState` message is not readable after the built-in motion control service is turned off. However, the simulator retains this message to allow users to utilize the position and velocity information for analyzing the developed control programs.

## Related links
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree Doc](https://support.unitree.com/home/zh/developer)
- [Mujoco Doc](https://mujoco.readthedocs.io/en/stable/overview.html)

## Message (DDS idl) type description
- Unitree Go2, B2, H1, B2w, Go2w robots use unitree_go idl for low-level communication.
- Unitree G1, H1-2, and AS2 robots use unitree_hg idl for low-level communication.


# Installation
## Nix / direnv
`flake.nix` provides every dependency for both simulators (MuJoCo 3.3.6 release, unitree_sdk2, cyclonedds, Python 3.13 with mujoco/pygame/opencv/unitree_sdk2py). Entering the shell links `simulate/mujoco` to the MuJoCo release.
```bash
direnv allow                      # or: nix develop
cd simulate && mkdir -p build && cd build && cmake .. && make -j4 unitree_mujoco
./unitree_mujoco -r go2 -s scene_terrain.xml
cd ../../simulate_python && python unitree_mujoco.py
```
Build the `unitree_mujoco` target only: `jstest` does not compile against the current unitree_sdk2.

## C++ Simulator (simulate)
### 1. Dependencies

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
```

#### unitree_sdk2
It is recommended to install `unitree_sdk2` in `/opt/unitree_robotics` path.
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```
For more details, see: https://github.com/unitreerobotics/unitree_sdk2
#### mujoco

Download the mujoco [release](https://github.com/google-deepmind/mujoco/releases), and extract it to the `~/.mujoco` directory;

```
cd unitree_mujoco/simulate/
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

### 2. Compile unitree_mujoco
```bash
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```
### 3. Test:
Run:
```bash
./unitree_mujoco -r go2 -s scene_terrain.xml
```
You should see the mujoco simulator with the Go2 robot loaded.
In a new terminal, run:
```bash
./test
```
The program will output the robot's pose and position information in the simulator, and each motor of the robot will continuously output 1Nm of torque.

**Note:** The testing program sends the unitree_go message. If you want to test G1 robot, you need to modify the program to use the unitree_hg message.

### AS2 simulation

AS2 uses the `unitree_hg` low-level messages. The simulator selects that IDL
automatically when `-r as2` is used; `-t 1` can be supplied to make the choice
explicit. From `simulate/build`, start the flat scene with:

```bash
./unitree_mujoco -r as2 -s plane_terrain.xml -t 1
```

Use `scene_terrain.xml` instead for the rough-terrain scene.

## Python Simulator (simulate_python)
### 1. Dependencies
#### unitree_sdk2_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
If you encounter an error during installation:
```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
Refer to: https://github.com/unitreerobotics/unitree_sdk2_python
#### mujoco-python
```bash
pip3 install mujoco
```

#### joystick
```bash
pip3 install pygame
```

### 2. Test
```bash
cd ./simulate_python
python3 ./unitree_mujoco.py
```
You should see the mujoco simulator with the Go2 robot loaded.
In a new terminal, run:
```bash
python3 ./test/test_unitree_sdk2.py
```
The program will output the robot's pose and position information in the simulator, and each motor of the robot will continuously output 1Nm of torque.

**Note:** The testing program sends the unitree_go message. If you want to test G1 robot, you need to modify the program to use the unitree_hg message.


# Usage
## 1. Simulation Configuration
### C++ Simulator
The configuration file for the C++ simulator is located at `/simulate/config.yaml`:
```yaml
# Robot name loaded by the simulator
# "go2", "b2", "b2w", "h1", "go2w", "g1", "h2", "as2"
robot: "go2"
# Robot simulation scene file
# For example, for go2, it refers to the scene.xml file in the /unitree_robots/go2/ folder
robot_scene: "scene.xml"
# DDS domain id, it is recommended to distinguish from the real robot (default is 0 on the real robot)
domain_id: 1

# -1: automatic, 0: unitree_go, 1: unitree_hg
# Automatic mode selects unitree_hg for as2.
idl_type: -1

use_joystick: 1 # Simulate Unitree WirelessController using a gamepad
joystick_type: "xbox" # support "xbox" and "switch" gamepad layout
joystick_device: "/dev/input/js0" # Device path
joystick_bits: 16 # Some game controllers may only have 8-bit accuracy

# Network interface name, for simulation, it is recommended to use the local loopback "lo"
interface: "lo"
# Whether to output robot link, joint, sensor information, 1 for output
print_scene_information: 1
# Whether to use virtual tape, 1 to enable
# Mainly used to simulate the hanging process of H1 robot initialization
enable_elastic_band: 0 # For H1
```
### Robot definitions (C++ simulator)
Each robot can have a definition file `simulate/robots/<robot>.yaml` that lists the bridge components publishing its extra DDS topics:
```yaml
idl: hg          # go | hg, optional (otherwise decided from the number of actuators)
components:
  - secondary_imu  # rt/secondary_imu
  - bms_state      # rt/lf/bmsstate
  - lf_lowstate    # rt/lf/lowstate, a copy of rt/lowstate at a lower rate
  - {type: lf_sportmodestate, hz: 50}   # rt/lf/sportmodestate at 50 Hz
```
The `lf_*` components republish what the base bridge publishes on `rt/lowstate` / `rt/sportmodestate`. Rate parameter: `rate_div` (publish every N-th bridge tick, default 10) or `hz` (the bridge ticks at up to 1000 Hz). `lf_lowstate` uses the `unitree_go` or `unitree_hg` type matching the robot's IDL; `rt/lf/sportmodestate` is always `unitree_go::SportModeState_`.

A `motors:` map (`DDS motor index: actuator name`) lets a model hold actuators that are not DDS motors, such as the G1-D wheels and lifting column; without it, actuator `i` is DDS motor `i`. A component can take parameters, e.g. `- {type: waist_pitch, actuator: waist_pitch}`.

### G1-D
```bash
./unitree_mujoco -r g1_d -s scene.xml
```
Model converted from `unitreerobotics/unitree_ros` (`g1_d_description`), without hands (hand mount sites: `left_hand_mount`, `right_hand_mount`). Topics: `rt/lowcmd` / `rt/lowstate` (29-motor G1 numbering: waist yaw = 12, arms = 15-28, legs unused), `rt/waistpitchcmd` / `rt/waistpitchstate`, `rt/hispeed_state` (`y` = column height in m), `rt/secondary_imu`, `rt/lf/bmsstate`, `rt/lf/lowstate`, `rt/sportmodestate`, `rt/lf/sportmodestate`. The wheels and the column are not DDS motors on the real robot (they use the `agv` service, not simulated yet); in the viewer you can drive the `left_wheel`, `right_wheel` and `lift` actuators with the control sliders. The four small caster spheres under the chassis are a stand-in so the base stays upright.

### G1 with BrainCo Revo2 hands
```bash
./unitree_mujoco -r g1_brainco -s scene_29dof.xml
```
`simulate/robots/g1_brainco.yaml` uses the plain G1 scenes (`scene_dir: g1`, so `-r g1 -s scene_29dof.xml` is unchanged) and attaches a BrainCo Revo2 hand to each wrist when the model is loaded (see [Model composition](#model-composition-attachments)). The G1 motors keep their numbering on `rt/lowcmd` / `rt/lowstate` (explicit `motors:` map) and all G1 topics are as for `-r g1`. The hands use the topics of [brainco_hand_service](https://github.com/unitreerobotics/brainco_hand_service), the service the real G1 runs for these hands:

| Topic | Type | Content |
|---|---|---|
| `rt/brainco/<side>/cmd` | `unitree_go::msg::dds_::MotorCmds_` | 6 `cmds`, order `[Thumb, ThumbAux, Index, Middle, Ring, Pinky]`; `q` in [0, 1] (0 open, 1 closed), `dq` in [0, 1] speed (fraction of the finger's max speed; 1 until the first command); other fields ignored |
| `rt/brainco/<side>/state` | `unitree_go::msg::dds_::MotorStates_` | 6 `states`, 100 Hz: `q` in [0, 1], `dq` in [-1, 1] (fraction of max speed, + = closing), `tau_est` in [-1, 1] (actuator force / force limit, the real service reports current / max); simulator extras: `q_raw` = joint angle (rad), `dq_raw` = joint velocity (rad/s) |
| `rt/brainco/<side>/touch` | `unitree_hg::msg::dds_::PressSensorState_` | simulator only, 100 Hz: `pressure[0..4]` normal force (N) and `pressure[5..9]` tangential force (N) on the fingertip pads `[Thumb, Index, Middle, Ring, Pinky]` |

ROS 2 clients see them as `/brainco/<side>/cmd` etc. with the `unitree_go` / `unitree_hg` message packages of unitree_ros2. In the model, `Thumb` is the thumb flexion joint (`thumb_proximal_joint`), `ThumbAux` the thumb rotation (`thumb_metacarpal_joint`) and each finger's distal joint follows its proximal joint (MuJoCo joint equality with the URDF mimic ratio). A normalized position maps linearly onto the joint range of the Revo2 URDF (`thumb` 0-1.047, `thumb_aux` 0-1.553, fingers 0-1.41 rad).

Component parameters (`components:` entry `{type: brainco_hand, side: left}`): `side` (required), `prefix` (names of the attached hand's elements, default `<side>_`), `topic` (default `rt/brainco/<side>`), `hz` (state/touch rate, default 100), `touch` (default true), `max_velocity` (6 values in rad/s, default from the URDF: 2.5303, 2.6175, 2.2685 x4).

Sources and what was assumed:
- Topics, types, finger order, [0, 1] normalization, `dq` = speed, 100 Hz: `brainco_hand_service` `main.cpp` / `test/test_brainco_hand_server.cpp` (`stark_set_finger_unit_mode(..., FINGER_UNIT_MODE_NORMALIZED)`, `include/stark-sdk.h`: positions 0-1000 with 0 = open, speeds/currents -1000..1000 with + = closing). That the normalized range is the full URDF joint range and maps linearly is an assumption.
- The service reads no tactile data, and neither do the ROS 2 nodes of [brainco_hand_ros2](https://github.com/BrainCoTech/brainco_hand_ros2) (its ros2_control interface exposes only position/velocity; the EtherCAT driver reads touch registers but only logs them). The Revo2 touch version measures per finger a normal force, a tangential force and its direction (`stark-sdk.h` `CTouchFingerItem`, 0.01 N units). The simulator publishes normal and tangential force in N on `rt/brainco/<side>/touch` with the SDK-shipped `PressSensorState_` type (the Dex3-1 hand's tactile type), so no new IDL is needed; direction, proximity and status are not simulated. This topic is a simulator extension: no real robot publishes it.
- The ROS 2 interface of brainco_hand_ros2 (`/<side>_revo2_hand_controller/joint_trajectory`, `/joint_states`) is not implemented: it is not what the G1 uses, and its message types are not shipped by unitree_sdk2.
- Dynamics (position servo gain, joint damping/armature, pad friction) are tuning values, not measured; the force limits are the URDF `effort` values.

The hand model is `unitree_robots/components/brainco_revo2/{left,right}.xml`, converted from [BrainCoTech/revo2_description](https://github.com/BrainCoTech/revo2_description) v1.1.1 (Apache-2.0, `LICENSE` in that directory; kinematics, limits, masses and mimic ratios as in the URDF, meshes decimated to about 2 MB for both hands). Each fingertip has a `touch` sensor and a `contact` sensor on a box site around its tactile pad. The wrist mounting (hand base 0.0591 m from the wrist yaw joint, the wrist flange `g1_adapter_<side>.xml`, the wrist yaw link inertia without the rubber hand) comes from `unitree_ros` `robots/g1_with_brainco_hand`.

### Model composition (attachments)
A robot definition can attach component models (hands, arms, ...) to its scene at load time:
```yaml
scene_dir: g1                   # optional: -s scenes come from unitree_robots/g1 instead of unitree_robots/<robot>
attachments:
  - type: brainco_revo2         # model unitree_robots/components/<type>/<side>.xml (<type>/<type>.xml without side)
    side: left                  # default name prefix "<side>_"; or set prefix/suffix
    mount: left_wrist_yaw_link  # body or site of the scene
    pos: [0.0591, 0, 0]         # optional pose of the component in the mount frame
    rpy: [-1.570796326, 0, -1.570796326]   # URDF convention; or quat: [w, x, y, z]
    remove_mesh_geoms: [left_rubber_hand]  # optional: delete the scene's geoms using these meshes
    mount_inertial: {mass: 0.0846, pos: [...], fullinertia: [ixx, iyy, izz, ixy, ixz, iyz]}  # optional
  - {model: brainco_revo2/g1_adapter_left.xml, side: left, mount: left_wrist_yaw_link, pos: [0.0415, 0, 0]}
```
The simulator parses the scene and each component with MuJoCo's `mjSpec` API, attaches the first body of the component (with its actuators, sensors, equalities, contact excludes, assets and defaults) via `mjs_attach` under the prefix, and compiles (`simulate/src/robot_compose.h`). So actuator `index` of the left hand becomes `left_index`, sensor `index_touch` becomes `left_index_touch`; components look elements up by these names. Mesh paths of a component resolve against its own file's `meshdir`. A later attachment can mount on a body added by an earlier one. Robots without `attachments` are loaded with `mj_loadXML` as before. Because attachments add actuators, a robot with attachments needs a `motors:` map for its DDS motors. Reloading the model from the viewer rebuilds the same composition; a different file dropped on the viewer is loaded as is.

Robots without a definition file keep the previous behaviour (`unitree_hg` robots get both components above). `idl_type` in `config.yaml` or `-t` still overrides `idl`. Components are registered in `simulate/src/components.h`.

### Python Simulator
The configuration file for the Python simulator is located at `/simulate_python/config.py`:
```python
# Robot name loaded by the simulator
# "go2", "b2", "b2w", "h1"
ROBOT = "go2"
# Robot simulation scene file
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"  # Robot scene
# DDS domain id, it is recommended to distinguish from the real robot (default is 0 on the real robot)
DOMAIN_ID = 1  # Domain id
# Network interface name, for simulation, it is recommended to use the local loopback "lo"
INTERFACE = "lo"  # Interface
# Whether to output robot link, joint, sensor information, True for output
PRINT_SCENE_INFORMATION = True

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# Whether to use virtual tape, 1 to enable
# Mainly used to simulate the hanging process of H1 robot initialization
ENABLE_ELASTIC_BAND = False
# Simulation time step (unit: s)
# To ensure the reliability of the simulation, it needs to be greater than the time required for viewer.sync() to render once
SIMULATE_DT = 0.003  
# Visualization interface runtime step, 0.02 corresponds to 50fps/s
VIEWER_DT = 0.02
```
### Joystick
The simulator will use an Xbox or Switch gamepad  to simulate the wireless controller of the robot. The button and joystick information of the wireless controller will be published through "rt/wireless_controller" topic. `use_joystick/USE_JOYSTICK` in `config.yaml/config.py` needs to be set to 0, when there is no gamepad. If your gamepad is not in Xbox or Switch layout, you can modify it in the source code (The button and joystick IDs can be  determined  using `jstest`):

In `simulate/src/unitree_sdk2_bridge/unitree_sdk2_bridge.cc`: 
```C++
 if (js_type == "xbox")
{
    js_id_.axis["LX"] = 0; // Left stick axis x
    js_id_.axis["LY"] = 1; // Left stick axis y
    js_id_.axis["RX"] = 3; // Right stick axis x
    js_id_.axis["RY"] = 4; // Right stick axis y
    js_id_.axis["LT"] = 2; // Left trigger
    js_id_.axis["RT"] = 5; // Right trigger
    js_id_.axis["DX"] = 6; // Directional pad x
    js_id_.axis["DY"] = 7; // Directional pad y

    js_id_.button["X"] = 2;
    js_id_.button["Y"] = 3;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 4;
    js_id_.button["RB"] = 5;
    js_id_.button["SELECT"] = 6;
    js_id_.button["START"] = 7;
}
```

In `simulate_python/unitree_sdk2_bridge.py`: 
```python
if js_type == "xbox":
    self.axis_id = {
        "LX": 0,  # Left stick axis x
        "LY": 1,  # Left stick axis y
        "RX": 3,  # Right stick axis x
        "RY": 4,  # Right stick axis y
        "LT": 2,  # Left trigger
        "RT": 5,  # Right trigger
        "DX": 6,  # Directional pad x
        "DY": 7,  # Directional pad y
    }

    self.button_id = {
        "X": 2,
        "Y": 3,
        "B": 1,
        "A": 0,
        "LB": 4,
        "RB": 5,
        "SELECT": 6,
        "START": 7,
    }
```

### Elastic band for humanoid 
Consider humanoid robots are not suitable for starting in ground, a virtual elastic band was designed to simulate the lifting and lowering of humanoid robots. Setting ` enable_elastic_mand/ENABLE_ELSTIC_BAND=1 ` can enable the virtual elastic band. After loading the robot, press' 9 'to activate or release the strap, press' 7' to lower the robot, and press' 8 'to lift the robot.

## 2. Terrain Generation Tool
We provide a tool to parametrically create simple terrains in the mujoco simulator, including stairs, rough ground, and height maps. The program is located in the `terrain_tool` folder. For specific usage instructions, refer to the README file in the `terrain_tool` folder.
![Terrain Generation Example](./doc/terrain.png)

## 3. Sim to Real
The `example` folder contains simple examples of using different interfaces to make the Go2 robot stand up and then lie down. These examples demonstrate how to implement the transition from simulation to reality using interfaces provided by Unitree. Here is an explanation of each folder name:
- `cpp`: Based on C++, using `unitree_sdk2` interface
- `python`: Based on Python, using  `unitree_sdk2_python` interface
- `ros2`: Based on ROS2, using `unitree_ros2` interface

### unitree_sdk2
1. Compile
```bash
cd example/cpp
mkdir build && cd build
cmake ..
make -j4
```
2. Run:
```bash
./stand_go2 # Control the robot in the simulation (make sure the Go2 simulation scene has been loaded)
./stand_go2 enp3s0 # Control the physical robot, where enp3s0 is the name of the network card connected to the robot
```
3. Sim to Real
```cpp
if (argc < 2)
{   
    // If no network card is input, use the simulated domain id and the local network card
    ChannelFactory::Instance()->Init(1, "lo");
}
else
{   
    // Otherwise, use the specified network card
    ChannelFactory::Instance()->Init(0, argv[1]);
}
```
### unitree_sdk2_python
1. Run
```bash
python3 ./stand_go2.py # Control the robot in the simulation (make sure the Go2 simulation scene has been loaded)
python3 ./stand_go2.py enp3s0 # Control the physical robot, where enp3s0 is the name of the network card connected to the robot
```
2. Sim to Real
```python
if len(sys.argv) < 2:
    // If no network card is input, use the simulated domain id and the local network card
    ChannelFactoryInitialize(1, "lo")
else:
    // Otherwise, use the specified network card
    ChannelFactoryInitialize(0, sys.argv[1])
```
### unitree_ros2

1. Compile
First, ensure that the unitree_ros2 environment has been properly configured, see [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2).

```bash
source ~/unitree_ros2/setup.sh
cd example/ros2
colcon build
```

2. Run simulation
```bash
source ~/unitree_ros2/setup_local.sh # Use the local network card
export ROS_DOMAIN_ID=1 # Modify the domain id to match the simulation
./install/stand_go2/bin/stand_go2 # Run
```

3. Run real robot
```bash
source ~/unitree_ros2/setup.sh # Use the network card connected to the robot
export ROS_DOMAIN_ID=0 # Use the default domain id
./install/stand_go2/bin/stand_go2 # Run
```
