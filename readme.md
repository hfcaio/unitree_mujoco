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
- `rt/lf/lowstate` (`LowState`) and `rt/lf/sportmodestate` (`SportModeState`): the low-frequency variants of `rt/lowstate` and `rt/sportmodestate` (ROS 2: `/lf/lowstate`, `/lf/sportmodestate`), published for robots whose definition lists the `lf_lowstate` / `lf_sportmodestate` components (Go2, G1, G1-D, B2 with Z1), by default every 10th bridge tick
- `rt/utlidar/cloud`, `rt/utlidar/cloud_deskewed`, `rt/utlidar/foot_position` (`sensor_msgs/PointCloud2`), `rt/utlidar/imu` (`sensor_msgs/Imu`), `rt/utlidar/robot_odom` (`nav_msgs/Odometry`), `rt/utlidar/robot_pose` (`geometry_msgs/PoseStamped`), `rt/utlidar/lidar_state` (`unitree_go::LidarState_`): the Go2 L1 lidar (ray cast against the scene) and state-estimation topics (ROS 2: `/utlidar/...`), see [Perception topics](#perception-topics-go2-l1-lidar-rtutlidar)
- `rt/brainco/{left,right}/cmd` (`unitree_go::MotorCmds_`), `rt/brainco/{left,right}/state` (`unitree_go::MotorStates_`) and `rt/brainco/{left,right}/touch` (`unitree_hg::PressSensorState_`, simulator only): BrainCo Revo2 hands on the G1 (`-r g1_brainco`), see [G1 with BrainCo hands](#g1-with-brainco-revo2-hands)
- `rt/dex3/{left,right}/cmd` (`unitree_hg::HandCmd_`), `rt/dex3/{left,right}/state` and `rt/lf/dex3/{left,right}/state` (`unitree_hg::HandState_`): Unitree Dex3-1 hands on the G1 (`-r g1_dex3`), see [G1 with Dex3-1 hands](#g1-with-unitree-dex3-1-hands)
- `rt/z1/cmd` (`unitree_go::MotorCmds_`) and `rt/z1/state` (`unitree_go::MotorStates_`): a Unitree Z1 arm on the B2 (`-r b2_z1`). **Simulator-defined topics**: the real Z1 has no DDS interface, see [B2 with Z1 arm](#b2-with-unitree-z1-arm)

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

### Robots and scenes
`-s` is looked up in the robot's scene directory; a wrong name prints the scenes that exist. Robots with a definition file in `simulate/robots/`:

| Run | What |
|---|---|
| `-r go2 -s scene.xml` | Go2 (plus `rt/utlidar/*`) |
| `-r b2_z1 -s scene.xml` | B2 with a Z1 arm and gripper |
| `-r g1 -s scene_29dof.xml` (or `scene_23dof.xml`) | G1 |
| `-r g1_brainco -s scene_29dof.xml` | G1 with BrainCo Revo2 hands |
| `-r g1_dex3 -s scene_29dof.xml` | G1 with Dex3-1 hands |
| `-r g1_d -s scene.xml` | G1-D (wheeled base, lifting column, arms) |

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

### G1 with Unitree Dex3-1 hands
```bash
./unitree_mujoco -r g1_dex3 -s scene_29dof.xml
```
`simulate/robots/g1_dex3.yaml` is built like `g1_brainco`: the plain G1 scenes (`scene_dir: g1`), a Dex3-1 hand attached to each wrist at load time, G1 motors mapped by name so `rt/lowcmd` / `rt/lowstate` keep the 29-motor numbering, and all G1 topics as for `-r g1`. The hands use the topics of the real robot (`unitree_sdk2` `example/g1/dex3/g1_dex3_example.cpp`, `unitree_ros2` `example/src/src/g1/dex3`), per hand:

| Topic | Type | Content |
|---|---|---|
| `rt/dex3/<side>/cmd` | `unitree_hg::msg::dds_::HandCmd_` | 7 `motor_cmd` in the order `[thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]`; each motor applies `tau + kp * (q - q_meas) + kd * (dq - dq_meas)` (rad, rad/s, N m), clamped to the joint's effort limit |
| `rt/dex3/<side>/state` | `unitree_hg::msg::dds_::HandState_` | 100 Hz: 7 `motor_state` (`q`, `dq`, `tau_est` = joint actuator torque), 9 `press_sensor_state`, `imu_state` (palm IMU) |
| `rt/lf/dex3/<side>/state` | `unitree_hg::msg::dds_::HandState_` | the same message at 10 Hz (the SDK example subscribes to this one) |

`mode` of a `motor_cmd` is the packed `RIS_Mode_t` of the example (bits 0-3 id, bits 4-6 status, bit 7 timeout): a motor whose status is 0 (also before the first command) gets zero torque, any other status enables the PD law; id and timeout are ignored. Commands are not CRC-checked (`HandCmd_` has no CRC). In the state, `mode` = `id | enabled << 4`. ROS 2 clients see `/dex3/<side>/cmd`, `/dex3/<side>/state`, `/lf/dex3/<side>/state` with `unitree_hg/msg/HandCmd` / `HandState`.

Component parameters (`{type: dex3_hand, side: left}`): `side` (required), `prefix` (names of the attached hand's elements, default `<side>_`), `topic` (default `rt/dex3/<side>`), `hz` (default 100), `lf_hz` (default 10, 0 disables the `rt/lf/...` topic).

Tactile: the hand model has 9 `touch` sensors per hand (`<side>_hand_press_0..8`, boxes on the inner side of the links). Their normal force in N is published in `press_sensor_state[i].pressure[0]`; the other 11 pressure entries, all `temperature` entries and `lost` stay 0. They are computed inside `mj_step`. Order: `press_sensor_state[0..2]` thumb (`thumb_1` link, `thumb_2` proximal half, `thumb_2` distal half), `[3..5]` middle (`middle_0` link, `middle_1` proximal half, `middle_1` distal half), `[6..8]` index (same), proximal to distal.

Sources and what was guessed:
- Topic names, `HandCmd_` / `HandState_` types, 7 motors and their order, 9 pressure sensors, the `mode` bit layout, the `rt/lf/dex3/<side>/state` topic: the SDK example and the unitree_ros2 example (`motor_cmd.resize(7)`, `press_sensor_state.resize(9)`, `RIS_Mode_t`), `unitree_hg` message definitions. Joint limits, effort limits, masses, inertias and kinematics: `unitree_ros` `robots/dexterous_hand_description/dex3_1/dex3_1_{l,r}.urdf`. Wrist mounting (palm at `(0.0415, +-0.003, 0)` of the wrist yaw link, wrist yaw inertia without the rubber hand): `robots/g1_description/g1_29dof_with_hand.urdf`.
- Guessed: the 100 Hz / 10 Hz rates (no reference states them); that status 0 means unpowered; the tactile layout and which link areas the 9 sensors cover (nothing in the references documents it), and that only `pressure[0]` of each is used; `power_v`, `system_v`, `device_v` = 24 and `power_a` = 0.1 (constants), `ddq`, `temperature`, `vol`, `sensor`, `motorstate`, `error`, `reserve` = 0; the palm IMU is a MuJoCo IMU at the palm frame origin; no state publication timeout or command watchdog. Dynamics (joint damping 0.01, armature 0.002, friction loss 0.001, geometry friction) are tuning values; the armature keeps the explicit 1 kHz PD with `kd` = 0.1 stable on the light finger links.
- The `thumb_1` limits of the URDF (-0.611 .. 1.047 left, -1.047 .. 0.611 right) differ from the constants in the SDK example (-0.724 .. 1.05 left, -1.05 .. 0.742 right); the model uses the URDF.

The hand model is `unitree_robots/components/dex3_1/{left,right}.xml` (BSD-3-Clause like `unitree_ros`; meshes decimated, about 1.3 MB for both hands). Collision uses the (decimated) link meshes, and a box for `thumb_1` as in the URDF.

### B2 with Unitree Z1 arm
```bash
./unitree_mujoco -r b2_z1 -s scene.xml      # or scene_terrain.xml
```
`simulate/robots/b2_z1.yaml` uses the plain B2 scenes (`scene_dir: b2`, so `-r b2` is unchanged) and attaches a Z1 arm with the Unitree gripper on the B2's back when the model is loaded (see [Model composition](#model-composition-attachments)). The B2 keeps the `unitree_go` IDL and its 12-motor numbering on `rt/lowcmd` / `rt/lowstate` (explicit `motors:` map); `rt/sportmodestate`, `rt/wirelesscontroller` are as for `-r b2`, and the definition adds `rt/lf/lowstate` / `rt/lf/sportmodestate`.

**The real Z1 has no DDS / ROS 2 topics.** What Unitree publishes for it:
- [z1_controller](https://github.com/unitreerobotics/z1_controller) + [z1_sdk](https://github.com/unitreerobotics/z1_sdk): the controller talks UDP to the arm (`config/config.xml`: 192.168.123.110:8881) and the SDK talks UDP to the controller (`include/unitree_arm_sdk/message/arm_common.h`: packed `SendCmd` / `RecvState` structs, `unitreeArm::sendRecv()` at 500 Hz, `dt` = 0.002). Its ROS interface (`sim/IOROS.cpp`) is ROS 1 and only drives Gazebo (`/z1_gazebo/JointNN_controller/command`, `unitree_legged_msgs/MotorCmd`).
- [z1_ros](https://github.com/unitreerobotics/z1_ros): ROS 1 Noetic (MoveIt, `z1_hw` joint trajectory controller, gripper action server); its `z1_sdk` is again UDP (`unitree_arm_common.h` `ArmCmd` / `ArmState`, `dt` = 0.004).
- Neither unitree_sdk2 nor unitree_ros2 mentions the Z1. The only ROS 2 driver found is the community [idra-lab/z1_ros2](https://github.com/idra-lab/z1_ros2) (ros2_control on top of the UDP SDK; `trajectory_msgs` / `control_msgs` types that unitree_sdk2 does not ship).

So the simulator exposes the arm the way Unitree exposes other add-on motors on DDS (as the BrainCo hand service does), with types unitree_sdk2 ships. These topic names and this message layout are a **decision of this simulator, not a real-robot interface**; a client written against them will not talk to a real Z1:

| Topic | Type | Content |
|---|---|---|
| `rt/z1/cmd` | `unitree_go::msg::dds_::MotorCmds_` | 7 `cmds`, order `[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`. Per motor: `mode` 0 = unpowered (zero torque), any other value = PD law `tau + kp * (q - q_meas) + kd * (dq - dq_meas)` with `q` rad, `dq` rad/s, `tau` N m, `kp` N m/rad, `kd` N m s/rad; the result is clamped to the effort limit (30 N m, joint2 60 N m, gripper 30 N m). `reserve` is ignored |
| `rt/z1/state` | `unitree_go::msg::dds_::MotorStates_` | 7 `states` in the same order, every 2nd bridge tick (500 Hz nominal; about 460 Hz measured, the bridge ticks at about 930 Hz): `mode` = 1 when the PD law is active else 0, `q` (rad), `dq` (rad/s), `ddq` (rad/s^2), `tau_est` (N m, joint actuator torque); `q_raw`, `dq_raw`, `ddq_raw`, `temperature`, `lost`, `reserve` = 0 |

ROS 2 clients see `/z1/cmd` and `/z1/state` with `unitree_go/msg/MotorCmds` / `MotorStates`. Before the first command every motor is unpowered: the arm lies folded (all joints 0, the Z1 rest pose) with joint2 and joint3 on their limits. No gravity compensation is added: as in z1_sdk's `LOWCMD` state (`examples/lowcmd_development.cpp` computes it with `armModel->inverseDynamics` and sends it as `tau`), the client provides it; without it the arm sags by gravity torque / `kp` (about 0.012 rad on joint2 with `kp` 600). The last command is held (no timeout), joint targets are not clamped to the joint range, and no CRC is checked (`MotorCmds_` has none). Component parameters (`{type: z1_arm, ...}`): `prefix` (names of the attached arm's elements, default `z1_`), `topic` (default `rt/z1`), `hz` (default 500).

Gains: the units are those of the current z1_ros low-level command (standard joint gains; its `doc/controller.md` notes that the 25.6 / 0.0128 factors of the older z1_controller were removed; its `example_lowcmd.cpp` uses `Kp = {500, 600, 500, 400, 300, 200}`, `Kd = 5`). The older z1_sdk gains (`kp = {20, 30, 30, 20, 15, 10}`, `kd = 2000`) are scaled: multiply `kp` by 25.6 and `kd` by 0.0128 (`z1_controller/sim/IOROS.cpp`) to get SI values. With the B2 scene's 2 ms time step and the explicit PD evaluated by the bridge, `kd` up to about 8 N m s/rad is stable; at 10 and above the wrist (joint6) oscillates, so the older SDK's default `kd` (25.6 N m s/rad) is too stiff here. A gripper with `kp` 20, `kd` 0.5 works.

Verified with a client that stands the B2 with `rt/lowcmd` (`b2_stand_example` gains `kp` 1000, `kd` 10, target `[0, 0.67, -1.3]` per leg) and drives the arm through `rt/z1/cmd` (z1_ros gains, target `[0.8, 1.5, -1.0, -0.54, 0.5, 1.0]`, gripper -1 then 0): the B2 stands at 0.62 m (base IMU height, as with the plain B2) with roll / pitch below 0.005 rad while the arm moves; joints reach their targets within 0.016 rad (joint2 / joint3 sag, no gravity compensation sent); the gripper opens to -1.000 and closes to -0.002 rad; with all motors set to `mode` 0 the arm falls onto its joint limits.

Sources and decisions:
- From the references: the joint and motor order (z1_sdk `LowlevelCmd` / `LowlevelState`: 6 joints, gripper last), the PD law and SI gain units (z1_ros `z1_controller/sim/IOROS.cpp` forwards `q`, `dq`, `tau`, `Kp`, `Kd` unchanged to unitree_ros `unitree_legged_control`, whose `computeTorque` evaluates `Kp (q_target - q) + Kd (dq_target - dq) + tau`), the gripper range (-pi/2 open .. 0 closed, z1_sdk `unitreeArm.h`), kinematics, limits (joint1 +-150 deg, joint2 0..170, joint3 -165..0, joint4 +-87, joint5 +-77, joint6 +-160, gripper -90..0), effort limits (30 N m, joint2 60), masses, inertias, joint damping / friction: unitree_ros `robots/z1_description/xacro/{robot.xacro,const.xacro}` with `UnitreeGripper:=true` (`xacro/z1.urdf` is the variant without gripper). The mount convention (arm base flange on top of the trunk, facing forward, no rotation): `robots/aliengoZ1_description`.
- Decided here: that the arm is on DDS at all, the topic names, the `MotorCmds_` / `MotorStates_` types, `mode` 0 = unpowered, the 500 Hz state rate (the z1_sdk loop rate), `ddq` from the simulation, no watchdog. The mount position on the B2 (`pos: [0.05, 0, 0.088]` in `base_link`): z = top plate of the B2 `base_link` mesh, x chosen so the folded arm clears the lidar dome (the aliengoZ1 offset `trunk_length / 2 - 0.07` would put the folded gripper into it); no B2 + Z1 description was found. Joint armature 0.01 kg m^2 (not measured), the gripper collision boxes (fitted to the gripper meshes), and the contact exclusion between link02 and link06 (their URDF cylinders overlap in the folded pose).

The arm model is `unitree_robots/components/z1/z1.xml` (BSD-3-Clause like `unitree_ros`, `LICENSE` in that directory; visual meshes are the URDF's collision STLs decimated to about 2000 triangles, about 1 MB in total; the URDF's DAE visual meshes are not used). Names are prefixed `z1_` by `b2_z1.yaml`: joints `z1_joint1` .. `z1_joint6`, `z1_jointGripper`, actuators `z1_joint1` .. `z1_joint6`, `z1_gripper`, sensors `z1_<actuator>_{pos,vel,torque}`, and a site `z1_ee` between the gripper jaws (convenience, not from the URDF).

### Perception topics (Go2 L1 lidar, `rt/utlidar/*`)
A real Go2 publishes, next to the motor topics, the topics of its L1 lidar and of its state estimator. The simulator publishes the ones whose type ships with unitree_sdk2 and whose content can be derived from the simulation; request/response services (`rt/api/*`) are out of scope. They are enabled in `simulate/robots/go2.yaml`:
```yaml
components:
  - {type: utlidar_cloud, hz: 15, points: 1440}  # rt/utlidar/cloud + rt/utlidar/cloud_deskewed
  - utlidar_imu            # rt/utlidar/imu
  - utlidar_lidar_state    # rt/utlidar/lidar_state
  - utlidar_robot_odom     # rt/utlidar/robot_odom
  - utlidar_robot_pose     # rt/utlidar/robot_pose
  - utlidar_foot_position  # rt/utlidar/foot_position
```
The topic list of a real Go2 is the `ros2 topic list` screenshot in the unitree_ros2 README (`docs/image/piFtteJ.png`). For each topic (DDS name = `rt` + ROS 2 name):

| ROS 2 topic | Type (unitree_sdk2 header) | Carries | Simulated |
|---|---|---|---|
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` (`idl/ros2/PointCloud2_.hpp`) | L1 point cloud, frame `utlidar_lidar` | yes, `utlidar_cloud` (ray casting) |
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | motion-compensated cloud | yes, `utlidar_cloud`: the same scan in the world frame `odom` (a simulated scan is taken at one instant, so it has no motion distortion) |
| `/utlidar/foot_position` | `sensor_msgs/PointCloud2` | foot positions | yes, `utlidar_foot_position` |
| `/utlidar/imu` | `sensor_msgs/Imu` (`idl/ros2/Imu_.hpp`) | the lidar's IMU | yes, `utlidar_imu` |
| `/utlidar/lidar_state` | `unitree_go/LidarState` (`idl/go2/LidarState_.hpp`) | lidar status | yes, `utlidar_lidar_state` |
| `/utlidar/robot_odom` | `nav_msgs/Odometry` (`idl/ros2/Odometry_.hpp`) | estimated base odometry | yes, `utlidar_robot_odom` (ground truth) |
| `/utlidar/robot_pose` | `geometry_msgs/PoseStamped` (`idl/ros2/PoseStamped_.hpp`) | estimated base pose | yes, `utlidar_robot_pose` (ground truth) |
| `/utlidar/height_map`, `/utlidar/height_map_array` | probably `unitree_go/HeightMap` (`idl/go2/HeightMap_.hpp`) | elevation map around the robot | no: which topic uses the type, and the map's size, resolution, origin and filling rules are not documented; it would be invented |
| `/utlidar/grid_map`, `/utlidar/range_map`, `/utlidar/range_info`, `/utlidar/voxel_map`, `/utlidar/map_state` | not documented | maps / ranges of the on-board mapping | no: type unknown |
| `/utlidar/voxel_map_compressed` | `unitree_go/VoxelMapCompressed` (`idl/go2/VoxelMapCompressed_.hpp`) | compressed voxel map | no: the compression format is not documented |
| `/utlidar/switch`, `/utlidar/mapping_cmd` | not documented | commands to the lidar / mapping (inputs) | no: type and semantics unknown |
| `/uwbstate` | `unitree_go/UwbState` (`idl/go2/UwbState_.hpp`) | UWB follow tag (remote) state | no: there is no tag in the simulation, constant values would be meaningless |
| `/uwbswitch` | `unitree_go/UwbSwitch` | enable UWB (input) | no, as `/uwbstate` |
| `/servicestate`, `/servicestateactivate`, `/selftest` | not documented | status of the robot's internal services / self test | no: describes on-board services that the simulator does not have (the SDK reads the service list through the `robot_state` RPC, out of scope) |
| `/frontvideostream` | `unitree_go/Go2FrontVideoData` | front camera, H.264 | no: needs off-screen rendering and video encoding |
| `/audiosender`, `/audioreceiver` | `unitree_go/AudioData` | microphone / speaker audio | no: no audio in the simulation |
| `/webrtcreq`, `/webrtcres`, `/rtc_status`, `/public_network_status`, `/gptflowfeedback` | not documented | app / WebRTC / cloud connectivity and the voice assistant | no: not robot state |
| `/parameter_events`, `/rosout` | ROS 2 standard | created by every ROS 2 node on the robot | no: infrastructure of ROS 2 nodes, any ROS 2 tool creates them itself |

G1: the topics of a real G1 found in unitree_sdk2 / unitree_ros2 besides the motor topics are `rt/secondary_imu`, `rt/lf/lowstate`, `rt/lf/sportmodestate`, `rt/lf/bmsstate` and the hand topics, all already simulated. The G1 carries a Livox MID-360 lidar and a RealSense D435 (`mid360_link` / `d435_link` in the unitree_ros G1 URDFs), but no repository documents topics for them (they come from the vendors' ROS drivers, not from unitree DDS), so the G1 definitions enable no perception component. `utlidar_cloud` can still be mounted on any robot, for example a MID-360-like pattern on the G1 torso (topic and frame of your choice):
```yaml
  - {type: utlidar_cloud, body: torso_link, pos: [0.0002835, 0.00003, 0.40618], rpy: [0, 0.0401426, 0],
     v_fov: [-7, 52], points: 4000, hz: 10, range: [0.1, 40], topic: rt/mid360/cloud, frame_id: mid360_link, deskewed_topic: ""}
```

**`utlidar_cloud`** casts rays from the lidar mount against the MuJoCo scene (geoms of groups 0-2, i.e. the scene and the robot's visual meshes, except the mount body) and publishes the hits. Message layout as on the robot (unitree_ros2 README / `docs/image/piFtdF1.png`): `height` 1, `is_dense` true, `point_step` 32, 6 fields `x`, `y`, `z` (float32 at 0, 4, 8), `intensity` (float32 at 16), `ring` (uint16 at 20), `time` (float32 at 24, seconds since the scan start), frame `utlidar_lidar`, about 1400 points per message at 15 Hz (the screenshot shows 1395-1405 points 67 ms apart). Like the L1's non-repetitive scan, each scan casts `points` rays along a low-discrepancy sequence over the field of view (uniform per solid angle) that continues from scan to scan, so successive scans fill the field of view. With the default 1440 rays a scan has about 1350 hits on `scene.xml` and 1390 on `scene_terrain.xml` (robot lying on the floor; rays above the horizon have no return). `intensity` is the luminance of the hit geom's colour (0-255), `ring` an elevation bin (`rings` bins), `time` the ray's index times scan period / `points` (the whole scan uses one snapshot of the state). The mount is the site `utlidar` added to `unitree_robots/go2/go2.xml` at the pose of the `radar` link of the unitree_ros `go2_description` (0.28945, 0, -0.046825 from `base_link`, pitch 2.8782 rad: the dome faces forward-down). Parameters: `site` (default `utlidar`) or `body` + `pos` + `rpy`; `hz` (15); `points` (1440 rays per scan); `h_fov` ([-180, 180] deg, azimuth about the sensor z axis); `v_fov` ([0, 90] deg, elevation above the sensor xy plane; the L1 FOV is 360 x 90 deg); `range` ([0.05, 30] m); `rings` (18); `noise` (0, range std. dev. in m); `geomgroups` ([0, 1, 2]); `exclude_body` (default the mount body, `none` to hit it); `topic` (`rt/utlidar/cloud`); `frame_id` (`utlidar_lidar`); `deskewed_topic` (`rt/utlidar/cloud_deskewed`, `""` to disable); `deskewed_frame_id` (`odom`).

Threading: the bridge thread is not the physics thread, and `mj_ray` / `mj_multiRay` use the `mjData` stack, so the lidar never casts rays on the simulation's `mjData`. It runs in its own thread at `hz`: it takes the physics mutex (the one `mj_step` runs under) only to copy `qpos` and the mocap poses into a private `mjData`, releases it, then runs `mj_kinematics` and `mj_multiRay` on the private copy. The physics thread waits at most for that copy, and the 1 kHz bridge tick does not run the scan. Cost (MuJoCo 3.3.6): a ray against primitives and height fields is cheap (1440 rays in 0.2-0.6 ms), a ray that reaches the bounding box of a mesh is expensive (about 10 us per ray near the G1's meshes). The default Go2 scan takes 1-3 ms (it faces away from the robot's meshes); the G1 example above takes about 50 ms per 4000 rays. If a scan takes longer than `1 / hz`, the next one starts right away (the rate drops, one core stays busy); `geomgroups` can leave out mesh groups. On the test machine `-r go2 -s scene_terrain.xml` runs at real time with the default lidar and with 20000 rays at 30 Hz, with the bridge tick rate unchanged.

**Other components** (each has `hz` and `topic`; rates are those of the bridge tick divider, like `lf_*`):
- `utlidar_imu` (default 250 Hz, frame `utlidar_imu`): the base IMU sensors (`imu_quat`, `imu_gyro`, `imu_acc`) rotated into the lidar frame (site `site`, default `utlidar`, on the IMU's body); orientation = lidar frame in the world; the lever-arm terms of the acceleration are ignored.
- `utlidar_robot_odom` (50 Hz, frame `odom`, child `base_link`) and `utlidar_robot_pose` (50 Hz, frame `odom`): the `base_link` pose in the world (from the same IMU-site sensors as `rt/sportmodestate`, moved to the body origin); odometry twist in the child frame. Ground truth: no drift, unlike the robot's estimator. Parameters `frame_id`, `child_frame_id`.
- `utlidar_foot_position` (50 Hz, frame `odom`): one point per foot (x, y, z float32, `point_step` 12) at the foot body origins (`bodies`, default `[FR_foot, FL_foot, RR_foot, RL_foot]`, the Unitree leg order). The foot bodies are posed on a private `mjData` refreshed with a non-blocking `try_lock` of the physics mutex.
- `utlidar_lidar_state` (1 Hz): `cloud_frequency` (measured), `cloud_size` (points of the last scan), `cloud_scan_num`, `imu_frequency`, `imu_rpy` from the two components above; versions `sim`, `error_state` 0, the other fields 0. The SDK's `LidarState_` has a `dirty_percentage` field that unitree_ros2's `LidarState.msg` lacks; the simulator publishes the SDK type.

Stamps use the wall clock, as on the robot (no `/clock`). What is taken from the references: the topic list, the types of `/utlidar/cloud`, `/utlidar/cloud_deskewed` and `/utlidar/foot_position`, the cloud's frame, layout size, point count and rate (unitree_ros2 README and its screenshots), the lidar mount (unitree_ros URDF), the message definitions (unitree_sdk2 IDL). Assumed: the types of `/utlidar/imu`, `/utlidar/robot_odom`, `/utlidar/robot_pose`, `/utlidar/lidar_state` (from their names and the SDK IDL), the field names and offsets of the cloud (the layout of the L1 SDK's point type, consistent with the 32-byte, 6-field screenshot), the frames other than `utlidar_lidar`, the foot cloud layout, all rates except the cloud's, the intensity model.

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

Robots without a definition file keep the previous behaviour (`unitree_hg` robots get both components above). `idl_type` in `config.yaml` or `-t` still overrides `idl`. Components are registered in `simulate/src/components.h` (perception components in `simulate/src/perception_components.h`).

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
