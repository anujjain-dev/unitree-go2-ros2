# Unitree Go2 ROS 2 - CHAMP Controller

> ROS 2 Humble package for the Unitree Go2 quadruped robot using the CHAMP controller framework with Ignition Fortress (Gazebo) simulation. Features teleoperation, 3D LiDAR (Velodyne), IMU, and 2D LiDAR support.

## Unitree Go2:
<div style="display: flex; gap: 50px;">
  <img src="https://oss-global-cdn.unitree.com/static/c487f93e06954100a44fac4442b94d94_288x238.png" width="250" />
  <img src=".docs/gz.png" width="350" /> 
</div>

> Unitree Robotics is focusing on the R&D, production, and sales of consumer and industry-class high-performance general-purpose legged and humanoid robots, six-axis manipulators, and so on. They attach great importance to independent research and development and technological innovation, fully self-researching key core robot components such as motors, reducers, controllers, LIDAR and high-performance perception and motion control algorithms, integrating the entire robotics industry chain.

## CHAMP Controller:
![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

> CHAMP is an open source development framework for building new quadrupedal robots and developing new control algorithms. The control framework is based on [*"Hierarchical controller for highly dynamic locomotion utilizing pattern modulation and impedance control : implementation on the MIT Cheetah robot"*](https://dspace.mit.edu/handle/1721.1/85490).

## Resources:
- [Go2 Description (URDF model)](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description) 
- [CHAMP Robots (configs)](https://github.com/chvmp/robots)
- [CHAMP Controller](https://github.com/chvmp/champ)
- [Velodyne Simulator](https://github.com/rahgirrafi/velodyne_simulator_ros2_gz.git)

## Tested on:
- Ubuntu 22.04 (ROS 2 Humble)
- Ignition Fortress (Gazebo)

## Features:

- ✅ Go2 robot configured with CHAMP controller
- ✅ Robot description ported to ROS 2
    - ✅ URDF with ros2_control tags
    - ✅ Launch files for ROS 2
    - ✅ Configuration files for ROS 2
- ✅ Ignition Fortress simulation
- ✅ RViz visualization
- ✅ Teleoperation support
- ✅ IMU sensor
- ✅ 2D LiDAR (Hokuyo)
- ✅ 3D LiDAR (Velodyne VLP-16)
- ✅ SLAM integration (slam_toolbox) — verified building maps end to end
- 🔶 Nav2 integration — stack runs, plans and drives; goal-reaching is limited by
  leg-odometry drift (see 4.10)
- ✅ Selectable locomotion controller (CHAMP gait controller or an RL policy)

## 1. Installation

### 1.0 Clone the repository:
    
```bash
cd <your_ws>/src
git clone https://github.com/rahgirrafi/unitree-go2-ros2.git
cd unitree-go2-ros2

# Initialize and update the velodyne submodule
git submodule update --init --recursive
```

> **Note:** The velodyne lidar package is included as a git submodule. If you've already cloned the repository without the `--recursive` flag, run `git submodule update --init --recursive` to fetch the submodule.

### 1.1 Install dependencies using rosdep:

```bash
# Install rosdep if not already installed
sudo apt install -y python3-rosdep libignition-msgs8-dev

# Initialize rosdep (only needed once)
sudo rosdep init  # Skip if already initialized
rosdep update

# Install all dependencies
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y
```

### 1.2 Build your workspace:
```bash
cd <your_ws>
colcon build
. <your_ws>/install/setup.bash
```
## 2. Quick Start

You don't need a physical robot to run the following demos. All dependencies will be
installed via rosdep.

> **Every terminal** you use must source the workspace first:
> ```bash
> source /opt/ros/humble/setup.bash
> source <your_ws>/install/setup.bash
> ```

### 2.1 Basic Simulation
Run the Ignition Fortress simulation:
```bash
ros2 launch go2_config gazebo.launch.py
```
This starts Ignition, spawns the Go2, loads the `ros2_control` controllers, and brings up
the CHAMP gait controller and state estimator. The robot stands up after a few seconds
once the controllers are active.

![Go2 Gazebo Launch](.docs/gz.png)

### 2.2 Simulation with RViz
Run the simulation with RViz visualization:
```bash
ros2 launch go2_config gazebo.launch.py rviz:=true
```
![Go2 Gazebo RViz Launch](.docs/gz_rviz.png)

### 2.3 Teleoperation
With the simulation already running, open a **second terminal**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Keep this terminal focused while pressing keys. It publishes to `/cmd_vel`, which the
gait controller converts into footstep trajectories.

### 2.4 Velodyne 3D LiDAR Simulation
Run the simulation with Velodyne VLP-16 3D LiDAR:
```bash
ros2 launch go2_config gazebo_velodyne.launch.py
```
![Go2 Velodyne Gazebo Launch](.docs/velodyne.png)

### 2.5 Velodyne with RViz Point Cloud
Run the Velodyne simulation with RViz to visualize point cloud data:
```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

> **Note:** Set point cloud topic to `/velodyne_points` in RViz.

![Go2 Velodyne Gazebo RViz Launch](.docs/velodyne_rviz_pcl.png)

### 2.6 Hokuyo 2D LiDAR Configuration

The 2D laser scanner is enabled with the `laser:=true` launch argument -- no file editing
or rebuild required.

Add the Hokuyo to the standard simulation:
```bash
ros2 launch go2_config gazebo.launch.py laser:=true
```

Or use it *instead of* the Velodyne VLP-16:
```bash
ros2 launch go2_config gazebo_velodyne.launch.py laser:=true rviz:=true
```

The scanner publishes `sensor_msgs/LaserScan` on `/scan` in the `front_laser` frame,
which is the topic the SLAM and Nav2 configs in `go2_config/config/autonomy/` expect.

> **Note:** Set the LaserScan topic to `/scan` in RViz.

Confirm scans are actually flowing before moving on to SLAM:
```bash
ros2 topic hz /scan          # expect ~50 Hz
ros2 topic echo /scan --once # frame_id should be "front_laser"
```

### 2.7 SLAM (Mapping)

SLAM needs the 2D scanner, so launch with `laser:=true`.

```bash
# Terminal 1 -- simulation
ros2 launch go2_config gazebo.launch.py laser:=true

# Terminal 2 -- slam_toolbox (also brings up Nav2)
ros2 launch go2_config slam.launch.py sim:=true rviz:=true

# Terminal 3 -- drive around to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save it when the coverage looks good:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f <your_ws>/src/unitree-go2-ros2/robots/configs/go2_config/maps/my_map
```

> `sim:=true` sets `use_sim_time`. Omitting it against a simulated robot makes scan
> matching fail, because the timestamps come from `/clock`.

See [section 4](#4-using-slam-and-navigation) for the full workflow, verification steps
and troubleshooting.

### 2.8 Autonomous Navigation

`navigate.launch.py` defaults to `maps/arena.yaml`, a map of `worlds/default.sdf`, so this
works without mapping anything first.

```bash
# Terminal 1 -- simulation
ros2 launch go2_config gazebo.launch.py laser:=true

# Terminal 2 -- Nav2
ros2 launch go2_config navigate.launch.py sim:=true rviz:=true
```

In RViz, place the robot with **2D Pose Estimate** first -- AMCL cannot localise until it
is told roughly where the robot is -- then send it somewhere with the **Nav2 Goal** tool.

Works with either locomotion controller, since Nav2 just publishes `/cmd_vel`.

See [section 4](#4-using-slam-and-navigation) for sending goals without RViz, tuning, and
current limits.

### 2.9 Choosing the Locomotion Controller

Both simulation launch files take a `controller` argument that selects what generates
joint commands:

```bash
# CHAMP gait controller (default, unchanged behaviour)
ros2 launch go2_config gazebo.launch.py controller:=champ

# RL policy trained in IsaacLab
ros2 launch go2_config gazebo.launch.py controller:=model
```

Everything else is identical between the two. The CHAMP state estimator and both EKFs
run in either mode, so `/odom`, TF, SLAM and Nav2 work the same way regardless — the
teleop and navigation sections above apply unchanged:

```bash
ros2 launch go2_config gazebo.launch.py laser:=true controller:=model
ros2 launch go2_config navigate.launch.py sim:=true rviz:=true
```

| | `controller:=champ` | `controller:=model` |
|---|---|---|
| Joint commands from | `quadruped_controller_node` | `rl_policy_node.py` |
| ros2_control controller | `joint_group_effort_controller` (trajectory + PID) | `rl_joint_effort_controller` (group effort) |
| `/cmd_vel` consumer | gait controller | policy, clipped to its trained command range |
| State estimation, EKFs, `/odom`, TF | running | running |

Only one ros2_control controller is loaded, since both claim the same effort command
interfaces — that is what makes the switch exclusive.

---

### 2.10 Running a Different World

`gazebo.launch.py` and `gazebo_velodyne.launch.py` take a `world` argument. It wants an
**absolute path** to an Ignition-format `.sdf`:

```bash
ros2 launch go2_config gazebo.launch.py \
  world:=$(ros2 pkg prefix go2_config)/share/go2_config/worlds/default.sdf
```

> **Also set `world_name`.** Ignition scopes sensor topics by the `<world name="...">`
> declared inside the file and ignores the `<topic>` written on a sensor. The IMU and
> foot-contact bridges rebuild those full topic paths, so a world named anything other
> than `default` needs to be told:
>
> ```bash
> ros2 launch go2_config gazebo.launch.py \
>   world:=/absolute/path/to/warehouse.sdf world_name:=warehouse
> ```
>
> Get it wrong and nothing errors — the IMU and foot contacts just go silent, which
> takes down leg odometry, and with it SLAM and Nav2. Verify with
> `ros2 topic hz /imu/data` (expect ~200 Hz).

The name to pass is the one in the file itself:

```bash
grep -m1 "<world name=" /path/to/your.sdf
```

#### Adding a world to the package

Drop the `.sdf` in `robots/configs/go2_config/worlds/`, then `colcon build
--packages-select go2_config` so it reaches the install space, and launch it by absolute
path as above. If the world `<include>`s models by `model://` URI, put their parent
directory on the resource path first:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/path/to/models
```

#### Spawn pose and GUI

The robot spawns at the origin by default, which is inside a wall in some worlds:

```bash
ros2 launch go2_config gazebo.launch.py \
  world:=/absolute/path/to/your.sdf world_name:=your \
  world_init_x:=2.0 world_init_y:=-1.0 world_init_z:=0.4 world_init_heading:=1.57
```

`headless:=True` runs Ignition with no GUI.

#### About the worlds shipped here

| File | Format | Notes |
|---|---|---|
| `default.sdf` | Ignition | A 12 × 12 m walled arena with five obstacles. The default, and what `maps/arena.yaml` maps. |
| `default.world`, `playground.world`, `outdoor.world` | Gazebo **Classic** | Legacy leftovers. They have no Ignition system plugins and will not load under Fortress — converting them means porting to SDF 1.8 and adding the `ignition-gazebo-*-system` plugins that `default.sdf` declares. |

### 2.11 Launch Argument Reference

| Argument | Default | Applies to | Description |
|----------|---------|------------|-------------|
| `laser` | `false` | `gazebo`, `gazebo_velodyne`, `bringup` | Mount the Hokuyo 2D scanner and publish `/scan`. On `gazebo_velodyne` it *replaces* the VLP-16. |
| `controller` | `champ` | `gazebo`, `gazebo_velodyne`, `bringup` | `champ` for the CHAMP gait controller, `model` for the IsaacLab-trained RL policy |
| `checkpoint` | from `config/rl/policy.yaml` | `gazebo`, `gazebo_velodyne` | rsl_rl checkpoint to run when `controller:=model` |
| `rviz` | `false` | all | Launch RViz alongside the stack |
| `sim` | `false` | `slam`, `navigate`, `bringup` | Use `/clock` simulation time. Set `true` whenever Gazebo is running. |
| `world` | `default.sdf` | `gazebo`, `gazebo_velodyne` | World file to load (see `go2_config/worlds/`) |
| `world_init_x/y/z` | `0.0`/`0.0`/`0.275` | `gazebo`, `gazebo_velodyne` | Robot spawn pose |
| `headless` | `False` | `gazebo`, `gazebo_velodyne` | Run Ignition without the GUI |
| `world_name` | `default` | `gazebo`, `gazebo_velodyne` | The `<world name=...>` inside the world file. Must match, or the IMU and foot-contact topics go silent. |
| `hardware_connected` | `false` | `bringup` | Set `true` when running against the physical robot |

`slam.launch.py` and `navigate.launch.py` take no `laser` argument -- they do not build a
robot description, they only consume `/scan` from whichever simulation or robot is already
running.

## 3. Using Your Own RL Policy

This section is the full recipe for taking a policy you trained in IsaacLab and driving
the robot with it.

### 3.1 What is supported

`rl_policy_node.py` rebuilds the actor network **directly from an rsl_rl checkpoint's
`model_state_dict`**, so neither rsl_rl nor IsaacLab has to be importable at runtime.
Out of the box it supports:

* rsl_rl `OnPolicyRunner` checkpoints — the `model_*.pt` files written into
  `logs/rsl_rl/<experiment>/<run>/`, which contain `actor.*` weights. (A TorchScript
  file exported by `export_policy_as_jit` is *not* what this loads.)
* A plain MLP `ActorCritic` of any layer sizes, with `elu`, `relu` or `tanh`.
* The standard IsaacLab velocity-tracking observation, in this exact order:
  `base_lin_vel(3)`, `base_ang_vel(3)`, `projected_gravity(3)`, `velocity_commands(3)`,
  `joint_pos_rel(12)`, `joint_vel_rel(12)`, `last_action(12)`, `height_scan(N)`.
* A `JointPositionAction` with a scale and `use_default_offset: true`.

Anything outside that — an RNN policy, observation history, a different term order,
`empirical_normalization` — needs a code change, described in [3.5](#35-when-you-need-to-touch-the-code).

The only runtime dependency is PyTorch (`pip install torch`). CPU is plenty enough for inference.
Core i5 10th gen CPU measures ~0.1 ms against a 20 ms budget.

### 3.2 Point it at your checkpoint

The quickest path, with no file editing at all:

```bash
ros2 launch go2_config gazebo.launch.py controller:=model \
  checkpoint:=/path/to/logs/rsl_rl/<experiment>/<run>/model_999.pt
```

To make it the default, edit `checkpoint:` in
`robots/configs/go2_config/config/rl/policy.yaml` — the launch file reads its default
from that file, so there is only one place to change.

If your policy is a drop-in retrain of the same task, this is all you need. If you
changed the environment at all, work through 3.3 before trusting it.

### 3.3 Transfer your training config

Every value the policy depends on lives in `go2_config/config/rl/policy.yaml`. Each one
has a counterpart in the `params/env.yaml` and `params/agent.yaml` that IsaacLab wrote
next to your checkpoint. **Go through this table whenever you retrain** — a mismatch here
does not throw an error, it just produces a robot that walks badly or falls over.

| `policy.yaml` key | Comes from | Notes |
|---|---|---|
| `hidden_dims` | `agent.yaml` → `policy.actor_hidden_dims` | Checked against the checkpoint at startup; a mismatch is a hard error |
| `activation` | `agent.yaml` → `policy.activation` | |
| `policy_rate` | `env.yaml` → `1 / (sim.dt × decimation)` | e.g. `1/(0.005 × 4)` = 50 Hz |
| `action_scale` | `env.yaml` → `actions.joint_pos.scale` | Target is `q_default + scale × action` |
| `default_joint_pos` | `env.yaml` → `scene.robot.init_state.joint_pos` | Regex patterns there; expand them **in policy joint order** (see 3.4) |
| `kp`, `kd`, `torque_limit` | `env.yaml` → `scene.robot.actuators.<group>` | `stiffness`, `damping`, `effort_limit` |
| `max_lin_vel_x/y`, `max_ang_vel_z` | `env.yaml` → `commands.base_velocity.ranges` | `/cmd_vel` is clipped to these; the policy never saw anything outside them |
| `height_scan_dim` | `env.yaml` → `scene.height_scanner.pattern_cfg` | `(size_x/res + 1) × (size_y/res + 1)`; e.g. 1.6×1.0 at 0.1 → 17 × 11 = **187** |
| `height_scan_constant` | measured, see 3.4 | |

Check `agent.yaml` says `empirical_normalization: null` and
`policy.actor_obs_normalization: false`. If either is on, the checkpoint carries a
running mean/std that must be applied to the observation before inference, and the node
does not do that yet.

### 3.4 The two settings that are easy to get wrong

**Joint order.** IsaacLab orders an articulation's joints breadth-first — all four hips,
then all four thighs, then all four calves — and names them `FL/FR/RL/RR`. This repo uses
CHAMP names. `ros_joint_names` in `policy.yaml` holds the mapping, index by index:

| Policy index | IsaacLab | This repo |
|---|---|---|
| 0–3 | `FL_hip`, `FR_hip`, `RL_hip`, `RR_hip` | `lf_hip_joint`, `rf_hip_joint`, `lh_hip_joint`, `rh_hip_joint` |
| 4–7 | `FL_thigh`, `FR_thigh`, `RL_thigh`, `RR_thigh` | `lf_upper_leg_joint`, `rf_upper_leg_joint`, `lh_upper_leg_joint`, `rh_upper_leg_joint` |
| 8–11 | `FL_calf`, `FR_calf`, `RL_calf`, `RR_calf` | `lf_lower_leg_joint`, `rf_lower_leg_joint`, `lh_lower_leg_joint`, `rh_lower_leg_joint` |

`default_joint_pos` must be expanded in this same order, and the `joints:` list of
`rl_joint_effort_controller` in
`go2_description/config/ros_control/ros_control.yaml` must match it too — that list
defines which effort in the published array goes to which joint.

If you are unsure of your own asset's ordering, print it once in IsaacLab with
`env.scene["robot"].data.joint_names` and copy it. Getting this wrong gives you a robot
that thrashes, not one that limps.

**Height scan.** If your environment has a height scanner, those rays are usually most of
the observation (187 of 235 in the rough task) and Gazebo has no equivalent sensor. Every
world shipped here is a flat ground plane, where the entire grid collapses to one
constant:

```
height_scan_constant = walking_base_height − 0.5
```

The 0.5 is the `offset` argument of `mdp.height_scan`. Measure the first term by standing
the robot up and reading `ros2 topic echo /odom_ground_truth` — this policy walks at
0.40 m, hence `−0.10`. It is worth doing: going from `−0.15` to `−0.10` took forward
tracking from 0.15 m/s to 0.44 m/s against a 0.5 m/s command and cut lateral drift over
12 s from 2.3 m to 0.5 m.

If your policy has **no** height scanner (the flat task, for example), set
`height_scan_dim: 0` and the term disappears from the observation.

> **On a world with real uneven terrain the constant is wrong** and the policy is
> effectively blind, since it reports flat ground everywhere. That case needs a real
> ray-cast sensor bridged into the node — replace `self.height_scan` in
> `rl_policy_node.py` with live sensor data.

### 3.5 When you need to touch the code

The node builds its observation in one place — `policy_step()` in
`go2_config/scripts/rl_policy_node.py`. Edit that function if your `env.yaml`'s
`observations.policy` block lists different terms, or the same terms in a different
order.

The startup check catches a *size* mismatch and refuses to run:

```
policy expects a 235-dim observation but the configured terms build 48
```

It cannot catch a *reordering* — same width, wrong meaning — so compare your
`observations.policy` block against the order listed in 3.1 by eye.

### 3.6 Run it and check it worked

```bash
ros2 launch go2_config gazebo.launch.py controller:=model
```

Then, in a second terminal, walk down this list:

```bash
# 1. Both controllers active? (not "unconfigured")
ros2 control list_controllers

# 2. IMU alive? Must be ~100 Hz. Silence here makes the policy think it is
#    always perfectly level, and it will flip over as soon as it tilts.
ros2 topic hz /imu/data

# 3. Policy loaded and commanding? Look for "loaded policy from ..." then
#    "stand-up complete, policy taking over"
ros2 topic hz /rl_joint_effort_controller/commands

# 4. Standing? z should settle around 0.3-0.4 m with small roll/pitch
ros2 topic echo /odom_ground_truth --once

# 5. Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3.7 Troubleshooting

| Symptom | Cause |
|---|---|
| Node exits: `policy expects a N-dim observation but the configured terms build M` | `height_scan_dim` wrong, or your observation has different terms — see 3.3 and 3.5 |
| Node exits: `checkpoint hidden dims [...] do not match configured [...]` | `hidden_dims` does not match `agent.yaml` |
| Node exits: `joint missing from /joint_states` | A name in `ros_joint_names` is not in the URDF |
| Robot lies there inert, no torques | Controllers stuck `unconfigured` — see the note below |
| Robot thrashes violently | Joint order wrong (3.4) |
| Robot walks, then flips onto its back | `/imu/data` not publishing — check with `ros2 topic hz /imu/data` |
| Robot belly-flops and never gets up | `stand_kp`/`stand_kd` too soft for the spawn pose |
| Walks, but weakly or drifting sideways | `height_scan_constant` off (3.4), or the policy is simply undertrained |

> **Controller startup race.** `champ_gazebo` waits a fixed 5–6 s for
> `controller_manager` before loading controllers. On a slow start that race is lost and
> they stay `unconfigured`, leaving the robot inert. Recover without relaunching:
> ```bash
> for c in joint_states_controller rl_joint_effort_controller; do
>   ros2 control set_controller_state $c inactive
>   ros2 control set_controller_state $c active
> done
> ```

### 3.8 How the node works

Two loops run at different rates, mirroring how the policy was trained:

| Loop | Rate | Does |
|------|------|------|
| Policy | 50 Hz (`sim.dt` × `decimation`) | Builds the observation, runs the actor, turns actions into joint position targets (`q_default + scale × a`) |
| PD | 200 Hz | `τ = kp·(q* − q) − kd·q̇`, clipped to `±torque_limit`, published to `rl_joint_effort_controller` |

The PD loop runs faster than the policy because IsaacLab drove the joints with an
implicit PD actuator at the physics rate; holding one torque for a whole 20 ms policy
step visibly under-damps the legs.

On startup the robot eases into the trained default stance over 2 s before the policy
takes over, so its first observation is one it has actually seen. That stand-up phase
uses its own stiffer gains (`stand_kp`/`stand_kd`, default 100/2): Gazebo spawns the
robot with every joint at its URDF zero, which is a sprawl, and it lands on its belly —
the trained gains are far too soft to push it back up, because IsaacLab never had to,
having initialised the robot already standing.

Two supporting sensor fixes came out of building this, and both benefit the CHAMP path:

* **`/imu/data` never published anything.** Ignition scopes sensor topics by world, model
  and link and ignores the `<topic>` on the sensor, so the bridge has to ask for
  `/world/<world>/model/<robot>/link/base_link/sensor/imu_sensor/imu` — bridging a bare
  `/imu/data` silently yields no messages at all. The policy needs it for
  `projected_gravity`. CHAMP defaults to `orientation_from_imu:=false`, which is why this
  went unnoticed.
* **Ground-truth base velocity** comes from an Ignition `OdometryPublisher` added to the
  description and bridged to `/odom_ground_truth`, since the IMU gives only orientation
  and angular velocity. Its twist is reported in the body frame, which is what the policy
  was trained on (verified against numerically differentiated world-frame motion).

### 3.9 Verified behaviour

Measured in the default world with `model_999.pt`, commanding 0.5 m/s forward for 12 s:
the robot stands up, walks 4.58 m (vx steady 0.41–0.45 m/s), drifts 0.54 m laterally and
stays upright throughout (roll ≈ 3°, pitch ≈ 3°, base height ≈ 0.40 m). It also walks
backwards, strafes and turns.

Velocity tracking is loose, and that is the policy rather than the bridge — this training
run finishes at `Metrics/base_velocity/error_vel_xy` ≈ 0.42 m/s and `error_vel_yaw` ≈
0.47 rad/s, with 9% of episodes still ending in a base-ground contact. Train longer if
you want tighter tracking.

Only the simulation paths are wired for `controller:=model`; the policy has not been run
against physical hardware.

## 4. Using SLAM and Navigation

Section 2 gives the two short recipes. This section is the full reference: what each
launch file actually starts, how to drive it without RViz, how it combines with the RL
controller, and what to do when it misbehaves.

### 4.1 How the pieces fit together

| Launch file | Starts | Where `map -> odom` comes from |
|---|---|---|
| `slam.launch.py` | slam_toolbox **and** the Nav2 stack (planner, controller, behaviours, BT navigator) | slam_toolbox, live |
| `navigate.launch.py` | `map_server`, AMCL **and** the Nav2 stack | AMCL, against a saved map |

Both take the same four arguments:

| Argument | Default | Meaning |
|---|---|---|
| `sim` | `false` | Sets `use_sim_time`. **Always `true` against Gazebo** |
| `rviz` | `false` | Opens the matching RViz config |
| `params_file` | `go2_config/config/autonomy/navigation.yaml` | Nav2 parameters |
| `map` | `go2_config/maps/arena.yaml` | `navigate.launch.py` only |
| `slam_params_file` | `go2_config/config/autonomy/slam.yaml` | `slam.launch.py` only |

Neither builds a robot description or starts a controller — they only consume `/scan`,
`/odom` and TF from whatever simulation or robot is already running. So the simulation
always comes first, and it needs `laser:=true`.

### 4.2 Mapping a world

**Terminal 1 — simulation:**

```bash
ros2 launch go2_config gazebo.launch.py laser:=true
```

**Terminal 2 — SLAM:**

```bash
ros2 launch go2_config slam.launch.py sim:=true rviz:=true
```

**Terminal 3 — drive:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Confirm it is actually mapping before driving far:

```bash
ros2 topic hz /map                   # map updates arriving
ros2 run tf2_ros tf2_echo map odom   # slam_toolbox is publishing the map->odom link
```

Then drive the **whole** space, slowly. Two reasons: AMCL later has to localise against
this map and does that badly if half the walls were never seen, and most of the drift in
the finished map comes from the feet slipping during fast turns.

**Saving.** `map_saver_cli` writes `<name>.pgm` and `<name>.yaml` into the current
directory unless you give it a path:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f <your_ws>/src/unitree-go2-ros2/robots/configs/go2_config/maps/my_map
```

That writes into `src/`, so run `colcon build --packages-select go2_config` for it to
reach the install space. To skip the rebuild, either save straight into
`<your_ws>/install/go2_config/share/go2_config/maps/`, or just pass the map by absolute
path when navigating.

Maps that ship with the package:

| Map | Matches | Notes |
|---|---|---|
| `arena.yaml` | `worlds/default.sdf` | The default. Built with the workflow above; imperfect, see 4.10 |
| `map.yaml`, `playground.yaml` | — | Older maps kept for reference; they match no world that loads under Fortress |

### 4.3 Navigating a saved map

```bash
# Terminal 1
ros2 launch go2_config gazebo.launch.py laser:=true

# Terminal 2
ros2 launch go2_config navigate.launch.py sim:=true rviz:=true
```

With your own map:

```bash
ros2 launch go2_config navigate.launch.py sim:=true rviz:=true \
  map:=/absolute/path/to/my_map.yaml
```

**Set the initial pose first.** AMCL starts at `initial_pose` from `navigation.yaml` (the
origin) and cannot localise until it is told roughly where the robot really is. In RViz
use **2D Pose Estimate**; click at the robot's position and drag in the direction it
faces. The laser scan should snap onto the map's walls. Then send it somewhere with the
**Nav2 Goal** tool.

`navigation.rviz` includes the Nav2 panel and goal tool. `slam.rviz` does not — it is a
plain map-and-laser view.

### 4.4 Sending goals without RViz

Three options, all equivalent.

**Publish a pose.** `bt_navigator` subscribes to `/goal_pose`:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

**Call the action** — this one blocks and reports the result, which is what you want in a
script or a test:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}}}}' --feedback
```

**Set the initial pose from the command line**, the equivalent of the RViz tool:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0},
    orientation: {w: 1.0}}}}'
```

**From Python**, using the action client:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

ac = ActionClient(node, NavigateToPose, "navigate_to_pose")
ac.wait_for_server()

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = 2.0
goal.pose.pose.position.y = 1.0
goal.pose.pose.orientation.w = 1.0
future = ac.send_goal_async(goal)
```

For a series of goals, `waypoint_follower` is already running:
`ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints ...`.

### 4.5 Navigating while still mapping

`slam.launch.py` starts the full Nav2 stack alongside slam_toolbox, so you can send goals
without a saved map — slam_toolbox supplies `map -> odom` in place of AMCL, and no
initial pose is needed. The catch is that `slam.rviz` has no goal tool, so either send
goals as in 4.4, or run RViz with the navigation config instead:

```bash
ros2 launch go2_config slam.launch.py sim:=true          # rviz off
rviz2 -d $(ros2 pkg prefix champ_navigation)/share/champ_navigation/rviz/navigation.rviz
```

Goals still have to land in already-mapped space — see 4.9.

### 4.6 Navigation with the RL controller

Nothing changes. Nav2 publishes `/cmd_vel`, and neither the gait controller nor the RL
policy cares where it came from. The state estimator and both EKFs run in either mode, so
`/odom` and TF are identical:

```bash
ros2 launch go2_config gazebo.launch.py laser:=true controller:=model
ros2 launch go2_config navigate.launch.py sim:=true rviz:=true
```

The one difference: the policy clips `/cmd_vel` to the command range it was trained on
(±1 m/s, ±1 rad/s). Nav2's own limits in `navigation.yaml` are lower than that, so they
bind first.

### 4.7 Topics, actions and frames

| | Name | Notes |
|---|---|---|
| Laser in | `/scan` | ~10 Hz, frame `front_laser`. Needs `laser:=true` |
| Odometry in | `/odom` | From champ's EKF chain |
| Velocity out | `/cmd_vel` | Nav2's final output, after `velocity_smoother` |
| Map | `/map` | From slam_toolbox or `map_server` |
| Global plan | `/plan` | |
| Costmaps | `/global_costmap/costmap`, `/local_costmap/costmap` | |
| Goal topic | `/goal_pose` | `geometry_msgs/PoseStamped` |
| Initial pose | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` |
| Navigate action | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Waypoints action | `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` |

Frames: `map -> odom -> base_footprint -> base_link -> front_laser`. Nav2's costmaps and
controller use `base_link`; AMCL and slam_toolbox use `base_footprint`.

Internally Nav2 routes `controller_server -> /cmd_vel_nav -> velocity_smoother ->
/cmd_vel`, so if the robot is not moving, check `/cmd_vel_nav` to tell a controller
problem from a smoother problem.

### 4.8 Where the tuning knobs live

| Want to change | File | Key |
|---|---|---|
| How fast it drives | `config/autonomy/navigation.yaml` | `FollowPath.max_vel_x`, `max_vel_theta` |
| How close counts as arrived | same | `general_goal_checker.xy_goal_tolerance`, `yaw_goal_tolerance` |
| Clearance around obstacles | same | `inflation_layer.inflation_radius`, `robot_radius` |
| How hard it hugs the path | same | `PathAlign.scale`, `PathDist.scale`, `GoalDist.scale` |
| Map resolution, loop closure | `config/autonomy/slam.yaml` | `resolution`, `loop_search_maximum_distance` |
| How often SLAM adds a node | same | `minimum_travel_distance`, `minimum_travel_heading` |
| Localisation confidence | `navigation.yaml` | `amcl.alpha1..alpha5`, `max_particles` |

The Go2 can strafe, but `max_vel_y` is deliberately `0.0` and AMCL uses
`DifferentialMotionModel` — the stack is configured to drive the robot as if it were
non-holonomic, which is more predictable. Raising `max_vel_y` means also switching AMCL to
`nav2_amcl::OmniMotionModel`.

### 4.9 Troubleshooting

| Symptom | Cause |
|---|---|
| No `/map`, log full of `Message Filter dropping message ... 'the timestamp on the message is earlier than all the data in the transform cache'` | TF is lagging the scans — see below |
| `GridBased: failed to create plan with tolerance 0.50` | The goal is in space the map does not cover. `track_unknown_space: true` makes unseen cells unplannable. Pick a goal inside the mapped area, or map more |
| Goal accepted, robot never moves | Check `/cmd_vel_nav` then `/cmd_vel`; if both are silent the controller never produced a command, usually because localisation put the robot somewhere invalid |
| Robot drives confidently to the wrong place | AMCL has lost the pose. Re-set **2D Pose Estimate** and map more of the world |
| Goal rejected immediately | `bt_navigator` is still unwinding a previous aborted goal, or its lifecycle node is not active — check `ros2 lifecycle get /bt_navigator` |
| Costmaps stay empty | `/scan` is not publishing. The simulation needs `laser:=true` |
| Map walls come out smeared or doubled | The SLAM/AMCL base frame is separated from the laser by a moving joint — see below |
| Everything inert after launch | Controllers lost the startup race — see 3.7 |

**Anchor SLAM and AMCL to `base_link`, not `base_footprint`.** The TF tree is
`odom -> base_footprint -> base_link -> front_laser`. Only the last link is static; the
EKF publishes `base_footprint -> base_link` live, and while the robot walks it swings
**±8–10° in roll and ±6–7° in pitch** (measured, and matching Gazebo ground truth — the
body really does rock that much). slam_toolbox and AMCL both treat the laser's offset
from their base frame as rigid, so anchoring them to `base_footprint` feeds them scans
rotated by that swing. `slam.yaml` and `navigation.yaml` therefore both use `base_link`,
which the laser is bolted to by a static transform.

**A caveat that no frame setting fixes.** The laser sits only ~0.32 m above the ground,
so when the body rocks 8° the beam meets the floor about 2.3 m out, and steeper rolls
clip the robot's own legs. Measured on `/scan`: standing still the minimum range never
drops below 1.59 m, but walking drives it to 0.12 m — the sensor's floor — and halves the
median range. Those phantom returns land in the map as obstacles. Reducing them means
reducing the rocking (gait tuning in `gait.yaml`, or stiffer joint gains), raising the
laser, or filtering short returns — not changing frames.

**The TF lag failure** is worth understanding because it silently breaks everything
downstream. slam_toolbox and both costmaps transform each scan at the scan's own
timestamp; if TF is behind, every scan is dropped. Diagnose by comparing the newest TF
stamp against the clock — anything past ~0.1 s of lag on `odom -> base_footprint` does it.
Three settings in this repo prevent it, and are worth knowing if you change the stack:

* `predict_to_current_time: true` in both `champ_base/config/ekf/*.yaml`. Without it
  robot_localization stamps its output at the last fused measurement, which left
  `odom -> base_footprint` about 0.35 s stale and made slam_toolbox drop **every** scan.
* `update_rate: 200` on `joint_states_controller` in
  `go2_description/config/ros_control/ros_control.yaml`. The broadcaster otherwise
  publishes on every controller_manager update (~1 kHz) and robot_state_publisher
  republishes TF on each one — enough traffic to starve the EKF.
* `transform_timeout: 0.5` in `slam.yaml` and `transform_tolerance: 0.5` on both costmaps.

### 4.10 Known limits

Verified working: the stack brings up and activates, both costmaps populate from `/scan`,
SLAM builds maps end to end, `navigate_to_pose` accepts goals, the planner produces paths,
and the controller drives the robot along them — in testing it navigated several metres
and closed to ~1.1 m of a goal while avoiding obstacles.

Not yet reliable is **reaching** an arbitrary goal, and the limiting factor is
localisation rather than Nav2 configuration. CHAMP's leg odometry drifts enough that maps
built by driving the full arena come out visibly skewed — the map saved during development
extends to y ≈ 16 m in a world whose walls stop at y = 6 m — and AMCL then loses the pose
over longer runs, after which the controller drives confidently to the wrong place.

`maps/arena.yaml` ships as the default because it at least corresponds to
`worlds/default.sdf`, unlike the older `map.yaml`. Expect to re-map for anything
demanding, and drive slowly when you do.

Improving this means better odometry, not Nav2 tuning. The most promising options are
fusing the ground-truth `/odom_ground_truth` for simulation work, or tuning champ's
`gait.yaml` odometry scaler so the leg odometry matches reality.

## 5. Tuning Gait Parameters

This applies to `controller:=champ`. The gait configuration for your robot can be found in `go2_config/config/gait/gait.yaml`.

![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/gait_parameters.png)

| Parameter | Unit | Description |
|-----------|------|-------------|
| **Knee Orientation** | - | How the knees should be bent. Configure as `.>>` `.><` `.<<` `.<>` where dot is the front of the robot. |
| **Max Linear Velocity X** | m/s | Robot's maximum forward/reverse speed. |
| **Max Linear Velocity Y** | m/s | Robot's maximum speed when moving sideways. |
| **Max Angular Velocity Z** | rad/s | Robot's maximum rotational speed. |
| **Stance Duration** | s | How long each leg spends on the ground while walking. Default: 0.25s. |
| **Leg Swing Height** | m | Trajectory height during swing phase. |
| **Leg Stance Height** | m | Trajectory depth during stance phase. |
| **Robot Walking Height** | m | Distance from hip to ground while walking. Setting too high can cause instability. |
| **CoM X Translation** | m | Shift reference point in X axis to compensate for uneven weight distribution. |
| **Odometry Scaler** | - | Multiplier for dead reckoning velocities. Typically 1.0 to 1.20. |

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'feat: Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is open source. See individual package licenses for details.

## Acknowledgements

This project builds upon and incorporates work from the following projects:

* [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) - Go2 robot description (URDF model)
* [CHAMP](https://github.com/chvmp/champ) - Quadruped controller framework
* [CHAMP Robots](https://github.com/chvmp/robots) - Robot configurations and setup examples
* [Velodyne Simulator](https://github.com/rahgirrafi/velodyne_simulator_ros2_gz.git) - Velodyne LiDAR simulation for Ignition Fortress

We are grateful to the developers and contributors of these projects for their valuable work.
