# Migration from Gazebo Classic to Ignition Fortress

This document describes the changes made to migrate the workspace from Gazebo Classic to Ignition Fortress (recommended for ROS 2 Humble).

## Overview

Gazebo Classic is deprecated and Ignition Fortress (now called Gazebo) is the recommended simulator for ROS 2 Humble. This migration updates all packages to use the new Ignition APIs and tools.

---

## Package Dependency Changes

### `champ_gazebo/package.xml`

**Removed:**
- `gazebo_ros`
- `gazebo_ros_pkgs`
- `gazebo_plugins`
- `gazebo_ros2_control`

**Added:**
- `ros_gz_sim`
- `ros_gz_bridge`
- `ros_gz_image`
- `gz_ros2_control`

### `champ_gazebo/CMakeLists.txt`

**Removed:**
- `find_package(gazebo_ros2_control REQUIRED)`
- `find_package(gazebo_ros REQUIRED)`
- `${GAZEBO_INCLUDE_DIRS}`
- `${GAZEBO_LIBRARIES}`

**Added:**
- `find_package(ros_gz_sim REQUIRED)`
- `find_package(gz_ros2_control REQUIRED)`

**Note:** The `contact_sensor` executable has been commented out as it uses Gazebo Classic APIs that are not compatible with Ignition.

### `go2_description/package.xml`

**Removed:**
- `gazebo_plugins`

**Added:**
- `ros_gz_sim`
- `gz_ros2_control`

### `go2_config/package.xml`

**Added:**
- `ros_gz_sim`
- `ros_gz_bridge`

---

## Launch File Changes

### `champ_gazebo/launch/gazebo.launch.py`

Complete rewrite for Ignition Fortress:

| Old (Gazebo Classic) | New (Ignition Fortress) |
|---------------------|------------------------|
| `gzserver` command | `ign gazebo -r` command |
| `gzclient` command | Integrated in `ign gazebo` |
| `gazebo_ros/spawn_entity.py` | `ros_gz_sim/create` |
| `libgazebo_ros_init.so` | Not needed |
| `libgazebo_ros_factory.so` | Not needed |

**Added ROS-Ignition bridges for:**
- `/clock` - Clock synchronization
- `/imu/data` - IMU sensor data
- `/scan` - Laser scan data

**Added:**
- `IGN_GAZEBO_RESOURCE_PATH` environment variable
- `RegisterEventHandler` for proper controller loading sequence
- `robot_state_publisher` node

### `champ_config/launch/gazebo.launch.py`

- Updated launch arguments to match new Ignition launch file
- Changed default world from `.world` to `.sdf`
- Removed deprecated arguments (`lite`, `gui`, `close_loop_odom`)
- Added `headless` argument

### `go2_config/launch/gazebo.launch.py`

- Updated launch arguments to match new Ignition launch file
- Changed default world from `.world` to `.sdf`
- Removed deprecated arguments (`lite`, `gui`, `close_loop_odom`)
- Added `headless` and `description_path` arguments

---

## URDF/Xacro Plugin Changes

### `go2_description/xacro/gazebo.xacro`

| Old Plugin | New Plugin |
|-----------|-----------|
| `libgazebo_ros_p3d.so` | `ignition-gazebo-pose-publisher-system` |
| `libgazebo_ros2_control.so` | `gz_ros2_control-system` |
| `libgazebo_ros_imu_sensor.so` | `ignition-gazebo-imu-system` |

### `go2_description/xacro/leg.xacro`

| Old | New |
|-----|-----|
| `gazebo_ros2_control/GazeboSystem` | `gz_ros2_control/GazeboSimSystem` |

### `go2_description/xacro/laser.xacro`

| Old | New |
|-----|-----|
| Sensor type: `ray` | Sensor type: `gpu_lidar` |
| `<ray>` element | `<lidar>` element |
| `libgazebo_ros_ray_sensor.so` | `ignition-gazebo-sensors-system` |

### `go2_description/xacro/velodyne.xacro`

| Old | New |
|-----|-----|
| Sensor type: `ray` | Sensor type: `gpu_lidar` |
| `<ray>` element | `<lidar>` element |
| `libgazebo_ros_velodyne_laser.so` | `ignition-gazebo-sensors-system` |

### `champ_description/urdf/leg.urdf.xacro`

| Old | New |
|-----|-----|
| `gazebo_ros2_control/GazeboSystem` | `gz_ros2_control/GazeboSimSystem` |

### `champ_description/urdf/hokuyo_utm30lx.urdf.xacro`

| Old | New |
|-----|-----|
| Sensor type: `ray` / `gpu_ray` | Sensor type: `gpu_lidar` |
| `<ray>` element | `<lidar>` element |
| `libgazebo_ros_ray_sensor.so` | `ignition-gazebo-sensors-system` |

---

## World File Changes

New SDF 1.8 world files were created for Ignition Fortress:

### New Files Created:
- `champ_gazebo/worlds/default.sdf`
- `champ_config/worlds/default.sdf`
- `go2_config/worlds/default.sdf`

### Key Differences from `.world` files:

1. **SDF Version:** Updated from `1.6`/`1.7` to `1.8`
2. **Required Plugins:** Added Ignition system plugins:
   - `ignition-gazebo-physics-system`
   - `ignition-gazebo-user-commands-system`
   - `ignition-gazebo-scene-broadcaster-system`
   - `ignition-gazebo-sensors-system`
   - `ignition-gazebo-imu-system`
   - `ignition-gazebo-contact-system`
3. **Material Format:** Changed from Gazebo material scripts to direct RGBA values
4. **Render Engine:** Specified `ogre2` for sensors

---

## Deprecated/Removed Components

### Contact Sensor (`contact_sensor.cpp`)

The contact sensor executable has been **commented out** in CMakeLists.txt because:
- It uses Gazebo Classic transport APIs (`gazebo::transport`)
- It subscribes to `~/physics/contacts` topic which doesn't exist in Ignition
- A new implementation using Ignition transport would be required

**Workaround:** Contact detection can be handled through:
- Ignition's built-in contact system plugin
- ros_gz_bridge to forward contact messages

---

## Installation Requirements

Install the required Ignition Fortress packages:

```bash
# Core Ignition packages
sudo apt install ignition-fortress

# ROS 2 - Ignition integration
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-ros-gz-bridge
sudo apt install ros-humble-ros-gz-image

# ros2_control for Ignition
sudo apt install ros-humble-gz-ros2-control
```

---

## Building and Running

### Build the workspace:

```bash
cd /path/to/ws_unitree_go2
colcon build --symlink-install
source install/setup.bash
```

### Launch the simulation:

**For GO2 robot:**
```bash
ros2 launch go2_config gazebo.launch.py
```

**For CHAMP robot:**
```bash
ros2 launch champ_config gazebo.launch.py
```

**Headless mode (no GUI):**
```bash
ros2 launch go2_config gazebo.launch.py headless:=true
```

---

## Troubleshooting

### Common Issues:

1. **"ign: command not found"**
   - Install Ignition Fortress: `sudo apt install ignition-fortress`

2. **"No rule to make target 'gz_ros2_control'"**
   - Install: `sudo apt install ros-humble-gz-ros2-control`

3. **Robot doesn't spawn**
   - Check that `robot_state_publisher` is publishing to `/robot_description`
   - Verify xacro files parse correctly: `xacro robot.xacro`

4. **No sensor data**
   - Check ros_gz_bridge is running
   - Verify topic names match between Ignition and ROS 2

5. **Controllers don't load**
   - Wait for robot to fully spawn before loading controllers
   - Check ros2_control configuration matches joint names

---

## References

- [Ignition Fortress Documentation](https://gazebosim.org/docs/fortress)
- [ros_gz Repository](https://github.com/gazebosim/ros_gz)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [Migration Guide: Gazebo Classic to Ignition](https://gazebosim.org/docs/fortress/migrating_gazebo_classic_ros2_packages)
