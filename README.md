# Group18 Final Project - Jackal Navigation

This repository contains the final project for **Group18** in the ME5413 course. The project involves autonomous navigation using the Clearpath Jackal robot in a simulated bridge environment.

The main entry point to launch the complete navigation stack is:

```bash
roslaunch jackal_navigation navigation.launch
```

This launch file brings up the full system, including map loading, AMCL-based localization, Move Base for global planning, RViz visualization, and keyboard teleoperation.

## Features

Running `navigation.launch` will start the following components:

- Bridge simulation world (`world.launch`)
- Keyboard teleoperation (`teleop_twist_keyboard`)
- Map server (loads static map `utm30.yaml`)
- AMCL (Adaptive Monte Carlo Localization)
- Move Base (path planning)
- RViz (for visualization)

## Launch Instructions

### 1. Environment Setup

Make sure your workspace is built and sourced:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Run the Main Launch File

```bash
roslaunch jackal_navigation navigation.launch
```

This command will:

- Launch the simulated bridge world (`me5413_world/launch/world.launch`)
- Load the static map `utm30.yaml`
- Start AMCL using parameters from `include/amcl_0405.launch`
- Start the `move_base` path planner
- Launch RViz with a preloaded configuration (`navigation.rviz`)
- Enable keyboard teleoperation

## Directory Structure

```
jackal_navigation/
├── launch/
│   ├── navigation.launch               # Main launch file
│   └── include/
│       ├── amcl_0405.launch            # AMCL parameter configuration
│       └── move_base.launch            # Move Base configuration
├── maps/
│   └── utm30.yaml                      # Static map file
```

## Visualization & Interaction

- Use **“2D Nav Goal”** in RViz to send navigation targets.
- Control the robot manually via keyboard:
  - W: Forward
  - A: Turn left
  - S: Backward
  - D: Turn right

## Notes

- If `teleop_twist_keyboard` is not found, install it with:

```bash
sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard
```

- To switch maps, modify the `map_file` argument in `navigation.launch`.

## Example Output

Once launched successfully, you should see messages like:

```
[INFO] Map server running...
[INFO] AMCL localization started...
[INFO] Move base ready for goals...
[INFO] RViz visualization launched...
```

For advanced usage, goal automation, or integration with bridge-crossing scripts, feel free to expand this README further.
