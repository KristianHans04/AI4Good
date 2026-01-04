# Robotics for Good Youth Challenge 2025-2026 - Agriculture Robot

This ROS2 codebase implements an autonomous agricultural robot for the Robotics for Good Youth Challenge 2025-2026 (Agriculture Edition). The robot performs cultivation, irrigation, harvesting, and sorting missions on a 2362mm × 1143mm field.

## Hardware Requirements

- **Target Hardware**: HP ZBook 15 G6 (64GB RAM, Quadro T2000 GPU)
- **OS**: Ubuntu 24.04.3 LTS
- **ROS2 Version**: Jazzy Jalisco
- **GPU Acceleration**: Leverages Quadro T2000 for OpenCV/YOLO-based computer vision tasks
- **Adaptability**: Designed for simulation but can be deployed on real robots with:
  - Cameras (RGB for fruit/plot detection)
  - Ultrasonic sensors (for elevated platform detection)
  - Encoders (for odometry)
  - Actuators (for seeding and harvesting)
  - Mechanical gates (for irrigation)

## Architecture

The system is organized into ROS2 packages:

- **sensing**: Computer vision for fruit and plot detection using OpenCV
- **navigation**: Odometry and motion control
- **manipulation**: Seed planting and fruit picking
- **mission_control**: State machine for match logic
- **simulation**: Gazebo world for testing

## Requirements

- Ubuntu 24.04.3 LTS
- ROS2 Jazzy
- OpenCV (python3-opencv)
- Gazebo (for simulation)

## Installation

1. Install ROS2 Jazzy Desktop:
   ```
   sudo apt update
   sudo apt install ros-jazzy-desktop
   ```

2. Install dependencies:
   ```
   sudo apt install python3-colcon-common-extensions python3-opencv
   ```

3. Clone/build workspace:
   ```
   cd ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running

### Simulation
```
ros2 launch gazebo.launch.py  # (if gazebo launch exists)
```

### Real Robot
```
ros2 launch mission_control mission.launch.py
```

### Test Scoring
```
python3 -m pytest src/mission_control/test/test_mission.py
```

## Mission Details

### Cultivation & Irrigation (Mission 1)
- Detect active plots (orange/gray/green) selected by referee
- Plant seeds: small → gray, medium → green, large → orange
- Irrigate only seeded plots to trigger water drop mechanism

### Harvesting & Sorting (Mission 2)
- Identify fruits: red (ripe) → Fruits zone, black (diseased) → Waste zone, green (unripe) → ignore
- Avoid moving green fruits

## Scoring (Senior Category)

### Mission 1
- Correct seed placement: +5
- Seed fully within 2×3 subdivision: +10
- Misplaced seed: -5
- Watering seeded plot: +30
- Watering unseeded plot: -10

### Mission 2
- Moving red/black fruit: +5
- Red fruit in Fruits: +5, in Waste: -5
- Black fruit in Waste: +10, in Fruits: -10
- Moving green fruit: -5

### Penalties
- Unauthorized interaction: -20
- Exiting field: -20
- Damaging structures: -20
- Handing seeds outside start zone: -20

## Code Structure

Each package has:
- `package.xml`: Dependencies
- `setup.py`: Python entry points
- Source files with ROS2 nodes

Key nodes:
- fruit_detector: Publishes fruit counts
- color_sensor: Publishes plot colors
- navigator: Moves robot to coordinates
- odometry: Computes position from encoders
- seed_planter: Releases seeds
- fruit_picker: Picks fruits
- mission_controller: Orchestrates match

## State Machine

States: INIT → SOWING → IRRIGATION → HARVESTING → FINISH

Time-limited to 120 seconds, prioritizing higher-scoring actions (black fruits over red).

## Simulation

Gazebo world includes:
- Field with plots and fruit rows
- Elevated platform for harvesting
- Start zone, Fruits/Waste zones

## Testing

Run tests to validate scoring logic against rulebook.

## Notes

- Autonomous after start, only seed transfer allowed in start zone
- Missions can run in any order/parallel
- Field boundaries enforced in navigation

## Git and Version Control

- Use the provided `.gitignore` to avoid committing build artifacts (build/, install/, log/, etc.)
- Commit only source code in `src/`, documentation, and config files
- Built binaries and installs are regenerated via `colcon build`