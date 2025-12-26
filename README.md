# legged_hunter_controllers

Controller configurations and launch files for the legged_hunter robot.

## Contents

- `config/hunter_controllers.yaml`: Controller configuration for ros2_control
- `launch/hunter_simulation.launch.py`: Launch file for MuJoCo simulation with ros2_control

## Dependencies

- `legged_hunter_description`: Robot URDF description
- `mujoco_ros2_control`: MuJoCo integration for ros2_control
- `controller_manager`: ROS 2 controller management

## Usage

```bash
ros2 launch legged_hunter_controllers hunter_simulation.launch.py
```
