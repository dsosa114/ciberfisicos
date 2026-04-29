# pendulum_robot_description

ROS 2 package containing the URDF/Xacro description, configuration, launch files, and RViz setup for a simple pendulum robot.

## Robot Description

The robot consists of three links connected in series:

- **`base_link`** — A fixed cubic base mounted to the world frame.
- **`rod_link`** — A cylindrical rod attached to the base via the `shoulder_lift_joint` (continuous revolute joint rotating around the Y axis).
- **`ball_link`** — A spherical mass fixed to the tip of the rod.

The `shoulder_lift_joint` supports both position and velocity command interfaces when ros2_control is activated, and exposes position, velocity, and effort state interfaces.

## Package Structure

```
pendulum_robot_description/
├── config/
│   ├── gz_bridge.yaml        # ROS–Gazebo topic bridge configuration
│   └── parameters.yaml       # Physical parameters (masses, dimensions, limits)
├── launch/
│   ├── display.launch.py     # Visualize the robot in RViz with joint_state_publisher_gui
│   └── gz_sim.launch.py      # Spawn the robot in Gazebo Sim (gz-sim)
├── rviz/
│   └── config.rviz           # Pre-configured RViz layout
└── urdf/
    ├── common_properties.xacro   # Reusable inertia macros and material definitions
    ├── gazebo.xacro              # Gazebo plugins (joint state publisher, ros2_control)
    ├── kinematic_chain.xacro     # Link and joint definitions
    └── pendulum_robot.urdf.xacro # Top-level robot description entry point
```

## Configuration

Robot physical parameters are defined in `config/parameters.yaml`:

| Parameter             | Default value | Description                        |
|-----------------------|---------------|------------------------------------|
| `width`               | 0.1 m         | Base link side length              |
| `base_height`         | 2.0 m         | Height of the base                 |
| `base_mass`           | 100.0 kg      | Mass of the base                   |
| `rod_length`          | 1.0 m         | Length of the pendulum rod         |
| `rod_radius`          | 0.03 m        | Radius of the rod                  |
| `rod_mass`            | 1.0 kg        | Mass of the rod                    |
| `ball_radius`         | 0.05 m        | Radius of the tip ball             |
| `ball_mass`           | 0.5 kg        | Mass of the tip ball               |
| `max_angular_velocity`| 2π rad/s      | Joint velocity limit               |
| `max_effort`          | 10.0 N·m      | Joint effort limit                 |

## Launch Files

### Display in RViz

Launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 for interactive URDF visualization.

```bash
ros2 launch pendulum_robot_description display.launch.py
```

Optional argument:

| Argument               | Default                              | Description                      |
|------------------------|--------------------------------------|----------------------------------|
| `robot_parameters_file`| `config/parameters.yaml`             | Path to robot parameters file    |

### Simulate in Gazebo Sim

Launches Gazebo Sim with the pendulum robot spawned in an empty world, along with `robot_state_publisher` and the ROS–Gazebo bridge.

```bash
ros2 launch pendulum_robot_description gz_sim.launch.py
```

Optional argument:

| Argument               | Default                              | Description                      |
|------------------------|--------------------------------------|----------------------------------|
| `robot_parameters_file`| `config/parameters.yaml`             | Path to robot parameters file    |

To enable ros2_control (required by `pendulum_robot_control`), pass `activate_control:=true` via the xacro argument inside the launch file.

## Dependencies

- `ament_cmake`
- `robot_state_publisher`
- `joint_state_publisher_gui` (display launch only)
- `rviz2`
- `ros_gz_sim` (Gazebo launch only)
- `gz_ros2_control` (when control is activated)

## License

Apache-2.0
