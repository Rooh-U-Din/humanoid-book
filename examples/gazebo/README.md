# Gazebo Warehouse Simulation Example

This example demonstrates robot simulation in Gazebo with a warehouse environment.

## What This Example Provides

- **Warehouse World**: 10x10m enclosed space with walls and obstacles
- **Launch File**: Automated Gazebo startup with world loading
- **Physics Simulation**: Realistic collision and gravity

## Prerequisites

### Install Gazebo 11 and ROS 2 Gazebo Packages

```bash
# Install Gazebo 11
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev

# Install ROS 2 Gazebo integration packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```

### Verify Installation

```bash
# Check Gazebo version
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x

# Check ROS 2 Gazebo packages
ros2 pkg list | grep gazebo
# Should list gazebo_ros, gazebo_plugins, etc.
```

## Running the Simulation

### Method 1: Using Launch File (Recommended)

```bash
cd ~/my_book/examples/gazebo
ros2 launch warehouse_sim.launch.py
```

This will:
1. Start Gazebo server (physics simulation)
2. Start Gazebo client (GUI)
3. Load the warehouse world

### Method 2: Direct Gazebo Command

```bash
cd ~/my_book/examples/gazebo
gazebo worlds/warehouse.world
```

## What You'll See

The Gazebo GUI will open showing:
- **Ground plane**: Gray floor
- **Warehouse walls**: Four walls forming 10x10m enclosure
- **Obstacles**:
  - Orange box at (3, 3)
  - Blue box at (7, 7)
  - Green cylinder at center (5, 5)

## Interacting with the Simulation

### Camera Controls

- **Rotate**: Left-click + drag
- **Pan**: Shift + Left-click + drag
- **Zoom**: Scroll wheel

### Insert Models

1. Click "Insert" tab on left panel
2. Browse available models (boxes, spheres, robots)
3. Click model to place in world

### Modify Physics

Edit `worlds/warehouse.world` to change:
- **Gravity**: `<gravity>0 0 -9.8</gravity>`
- **Time step**: `<max_step_size>0.001</max_step_size>`
- **Real-time factor**: `<real_time_factor>1.0</real_time_factor>`

## Next Steps: Adding a Robot

To spawn a robot (TurtleBot3, custom robot), you need:

1. **Robot Description** (URDF/SDF file)
2. **Spawn Node** in launch file:

```python
spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'my_robot',
        '-file', 'path/to/robot.urdf',
        '-x', '2.0',
        '-y', '2.0',
        '-z', '0.5'
    ],
    output='screen'
)
```

3. **Gazebo Plugins** for sensors and control (camera, lidar, differential drive)

### Example: TurtleBot3 Integration

```bash
# Install TurtleBot3 packages
sudo apt install -y ros-humble-turtlebot3*

# Set robot model
export TURTLEBOT3_MODEL=burger

# Launch with TurtleBot3 in warehouse
# (Requires custom launch file combining this world with TurtleBot3 spawn)
```

## Troubleshooting

### Problem: Gazebo crashes on launch

**Solution**: Check GPU drivers
```bash
glxinfo | grep OpenGL
# Should show proper GPU renderer
```

### Problem: World not loading

**Solution**: Verify file path in launch file matches `worlds/warehouse.world`

### Problem: Slow simulation (RTF < 1.0)

**Solution**: Reduce physics step size or simplify collision meshes

## File Structure

```
examples/gazebo/
├── worlds/
│   └── warehouse.world          # SDF world file
├── models/
│   └── turtlebot3/             # Robot models (future)
│       └── model.urdf
├── warehouse_sim.launch.py      # ROS 2 launch file
└── README.md                    # This file
```

## Additional Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS 2 + Gazebo Guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)
- [SDF Specification](http://sdformat.org/)
- Book Chapter: [Gazebo Simulation](/docs/modules/gazebo/intro)

---

**License**: MIT
**Maintainer**: Physical AI & Humanoid Robotics Course
