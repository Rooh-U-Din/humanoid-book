# ROS 2 Talker/Listener Example

This example demonstrates the fundamental **publisher/subscriber** communication pattern in ROS 2.

## What This Example Does

- **Talker Node**: Publishes "Hello World: N" messages to `/chatter` topic at 1 Hz
- **Listener Node**: Subscribes to `/chatter` topic and logs received messages

## Prerequisites

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill (Desktop install)
- **Python**: 3.10+ (included with ROS 2 Humble)

### Install ROS 2 Humble

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### Source ROS 2 (add to ~/.bashrc for persistence)

```bash
source /opt/ros/humble/setup.bash
```

## Installation

### 1. Clone the Repository

```bash
cd ~/
git clone https://github.com/your-username/my_book.git
cd my_book/examples/ros2/talker_listener
```

### 2. Build the Package

```bash
# Create a ROS 2 workspace (if not already created)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Link this example into the workspace
ln -s ~/my_book/examples/ros2/talker_listener .

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select talker_listener

# Source the workspace
source install/setup.bash
```

## Running the Example

### Terminal 1: Run Talker

```bash
source ~/ros2_ws/install/setup.bash
ros2 run talker_listener talker
```

**Expected Output:**
```
[INFO] [talker]: Talker node started - publishing to /chatter at 1 Hz
[INFO] [talker]: Publishing: "Hello World: 0"
[INFO] [talker]: Publishing: "Hello World: 1"
[INFO] [talker]: Publishing: "Hello World: 2"
...
```

### Terminal 2: Run Listener

```bash
source ~/ros2_ws/install/setup.bash
ros2 run talker_listener listener
```

**Expected Output:**
```
[INFO] [listener]: Listener node started - subscribed to /chatter
[INFO] [listener]: I heard: "Hello World: 0"
[INFO] [listener]: I heard: "Hello World: 1"
[INFO] [listener]: I heard: "Hello World: 2"
...
```

### Terminal 3: Inspect with ROS 2 CLI Tools

```bash
# List active nodes
ros2 node list
# Output: /talker, /listener

# List active topics
ros2 topic list
# Output: /chatter, /rosout, ...

# Echo topic data
ros2 topic echo /chatter

# Check topic frequency
ros2 topic hz /chatter
# Output: average rate: 1.000

# Get topic info
ros2 topic info /chatter
# Output: Type: std_msgs/msg/String, Publisher count: 1, Subscription count: 1
```

## Code Walkthrough

### Talker Node (Publisher)

```python
# Create publisher
self.publisher = self.create_publisher(String, 'chatter', 10)

# Create 1 Hz timer
self.timer = self.create_timer(1.0, self.timer_callback)

# Publish in callback
def timer_callback(self):
    msg = String()
    msg.data = f'Hello World: {self.count}'
    self.publisher.publish(msg)
```

### Listener Node (Subscriber)

```python
# Create subscription
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10
)

# Process messages in callback
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```

## Understanding the Pattern

### Communication Flow

```
Talker Node                      Listener Node
    |                                  |
    | publishes to /chatter            |
    |--------------------------------->|
    |                                  | receives message
    |                                  | logs to console
```

### Key Concepts

1. **Asynchronous**: Publisher doesn't wait for subscribers
2. **Many-to-Many**: Multiple publishers and subscribers can use the same topic
3. **Message Type**: Both must agree on `std_msgs/msg/String`
4. **QoS**: Queue size of 10 means last 10 messages buffered

## Troubleshooting

### Problem: "No executable found"

**Solution**: Make sure you sourced the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Problem: Listener not receiving messages

**Solution**: Check if nodes are running:
```bash
ros2 node list  # Should show /talker and /listener
```

### Problem: "ImportError: No module named rclpy"

**Solution**: Source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

## Next Steps

- Try the **Service Example**: `examples/ros2/add_two_ints/`
- Learn about **QoS Profiles**: Modify queue size and reliability
- Explore **Parameters**: Make publish rate configurable
- Check out the book: [ROS 2 Fundamentals](/docs/modules/ros2/fundamentals)

## Additional Resources

- [ROS 2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 std_msgs Package](https://docs.ros.org/en/humble/p/std_msgs/)
- [ROS 2 CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

---

**License**: MIT
**Maintainer**: Physical AI & Humanoid Robotics Course
