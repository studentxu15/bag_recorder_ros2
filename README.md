# bag_recorder

[中文](README_zh.md) | [English](README.md)


A ROS 2 node for recording rosbag2 files with support for bag splitting and topic configuration. This package allows you to record specific ROS 2 topics, split bags by size, and configure recording parameters easily.

## Features

- Record specific ROS 2 topics defined in a YAML configuration file
- Automatic bag splitting when reaching maximum size limit
- Configurable recording duration
- Customizable bag storage path
- Topic subscription management with automatic retry
- Support for dynamic topic type detection

## Dependencies

- ROS 2 (Foxy or newer)
- Python 3.8+
- rosbag2_py
- rclpy
- PyYAML

## Installation

1. Create a ROS 2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/your_username/bag_recorder.git
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select bag_recorder
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Basic Usage

Launch the bag recorder with default parameters:
```bash
ros2 launch bag_recorder bag_recorder.launch.py
```

### Custom Configuration

You can modify the recording parameters by passing them as arguments to the launch file:

```bash
ros2 launch bag_recorder bag_recorder.launch.py bag_save_path:=/path/to/save/bags max_bag_size_mb:=200 record_duration:=3600
```

### Configuration File

Edit the `config/record_topics.yaml` file to specify which topics to record:
```yaml
# 需录制的ROS2话题列表（可按需增删）
topics:
  - /livox/lidar
  - /camera/image_raw
  - /odometry
```

## Parameters

| Parameter          | Description                                                                 | Default Value          |
|--------------------|-----------------------------------------------------------------------------|------------------------|
| `config_path`      | Path to the YAML configuration file containing topics to record             | `config/record_topics.yaml` |
| `bag_save_path`    | Root directory where bag files will be saved                                | `~/.ros/bag2`          |
| `record_duration`  | Total recording duration in seconds (-1 for infinite recording)             | `-1`                   |
| `max_bag_size_mb`  | Maximum size of each bag file in megabytes (triggers split when exceeded)   | `100`                  |
| `topic_rate_limit` | Topic frequency limit in Hz (0.0 for no limit)                              | `0.0`                  |

## Bag File Naming

Bag files are named using the start time of the recording and an incrementing index:
```
YYYYMMDD_HHMMSS_<index>
```

Example: `20231015_143022_0`, `20231015_143022_1`, etc.

## Stopping Recording

- If a `record_duration` is set, recording will stop automatically after the specified time
- Press `Ctrl+C` in the terminal to stop recording manually

## License

This package is released under the Apache-2.0 License. See the `LICENSE` file for details.

## Author

Student Xu (xuchunbobo520@gmail.com)

## Troubleshooting

- **Topics not recording**: Check if the topics are correctly specified in the YAML file and if they are being published. The node will keep trying to subscribe to pending topics every second.
- **Bag files not created**: Verify the `bag_save_path` has write permissions and is accessible.
- **Size calculation issues**: The node checks bag size every second. If you experience unexpected splitting, check for large messages that might cause size spikes.