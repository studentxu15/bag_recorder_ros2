# bag_recorder

一个用于录制 rosbag2 文件的 ROS 2 节点，支持数据包分割和话题配置功能。该包允许你录制特定的 ROS 2 话题、按大小分割数据包，并能轻松配置录制参数。

## 功能特点

- 录制 YAML 配置文件中定义的特定 ROS 2 话题
- 达到最大大小限制时自动分割数据包
- 可配置录制时长
- 可自定义数据包存储路径
- 话题订阅管理及自动重试机制
- 支持动态话题类型检测

## 依赖项

- ROS 2 (Foxy 或更新版本)
- Python 3.8+
- rosbag2_py
- rclpy
- PyYAML

## 安装步骤

1. 创建 ROS 2 工作空间（如果没有）：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 克隆本仓库：
```bash
git clone https://github.com/你的用户名/bag_recorder.git
```

3. 构建包：
```bash
cd ~/ros2_ws
colcon build --packages-select bag_recorder
```

4. 激活工作空间：
```bash
source install/setup.bash
```

## 使用方法

### 基本用法

使用默认参数启动数据包录制器：
```bash
ros2 launch bag_recorder bag_recorder.launch.py
```

### 自定义配置

你可以通过向启动文件传递参数来修改录制参数：

```bash
ros2 launch bag_recorder bag_recorder.launch.py bag_save_path:=/path/to/save/bags max_bag_size_mb:=200 record_duration:=3600
```

### 配置文件

编辑 `config/record_topics.yaml` 文件来指定需要录制的话题：
```yaml
# 需录制的ROS2话题列表（可按需增删）
topics:
  - /livox/lidar
  - /camera/image_raw
  - /odometry
```

## 参数说明

| 参数名称           | 描述说明                                                                 | 默认值                  |
|--------------------|--------------------------------------------------------------------------|-------------------------|
| `config_path`      | 包含待录制话题的 YAML 配置文件路径                                       | `config/record_topics.yaml` |
| `bag_save_path`    | 数据包文件保存的根目录                                                   | `~/.ros/bag2`           |
| `record_duration`  | 总录制时长（秒），-1 表示无限期录制                                      | `-1`                    |
| `max_bag_size_mb`  | 每个数据包文件的最大大小（兆字节），超过时触发分割                        | `100`                   |
| `topic_rate_limit` | 话题频率限制（赫兹），0.0 表示无限制                                     | `0.0`                   |

## 数据包命名规则

数据包文件使用录制开始时间和递增索引命名：
```
YYYYMMDD_HHMMSS_<index>
```

示例：`20231015_143022_0`、`20231015_143022_1` 等。

## 停止录制

- 如果设置了 `record_duration`，录制将在指定时间后自动停止
- 在终端中按 `Ctrl+C` 手动停止录制

## 许可证

本包采用 Apache-2.0 许可证发布。详情参见 `LICENSE` 文件。

## 作者

徐同学 (xuchunbobo520@gmail.com)

## 问题排查

- **话题未录制**：检查 YAML 文件中话题是否正确指定以及话题是否正在发布。节点会每秒尝试订阅待订阅的话题。
- **未创建数据包文件**：验证 `bag_save_path` 是否有写入权限且可访问。
- **大小计算问题**：节点每秒检查一次数据包大小。如果遇到意外分割，检查是否有可能导致大小骤增的大型消息。