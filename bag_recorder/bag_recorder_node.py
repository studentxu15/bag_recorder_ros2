#!/usr/bin/env python3
import os
import yaml
import rclpy
import signal
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosbag2_py import (
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata
)
from datetime import datetime
from threading import Lock


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("bag_recorder_node")

        self.lock = Lock()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('config_path', ''),          # YAML配置路径
                ('bag_save_path', './bags'),  # 根保存路径
                ('record_duration', -1),      # 总录制时长（秒），-1无限
                ('max_bag_size_mb', 500),     # 单包最大大小（MB）
                ('topic_rate_limit', 0.0)     # 话题频率限制（Hz）
            ]
        )

        self.registered_topics = set()

        # 2. 获取参数
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.bag_root_path = self.get_parameter('bag_save_path').get_parameter_value().string_value
        self.record_duration = self.get_parameter('record_duration').get_parameter_value().integer_value
        self.max_bag_size_mb = self.get_parameter('max_bag_size_mb').get_parameter_value().integer_value
        self.max_bag_size_bytes = self.max_bag_size_mb * 1024 * 1024  # 转换为字节

        # 3. 校验配置
        if not os.path.exists(self.config_path):
            self.get_logger().fatal(f"Configuration file does not exist: {self.config_path}")
            rclpy.shutdown()
            return

        # 4. 读取话题列表
        self.topics = self.load_topics_from_yaml()
        if not self.topics:
            self.get_logger().fatal("No valid topics in configuration file!")
            rclpy.shutdown()
            return
        self.get_logger().info(f"Topics to record: {self.topics}")

        # 5. 创建根保存目录
        os.makedirs(self.bag_root_path, exist_ok=True)

        # 6. 初始化变量（切分包核心）
        self.current_bag_writer = None  # 当前bag写入器
        self.current_bag_path = ""      # 当前bag路径
        self.record_start_time = datetime.now()  # 总录制开始时间（用于命名）
        self.bag_index = 0              # 包序号（区分切分后的包）

        # 7. 创建第一个bag文件
        self.create_new_bag()

        # 8. 订阅管理
        self.subscribers = {}
        self.pending_topics = set(self.topics)  # 待订阅话题集合

        # 9. 定时尝试订阅待订阅话题
        self.try_subscribe_timer = self.create_timer(1.0, self.try_subscribe_pending_topics)

        # 10. 启动大小监控定时器（每秒检查一次）
        self.size_check_timer = self.create_timer(1.0, self.check_bag_size)

        # 11. 总录制时长控制
        if self.record_duration > 0:
            self.get_logger().info(f"Total recording duration: {self.record_duration} seconds")
            self.create_timer(self.record_duration, self.stop_recording)

        self.get_logger().info("Bag recording started!")

    def load_topics_from_yaml(self):
        """从YAML加载话题列表"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            topics = config.get('topics', [])
            topics = [t.strip() for t in topics if t.strip()]
            return list(set(topics))  # 去重
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML file: {str(e)}")
            return []

    def get_bag_name(self):
        """生成bag文件名：录制开始时间 + 序号（如20251223_160000_0）"""
        start_time_str = self.record_start_time.strftime("%Y%m%d_%H%M%S")
        return f"{start_time_str}_{self.bag_index}"

    def create_new_bag(self):
        """创建新的bag文件（初始化写入器）"""
        with self.lock:
            # 关闭旧的writer（如果存在）
            if self.current_bag_writer:
                try:
                    self.current_bag_writer = None  # 自动关闭
                except Exception as e:
                    self.get_logger().warn(f"Failed to close old bag: {str(e)}")

            # 生成新bag路径
            bag_name = self.get_bag_name()
            self.current_bag_path = os.path.join(self.bag_root_path, bag_name)
            self.bag_index += 1

            # 初始化新writer
            self.current_bag_writer = SequentialWriter()
            storage_options = StorageOptions(
                uri=self.current_bag_path,
                storage_id='sqlite3'
            )
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            self.current_bag_writer.open(storage_options, converter_options)

            self.registered_topics.clear()

            self.get_logger().info(f"Created new bag file: {self.current_bag_path} (Max size per bag: {self.max_bag_size_mb}M)")

    def get_topic_type(self, topic_name):
        """获取话题类型（如geometry_msgs/msg/Twist）"""
        try:
            topic_info = self.get_topic_names_and_types()
            for name, types in topic_info:
                if name == topic_name and len(types) > 0:
                    return types[0]
            self.get_logger().warn(f"Topic {topic_name} is not published, cannot get type")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to get topic type: {str(e)}")
            return None

    def get_msg_class(self, topic_type):
        """动态导入消息类"""
        try:
            pkg, subfolder, cls_name = topic_type.split('/')
            module = __import__(f"{pkg}.{subfolder}", fromlist=[cls_name])
            return getattr(module, cls_name)
        except Exception as e:
            self.get_logger().error(f"Failed to import message type {topic_type}: {str(e)}")
            return None

    def try_subscribe_pending_topics(self):
        """尝试订阅待定话题"""
        if not self.pending_topics:
            # 全部已订阅，停止定时器（可选）
            self.try_subscribe_timer.cancel()
            return

        topic_info = self.get_topic_names_and_types()
        available_topics = {name: types for name, types in topic_info}

        subscribed_now = []
        for topic in list(self.pending_topics):
            if topic in available_topics and len(available_topics[topic]) > 0:
                topic_type = available_topics[topic][0]
                msg_class = self.get_msg_class(topic_type)
                if not msg_class:
                    self.get_logger().warn(f"Failed to import message type, skip subscribing: {topic}")
                    continue
                # 创建订阅者
                self.subscribers[topic] = self.create_subscription(
                    msg_class,
                    topic,
                    lambda msg, t=topic: self.write_to_bag(msg, t),
                    qos_profile=10
                )
                self.get_logger().info(f"Successfully subscribed to topic: {topic} 【{topic_type}】")
                subscribed_now.append(topic)

        # 从待订阅集合移除已订阅话题
        for topic in subscribed_now:
            self.pending_topics.remove(topic)

    def write_to_bag(self, msg, topic):
        """将消息写入当前bag"""
        if not self.current_bag_writer:
            return
        with self.lock:
            try:
                if topic not in self.registered_topics:
                    # 尝试获取话题类型并注册
                    topic_type = self.get_topic_type(topic)
                    if not topic_type:
                        self.get_logger().warn(f"Cannot get topic type before writing, skip registration: {topic}")
                        return  # 类型未知，暂不写入
                    topic_metadata = TopicMetadata(
                        name=topic,
                        type=topic_type,
                        serialization_format='cdr'
                    )
                    self.current_bag_writer.create_topic(topic_metadata)
                    self.registered_topics.add(topic)
                    self.get_logger().info(f"Registered topic to bag: {topic} 【{topic_type}】")

                timestamp = self.get_clock().now().nanoseconds
                self.current_bag_writer.write(topic, serialize_message(msg), timestamp)
            except Exception as e:
                self.get_logger().error(f"Failed to write to {topic}: {str(e)}")

    def get_directory_size(self, dir_path):
        """计算目录总大小（字节）"""
        total_size = 0
        try:
            for dirpath, dirnames, filenames in os.walk(dir_path):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    if os.path.exists(fp):
                        total_size += os.path.getsize(fp)
        except Exception as e:
            self.get_logger().error(f"Failed to calculate directory size: {str(e)}")
        return total_size

    def check_bag_size(self):
        """检查当前bag大小，超过阈值则新建"""
        if not self.current_bag_path or not os.path.exists(self.current_bag_path):
            return
        # 计算当前bag目录大小
        current_size = self.get_directory_size(self.current_bag_path)
        if current_size >= self.max_bag_size_bytes:
            self.get_logger().info(
                f"Current bag size {current_size/1024/1024:.1f}M ≥ {self.max_bag_size_mb}M, creating new bag"
            )
            self.create_new_bag()

    def stop_recording(self):
        """停止录制，清理资源"""
        self.get_logger().info("Recording finished, cleaning up resources...")
        with self.lock:
            self.current_bag_writer = None  # 关闭当前bag
        self.size_check_timer.cancel()
        if self.try_subscribe_timer:
            self.try_subscribe_timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    stopped = [False]

    def handle_signal(signum, frame):
        if not stopped[0]:
            node.get_logger().info(f"Received signal {signum}, stopping recording...")
            node.stop_recording()
            stopped[0] = True

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGHUP, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)  # Ctrl+C 也可以

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        handle_signal(signal.SIGINT, None)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()