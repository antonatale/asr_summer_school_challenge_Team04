#!/usr/bin/env python3
"""Convert detections at their image timestamp, allowing bounded TF delivery delay."""
import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from apriltag_msgs.msg import AprilTagDetectionArray
from landmark_msgs.msg import Landmark, LandmarkArray


class TagLandmarks(Node):
    def __init__(self):
        super().__init__('tag_landmarks')
        self.base = self.declare_parameter('robot_base_frame', 'base_link').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=30)
        self.dropped = 0
        self.pub = self.create_publisher(LandmarkArray, '/camera/landmarks', 10)
        self.create_subscription(AprilTagDetectionArray, '/camera/detections', self.receive,
                                 qos_profile_sensor_data)
        self.create_timer(.02, self.flush)

    def receive(self, msg):
        self.pending.append((time.monotonic(), msg, list(msg.detections)))

    def flush(self):
        for _ in range(len(self.pending)):
            received, msg, detections = self.pending.popleft()
            result = LandmarkArray()
            result.header.stamp = msg.header.stamp
            result.header.frame_id = self.base
            missing = []
            for tag in detections:
                try:
                    tf = self.buffer.lookup_transform(self.base, f'{tag.family}:{tag.id}',
                                                       Time.from_msg(msg.header.stamp))
                except TransformException:
                    missing.append(tag)
                    continue
                item = Landmark()
                item.id = tag.id
                item.hamming = tag.hamming
                item.goodness = tag.goodness
                item.decision_margin = tag.decision_margin
                t = tf.transform.translation
                item.range = math.hypot(t.x, t.y)
                item.bearing = math.atan2(t.y, t.x)
                result.landmarks.append(item)
            if result.landmarks or not detections:
                self.pub.publish(result)
            if missing:
                if time.monotonic()-received < .5:
                    self.pending.append((received, msg, missing))
                else:
                    self.dropped += len(missing)
                    self.get_logger().warning(f'Tag TF at image timestamp unavailable; dropped={self.dropped}',
                                               throttle_duration_sec=5.)


def main():
    rclpy.init()
    node = TagLandmarks()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
