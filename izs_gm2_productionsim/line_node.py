import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

from .utils import dumps_json


class LineNode(Node):
    def __init__(self):
        super().__init__('line')

        self.declare_parameter('rate_sec', 1.0)
        self.declare_parameter('defect_prob', 0.25)

        self.rate_sec = float(self.get_parameter('rate_sec').value)
        self.defect_prob = float(self.get_parameter('defect_prob').value)

        self.pub = self.create_publisher(String, '/line/workpiece', 10)
        self._wpno = 0

        self.timer = self.create_timer(self.rate_sec, self._tick)
        self.get_logger().info(f'LineNode started | rate_sec={self.rate_sec}, defect_prob={self.defect_prob}')

    def _tick(self):
        self._wpno += 1
        ts = time.time()

        true_good = (random.random() >= self.defect_prob)
        true_quality = 'GOOD' if true_good else 'BAD'

        payload = {
            "WPNo": self._wpno,
            "Timestamp": ts,
            "true_quality": true_quality
        }

        msg = String()
        msg.data = dumps_json(payload)
        self.pub.publish(msg)

        self.get_logger().info(f'New workpiece WPNo={self._wpno} true_quality={true_quality}')


def main(args=None):
    rclpy.init(args=args)
    node = LineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
