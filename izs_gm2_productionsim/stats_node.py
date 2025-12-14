import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.executors import ExternalShutdownException

from .utils import loads_json, dumps_json


class StatsNode(Node):
    def __init__(self):
        super().__init__('stats')

        self.total = 0
        self.good = 0
        self.bad = 0
        self.correct = 0
        self.wrong = 0

        self.sub = self.create_subscription(String, '/qc/result', self._on_result, 10)
        self.pub = self.create_publisher(String, '/qc/stats', 10)

        self.srv = self.create_service(Trigger, '/qc/reset_stats', self._on_reset)

        self.get_logger().info('StatsNode started | service: /qc/reset_stats')

    def _on_result(self, msg: String):
        data = loads_json(msg.data)
        decision = data.get('decision', 'UNKNOWN')
        true_q = data.get('true_quality', 'UNKNOWN')

        self.total += 1
        if decision == 'GOOD':
            self.good += 1
        elif decision == 'BAD':
            self.bad += 1

        if true_q in ('GOOD', 'BAD'):
            if decision == true_q:
                self.correct += 1
            else:
                self.wrong += 1

        out = {
            "total": self.total,
            "good": self.good,
            "bad": self.bad,
            "accuracy": (self.correct / (self.correct + self.wrong)) if (self.correct + self.wrong) > 0 else None,
        }

        m = String()
        m.data = dumps_json(out)
        self.pub.publish(m)

        self.get_logger().info(f"STATS total={self.total} good={self.good} bad={self.bad}")

    def _on_reset(self, req, resp):
        self.total = self.good = self.bad = self.correct = self.wrong = 0
        resp.success = True
        resp.message = "Stats reset."
        self.get_logger().warn("Stats reset via /qc/reset_stats")
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = StatsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
