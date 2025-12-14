import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

from .utils import loads_json, dumps_json


class QualityNode(Node):
    def __init__(self):
        super().__init__('quality')

        self.declare_parameter('diff_threshold', 1.0)
        self.declare_parameter('log_pairs', True)
        self.declare_parameter('drop_after_sec', 10.0)

        self.diff_threshold = float(self.get_parameter('diff_threshold').value)
        self.log_pairs = bool(self.get_parameter('log_pairs').value)
        self.drop_after_sec = float(self.get_parameter('drop_after_sec').value)

        self._buf = {}  # WPNo -> {ts_first, A?, B?, true_quality?}

        self.sub_A = self.create_subscription(String, '/measurements/device/sensor_A', self._on_a, 10)
        self.sub_B = self.create_subscription(String, '/measurements/device/sensor_B', self._on_b, 10)

        self.pub = self.create_publisher(String, '/qc/result', 10)

        self.timer_gc = self.create_timer(1.0, self._gc)

        self.get_logger().info(
            f'QualityNode started | diff_threshold={self.diff_threshold}, drop_after_sec={self.drop_after_sec}'
        )

    def _on_a(self, msg: String):
        self._handle(msg, 'A')

    def _on_b(self, msg: String):
        self._handle(msg, 'B')

    def _handle(self, msg: String, key: str):
        data = loads_json(msg.data)
        wpno = int(data['WPNo'])
        ts = float(data.get('Timestamp', time.time()))
        true_quality = data.get('true_quality', 'UNKNOWN')

        sensor_obj = data.get('sensor_A' if key == 'A' else 'sensor_B', {})
        value = float(sensor_obj.get('value'))

        entry = self._buf.get(wpno)
        if entry is None:
            entry = {'ts_first': time.time(), 'true_quality': true_quality}
            self._buf[wpno] = entry

        entry[key] = {'ts': ts, 'value': value}

        if self.log_pairs:
            self.get_logger().info(f'Got {key} | WPNo={wpno} value={value:.3f} true={true_quality}')

        if 'A' in entry and 'B' in entry:
            self._evaluate_and_publish(wpno, entry)
            del self._buf[wpno]

    def _evaluate_and_publish(self, wpno: int, entry: dict):
        vA = float(entry['A']['value'])
        vB = float(entry['B']['value'])
        diff = vB - vA

        good = bool(diff >= self.diff_threshold)
        decision = 'GOOD' if good else 'BAD'

        out = {
            "WPNo": wpno,
            "Timestamp": max(float(entry['A']['ts']), float(entry['B']['ts'])),
            "sensor_A": entry['A'],
            "sensor_B": entry['B'],
            "diff": diff,
            "diff_threshold": self.diff_threshold,
            "decision": decision,
            "true_quality": entry.get('true_quality', 'UNKNOWN'),
        }

        m = String()
        m.data = dumps_json(out)
        self.pub.publish(m)

        self.get_logger().info(f'QC WPNo={wpno} diff={diff:.3f} -> {decision}')

    def _gc(self):
        now = time.time()
        to_delete = [wpno for wpno, e in self._buf.items() if (now - e.get('ts_first', now)) > self.drop_after_sec]
        for wpno in to_delete:
            del self._buf[wpno]


def main(args=None):
    rclpy.init(args=args)
    node = QualityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
