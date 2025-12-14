import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

from .utils import loads_json, dumps_json


class MeasuringDevice(Node):
    def __init__(self):
        super().__init__('device')

        # mint a qualitycheckben: paraméterezhető “alapérték + zaj”
        self.declare_parameter('noise', 0.08)
        self.declare_parameter('base_A_good', 5.0)
        self.declare_parameter('base_B_good', 7.0)
        self.declare_parameter('base_A_bad', 7.0)
        self.declare_parameter('base_B_bad', 5.0)

        self.noise = float(self.get_parameter('noise').value)

        self.pub_A = self.create_publisher(String, '/measurements/device/sensor_A', 10)
        self.pub_B = self.create_publisher(String, '/measurements/device/sensor_B', 10)

        self.sub = self.create_subscription(String, '/line/workpiece', self._on_workpiece, 10)

        self.get_logger().info(f'MeasuringDevice started | noise={self.noise}')

    def _noisy(self, base: float) -> float:
        return float(base) + random.gauss(0.0, self.noise)

    def _on_workpiece(self, msg: String):
        data = loads_json(msg.data)
        wpno = int(data['WPNo'])
        ts = time.time()
        true_quality = data.get('true_quality', 'UNKNOWN')

        if true_quality == 'GOOD':
            base_A = float(self.get_parameter('base_A_good').value)
            base_B = float(self.get_parameter('base_B_good').value)
        else:
            base_A = float(self.get_parameter('base_A_bad').value)
            base_B = float(self.get_parameter('base_B_bad').value)

        vA = self._noisy(base_A)
        vB = self._noisy(base_B)

        payload_A = {"WPNo": wpno, "Timestamp": ts, "true_quality": true_quality, "sensor_A": {"value": vA}}
        payload_B = {"WPNo": wpno, "Timestamp": ts, "true_quality": true_quality, "sensor_B": {"value": vB}}

        outA = String(); outA.data = dumps_json(payload_A)
        outB = String(); outB.data = dumps_json(payload_B)

        self.pub_A.publish(outA)
        self.pub_B.publish(outB)
        self.get_logger().info(f'Published measures WPNo={wpno} A={vA:.3f} B={vB:.3f} true={true_quality}')


def main(args=None):
    rclpy.init(args=args)
    node = MeasuringDevice()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
