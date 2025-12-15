import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .utils import loads_json


class VizNode(Node):
    def __init__(self):
        super().__init__('viz')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('speed_mps', 0.25)        # szalag sebesség (m/s)
        self.declare_parameter('lane_y', 0.0)            # szalag y pozíció
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('end_x', 4.0)             # meddig fusson a szalagon

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.speed = float(self.get_parameter('speed_mps').value)
        self.lane_y = float(self.get_parameter('lane_y').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.end_x = float(self.get_parameter('end_x').value)

        self.pub = self.create_publisher(MarkerArray, '/viz/workpieces', 10)

        self.sub_wp = self.create_subscription(String, '/line/workpiece', self.on_workpiece, 10)
        self.sub_qc = self.create_subscription(String, '/qc/result', self.on_qc, 10)

        # wpno -> state
        self.items = {}  # {wpno: {"x":..., "t":..., "status": "UNKNOWN|OK|NOK"}}
        self._last = time.time()

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('VizNode started -> publishing MarkerArray on /viz/workpieces')

    def on_workpiece(self, msg: String):
        d = loads_json(msg.data)
        wpno = int(d['WPNo'])
        if wpno not in self.items:
            self.items[wpno] = {"x": self.start_x, "t": time.time(), "status": "UNKNOWN"}

    def on_qc(self, msg: String):
        d = loads_json(msg.data)
        wpno = int(d['WPNo'])
        decision = str(d.get('decision', 'UNKNOWN')).upper()  # GOOD/BAD vagy OK/NOK
        # normalizálás
        if decision in ('GOOD', 'OK'):
            status = "OK"
        elif decision in ('BAD', 'NOK', 'NOT_OK'):
            status = "NOK"
        else:
            status = "UNKNOWN"

        if wpno in self.items:
            self.items[wpno]["status"] = status
        else:
            # ha a QC előbb jönne meg (ritka), akkor is felvesszük
            self.items[wpno] = {"x": self.start_x, "t": time.time(), "status": status}

    def tick(self):
        now = time.time()
        dt = now - self._last
        self._last = now

        # mozgatás
        to_delete = []
        for wpno, it in self.items.items():
            it["x"] += self.speed * dt
            if it["x"] > self.end_x:
                to_delete.append(wpno)
        for wpno in to_delete:
            del self.items[wpno]

        # marker array
        arr = MarkerArray()

        # Szalag (line strip)
        belt = Marker()
        belt.header.frame_id = self.frame_id
        belt.header.stamp = self.get_clock().now().to_msg()
        belt.ns = "belt"
        belt.id = 1
        belt.type = Marker.LINE_STRIP
        belt.action = Marker.ADD
        belt.scale.x = 0.05
        belt.color.a = 1.0
        belt.color.r = 0.7
        belt.color.g = 0.7
        belt.color.b = 0.7

        p1 = Point(); p1.x = self.start_x; p1.y = self.lane_y; p1.z = 0.0
        p2 = Point(); p2.x = self.end_x;   p2.y = self.lane_y; p2.z = 0.0
        belt.points = [p1, p2]
        arr.markers.append(belt)

        # Workpiece-ek (kockák)
        base_id = 1000
        for i, (wpno, it) in enumerate(sorted(self.items.items(), key=lambda x: x[0])):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "workpieces"
            m.id = base_id + int(wpno) % 100000  # legyen stabil int
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = float(it["x"])
            m.pose.position.y = self.lane_y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0

            m.scale.x = 0.25
            m.scale.y = 0.18
            m.scale.z = 0.18

            status = it["status"]
            m.color.a = 1.0
            if status == "OK":
                m.color.r, m.color.g, m.color.b = (0.1, 0.9, 0.1)
            elif status == "NOK":
                m.color.r, m.color.g, m.color.b = (0.95, 0.1, 0.1)
            else:
                m.color.r, m.color.g, m.color.b = (0.2, 0.4, 0.9)

            arr.markers.append(m)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
