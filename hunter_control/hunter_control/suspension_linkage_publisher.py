"""Publishes the HUNTeR suspension/steering linkage as RViz markers.

The URDF carries the axles as heave + roll joints (URDF cannot close the
3-link + Panhard loops); this node draws the physical links, Panhard rods,
coil-over dampers (spring scaled to its current length) and tie rods from the
live joint states.

Subscriptions:  joint_states (sensor_msgs/JointState)
Publications:   suspension_linkage/markers (visualization_msgs/MarkerArray, frame base_link)
                suspension_linkage/coilover_lengths (std_msgs/Float64MultiArray)
                  [front_left, front_right, rear_left, rear_right] in metres
"""

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from .suspension_linkage import SuspensionLinkage, helix_points, look_rotation
from .vehicle_config import load_vehicle

MESH = "package://hunter_description/meshes/visual/"
STEEL = ColorRGBA(r=0.35, g=0.36, b=0.37, a=1.0)
DARK = ColorRGBA(r=0.15, g=0.15, b=0.16, a=1.0)
SPRING = ColorRGBA(r=0.60, g=0.11, b=0.09, a=1.0)
CHROME = ColorRGBA(r=0.78, g=0.79, b=0.80, a=1.0)


class LinkagePublisher(Node):
    def __init__(self) -> None:
        super().__init__("suspension_linkage_publisher")
        self.declare_parameter("vehicle_config_file", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("rate", 20.0)
        cfg = load_vehicle(self.get_parameter("vehicle_config_file").value or None)
        self.linkage = SuspensionLinkage(cfg)
        self.co = cfg["suspension"]["linkage"]["coilover"]
        self.joints: Dict[str, float] = {}
        self.create_subscription(JointState, "joint_states", self._on_js, 20)
        self.pub = self.create_publisher(MarkerArray, "suspension_linkage/markers", 5)
        self.pub_len = self.create_publisher(Float64MultiArray, "suspension_linkage/coilover_lengths", 5)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._publish)

    def _on_js(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def _marker(self, mid, mesh, start, end, scale, color, stamp):
        m = Marker()
        m.header.frame_id = self.get_parameter("frame_id").value
        m.header.stamp = stamp
        m.ns = "suspension_linkage"
        m.id = mid
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = MESH + mesh
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = start
        q = look_rotation(start, end)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color = color
        return m

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        mid = 0
        lengths = {}
        for e in self.linkage.elements(self.joints):
            if e.kind in ("rod", "tie_rod"):
                arr.markers.append(self._marker(mid, "link_rod.stl", e.start, e.end,
                                                (e.length, e.diameter, e.diameter), STEEL, stamp))
                mid += 1
            else:  # coil-over: body at the chassis end, rod at the axle end, spring between the seats
                lengths[e.name] = e.length
                arr.markers.append(self._marker(mid, "coilover_body.stl", e.start, e.end, (1.0, 1.0, 1.0), DARK, stamp))
                arr.markers.append(self._marker(mid + 1, "coilover_rod.stl", e.end, e.start, (1.0, 1.0, 1.0), CHROME, stamp))
                spring = Marker()
                spring.header.frame_id = self.get_parameter("frame_id").value
                spring.header.stamp = stamp
                spring.ns = "suspension_linkage"
                spring.id = mid + 2
                spring.type = Marker.LINE_STRIP
                spring.action = Marker.ADD
                spring.pose.orientation.w = 1.0
                spring.scale.x = self.co["wire_diameter"]
                spring.color = SPRING
                spring.points = [Point(x=p[0], y=p[1], z=p[2]) for p in helix_points(e, self.co)]
                arr.markers.append(spring)
                mid += 3
        self.pub.publish(arr)
        order = ("front_coilover_left", "front_coilover_right", "rear_coilover_left", "rear_coilover_right")
        self.pub_len.publish(Float64MultiArray(data=[lengths.get(k, 0.0) for k in order]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinkagePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
