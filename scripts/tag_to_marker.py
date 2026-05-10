#!/usr/bin/env python3
"""apriltag_msgs/AprilTagDetectionArray → visualization_msgs/Marker for iPhone AR.

apriltag_ros publishes detection poses via TF, not in the detection message.
The iPhone has no TF listener — it only knows /ar_markers — so this node
listens for detections, looks up the corresponding TF pose, converts it from
the OpenCV optical convention (+y down, +z forward) to the iPhone AR
convention (+y up, -z forward), and publishes a green CUBE marker on
/ar_markers under the `camera` frame the iPhone interprets directly.

For RViz, no Marker bridge is needed — apriltag_ros' TF broadcast plus an
`Axes` or `TF` display in RViz already shows each tag's pose.
"""

from __future__ import annotations

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class TagToMarker(Node):
    def __init__(self) -> None:
        super().__init__("tag_to_marker")

        self.declare_parameter("detections_topic", "detections")
        self.declare_parameter("marker_topic", "/ar_markers")
        self.declare_parameter("tag_frame_prefix", "iphone_tag_")
        self.declare_parameter("namespace", "iphone_apriltag")
        self.declare_parameter("marker_id_offset", 1000)
        self.declare_parameter("tag_size", 0.17)
        self.declare_parameter("color_rgba", [0.0, 1.0, 0.0, 0.7])
        self.declare_parameter("lifetime_sec", 2)

        det_topic = self.get_parameter("detections_topic").value
        mk_topic = self.get_parameter("marker_topic").value
        self.tag_prefix = self.get_parameter("tag_frame_prefix").value
        self.ns = self.get_parameter("namespace").value
        self.id_offset = int(self.get_parameter("marker_id_offset").value)
        self.tag_size = float(self.get_parameter("tag_size").value)
        rgba = self.get_parameter("color_rgba").value
        self.color = (float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3]))
        self.lifetime = int(self.get_parameter("lifetime_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timeout = RclpyDuration(seconds=0.1)

        self.create_subscription(AprilTagDetectionArray, det_topic, self._on_detections, 10)
        self.pub = self.create_publisher(Marker, mk_topic, 10)
        self._summarized: set[int] = set()

        self.get_logger().info(
            f"tag_to_marker — {det_topic} → {mk_topic} "
            f"(tag_frames={self.tag_prefix}<id>, ns={self.ns}, id_offset={self.id_offset})"
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        parent = msg.header.frame_id
        for d in msg.detections:
            tag_id = int(d.id)
            child = f"{self.tag_prefix}{tag_id}"
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, Time(), self.tf_timeout)
            except TransformException:
                continue

            t = tf.transform.translation
            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = "camera"
            marker.ns = self.ns
            marker.id = tag_id + self.id_offset
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(t.x)
            marker.pose.position.y = -float(t.y)
            marker.pose.position.z = -float(t.z)
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.tag_size
            marker.scale.y = self.tag_size
            marker.scale.z = 0.005
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.color
            marker.text = f"Tag #{tag_id}"
            marker.lifetime = DurationMsg(sec=self.lifetime, nanosec=0)
            self.pub.publish(marker)

            if tag_id not in self._summarized:
                self._summarized.add(tag_id)
                self.get_logger().info(
                    f"Tag {tag_id}: marker on {self.pub.topic_name} "
                    f"(family={d.family}, decision_margin={d.decision_margin:.1f})"
                )


def main() -> None:
    rclpy.init()
    node = TagToMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
