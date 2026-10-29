"""Gazebo downward camera -> /camera/image_raw, restamped on arrival.

ros_gz_bridge publishes the gz camera with Gazebo SIM-time stamps, while flight_controller stamps
/flight_controller/pose with the wall clock (no node uses use_sim_time). target_search pairs every red /
QR frame with the pose at the frame's stamp, so the frame is restamped with this node's (wall) clock when
it arrives, like the SIYI RTSP node does with real frames. The pixels are passed through (rgb8).

Parameters: input_topic (/bc/down_camera/image), output_topic (/camera/image_raw), frame_id (down_camera)
"""
from rclpy.node import Node
from sensor_msgs.msg import Image

from bc_common.ros_utils import IMAGE_QOS, IMAGE_TOPIC, spin


class GzCameraRelay(Node):
    def __init__(self):
        super().__init__("gz_camera_relay")
        src = str(self.declare_parameter("input_topic", "/bc/down_camera/image").value)
        dst = str(self.declare_parameter("output_topic", IMAGE_TOPIC).value)
        self.frame_id = str(self.declare_parameter("frame_id", "down_camera").value)
        self.count = 0
        self.pub = self.create_publisher(Image, dst, IMAGE_QOS)
        self.create_subscription(Image, src, self.on_image, IMAGE_QOS)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"Relaying {src} -> {dst} (restamped with the wall clock)")

    def on_image(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self.count += 1

    def report(self):
        if self.count == 0:
            self.get_logger().warn("No Gazebo camera frames yet: is the world running (not paused) and the "
                                   "bridge up?", throttle_duration_sec=15.0)
        else:
            self.get_logger().debug(f"{self.count / 5.0:.1f} frames/s")
        self.count = 0


def main(args=None):
    spin(GzCameraRelay, args)


if __name__ == "__main__":
    main()
