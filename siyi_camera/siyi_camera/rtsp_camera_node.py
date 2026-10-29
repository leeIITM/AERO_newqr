"""SIYI A8 mini live stream -> /camera/image_raw (continuous; what the detectors read).

Parameters
    camera_src   RTSP URL, video device index ("0") or file; "" = rtsp://<camera_ip>:8554/<stream>.264
    camera_ip    192.168.144.25 (SIYI factory address)
    stream       main | sub
    image_topic  /camera/image_raw
    frame_id     camera
    rtsp_transport    udp (lowest latency) | tcp (no lost packets, so no HEVC reference errors / smeared
                      frames, ~100-200 ms more latency)
    ffmpeg_log_level  quiet (default) | error | warning | info: FFmpeg's own decoder messages, e.g.
                      "[hevc] Could not find ref with POC n" after a lost UDP packet
"""
from rclpy.node import Node
from sensor_msgs.msg import Image

from bc_common.image import bgr_to_imgmsg
from bc_common.ros_utils import IMAGE_QOS, IMAGE_TOPIC, spin
from bc_common.stream import SiyiStreamWorker, default_rtsp_url


class RtspCamera(Node):
    def __init__(self):
        super().__init__("siyi_camera")
        src = str(self.declare_parameter("camera_src", "").value)
        ip = str(self.declare_parameter("camera_ip", "192.168.144.25").value)
        stream = str(self.declare_parameter("stream", "main").value)
        topic = str(self.declare_parameter("image_topic", IMAGE_TOPIC).value)
        self.frame_id = str(self.declare_parameter("frame_id", "camera").value)
        transport = str(self.declare_parameter("rtsp_transport", "udp").value)
        log_level = str(self.declare_parameter("ffmpeg_log_level", "quiet").value)
        src = src or default_rtsp_url(stream, ip)
        self.pub = self.create_publisher(Image, topic, IMAGE_QOS)
        self.worker = SiyiStreamWorker(src, log=self.get_logger().info, transport=transport,
                                       ffmpeg_log_level=log_level)
        self.last_id = 0
        self.create_timer(1.0 / 60.0, self.publish_latest)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"Opening {src} ({transport.upper()}) -> {topic}, FFmpeg log {log_level}")

    def publish_latest(self):
        frame, frame_id = self.worker.get_frame_with_id()
        if frame is None or frame_id == self.last_id:
            return
        self.last_id = frame_id
        msg = bgr_to_imgmsg(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def report(self):
        if not self.worker.is_connected:
            self.get_logger().warn("No camera frames for 2 s (reconnecting).")
        else:
            self.get_logger().debug(f"camera {self.worker.fps:.1f} fps")

    def shutdown(self):
        self.worker.stop()


def main(args=None):
    spin(RtspCamera, args)


if __name__ == "__main__":
    main()
