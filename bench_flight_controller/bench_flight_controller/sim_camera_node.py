"""Synthetic downward camera for bench runs without the SIYI A8 mini (camera:=sim).

Renders the SimWorld field from /flight_controller/pose and publishes /camera/image_raw.

Parameters: width (1280), height (720), hfov_rad (1.22, must match the detectors), rate_hz (15),
noise (0.0 grey levels), image_topic, and the field layout (sim_world.FieldParams: green_sway,
corridor_gap, corridor_length, wall_height, arch_height, field_yaw_deg, ...)
"""
from rclpy.node import Node
from sensor_msgs.msg import Image

from bc_common.image import bgr_to_imgmsg
from bc_common.ros_utils import IMAGE_QOS, IMAGE_TOPIC, VehicleListener, spin
from bench_flight_controller.sim_world import FieldParams, SimWorld


class SimCamera(Node):
    def __init__(self):
        super().__init__("sim_camera")
        self.width = int(self.declare_parameter("width", 1280).value)
        self.height = int(self.declare_parameter("height", 720).value)
        self.hfov = float(self.declare_parameter("hfov_rad", 1.22).value)
        self.noise = float(self.declare_parameter("noise", 0.0).value)
        rate = float(self.declare_parameter("rate_hz", 15.0).value)
        topic = str(self.declare_parameter("image_topic", IMAGE_TOPIC).value)
        defaults = FieldParams()
        field = FieldParams(**{n: type(getattr(defaults, n))(self.declare_parameter(n, getattr(defaults, n)).value)
                               for n in FieldParams.names()})
        self.world = SimWorld(field=field)
        self.vehicle = VehicleListener(self)
        self.pub = self.create_publisher(Image, topic, IMAGE_QOS)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Synthetic camera {self.width}x{self.height} @ {rate:.0f} Hz -> {topic} | {field}")

    def tick(self):
        if not self.vehicle.fresh():
            return
        p = self.vehicle.latest
        frame = self.world.render(p.x, p.y, p.heading, self.vehicle.alt_rel(), self.width, self.height,
                                  self.hfov, self.noise)
        msg = bgr_to_imgmsg(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.pub.publish(msg)


def main(args=None):
    spin(SimCamera, args)


if __name__ == "__main__":
    main()
