import math

from banner_detector.detector import BannerDetector, BannerParams
from bc_common.geometry import pixel_to_ned_offset
from bench_flight_controller.sim_vehicle import SimVehicle
from bench_flight_controller.sim_world import FieldParams, SimWorld, field_layout


def test_vehicle_reaches_setpoint():
    v = SimVehicle(2.0, 0.8, 1.5, math.radians(90))
    v.sp, v.sp_yaw = [3.0, -2.0, -5.0], math.pi / 2
    for _ in range(int(30 / 0.02)):
        v.step(0.02)
    assert math.dist(v.pos, v.sp) < 0.05 and v.speed() < 0.05 and abs(v.heading - math.pi / 2) < 1e-3


def test_rendered_banner_reprojects_to_its_true_position():
    world = SimWorld()
    d = BannerDetector(BannerParams(use_ocr=False), log=lambda *_: None)
    for heading in (0.0, 0.4):
        d.reset()
        frame = world.render(4.0, -6.2, heading, 5.0)
        for _ in range(3):
            b, _, _ = d.detect(frame, "green", 5.0)
        assert b is not None
        dn, de = pixel_to_ned_offset(b["u"], b["v"], 1280, 720, 5.0, heading)
        assert math.hypot(4.0 + dn - 4.3, -6.2 + de + 7.5) < 0.1


def test_blue_board_seen_facing_back():
    f = FieldParams(corridor_gap=2.0, field_yaw_deg=8.0)
    world = SimWorld(field=f)
    bn, be = field_layout(f)["blue_board"]
    d = BannerDetector(BannerParams(use_ocr=False), log=lambda *_: None)
    heading = math.radians(8.0) + math.pi
    n, e = bn - 0.3 * math.cos(heading), be - 0.3 * math.sin(heading)  # 0.3 m before the board, facing it
    frame = world.render(n, e, heading, 5.0)
    for _ in range(3):
        b, diags, _ = d.detect(frame, "blue", 5.0)
    assert b is not None
    dn, de = pixel_to_ned_offset(b["u"], b["v"], 1280, 720, 5.0, heading)
    assert math.hypot(n + dn - bn, e + de - be) < 0.1
