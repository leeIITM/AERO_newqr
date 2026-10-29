"""Frame conversions and setpoint building of ardupilot_utils (MAVROS ENU/FLU <-> the nodes' NED)."""
import math

import pytest
from mavros_msgs.msg import PositionTarget

from flight_controller.ardupilot_utils import (
    enu_flu_quaternion_to_rpy, enu_yaw_from_heading, heading_from_enu_quaternion, latlon_offset,
    make_setpoint, offset_latlon, wrap_pi,
)


def yaw_quaternion(yaw):
    """Quaternion (x, y, z, w) of a pure rotation about the up axis."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


@pytest.mark.parametrize("yaw_enu, heading", [
    (math.pi / 2, 0.0),  # facing north
    (0.0, math.pi / 2),  # facing east
    (-math.pi / 2, math.pi),  # facing south
    (math.pi, -math.pi / 2),  # facing west
])
def test_heading_from_enu_quaternion(yaw_enu, heading):
    assert wrap_pi(heading_from_enu_quaternion(*yaw_quaternion(yaw_enu)) - heading) == pytest.approx(0.0, abs=1e-9)


def test_heading_and_enu_yaw_are_inverse():
    for heading in (-3.0, -1.0, 0.0, 0.7, 2.5):
        assert wrap_pi(enu_yaw_from_heading(enu_yaw_from_heading(heading)) - heading) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("yaw_enu, heading_deg", [(math.pi / 2, 0.0), (0.0, 90.0), (-math.pi / 2, 180.0)])
def test_level_attitude_yaw_is_heading(yaw_enu, heading_deg):
    roll, pitch, yaw = enu_flu_quaternion_to_rpy(*yaw_quaternion(yaw_enu))
    assert math.degrees(roll) == pytest.approx(0.0, abs=1e-6)
    assert math.degrees(pitch) == pytest.approx(0.0, abs=1e-6)
    assert math.degrees(wrap_pi(yaw - math.radians(heading_deg))) == pytest.approx(0.0, abs=1e-6)


def test_nose_up_is_positive_pitch_and_right_wing_down_is_positive_roll():
    # Facing east (ENU yaw 0): rotating about the body y axis (left) by -a lowers the nose.
    a = math.radians(10.0)
    nose_up = (0.0, -math.sin(a / 2), 0.0, math.cos(a / 2))  # +x forward rotated up about -y
    _, pitch, _ = enu_flu_quaternion_to_rpy(*nose_up)
    assert pitch == pytest.approx(a, abs=1e-9)
    right_down = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))  # rotation about +x forward: left up, right down
    roll, _, _ = enu_flu_quaternion_to_rpy(*right_down)
    assert roll == pytest.approx(a, abs=1e-9)


def test_latlon_round_trip():
    lat, lon = offset_latlon(13.0, 80.0, 25.0, -40.0)
    north, east = latlon_offset(lat, lon, 13.0, 80.0)
    assert (north, east) == pytest.approx((25.0, -40.0), abs=1e-6)
    assert lat > 13.0 and lon < 80.0


def test_position_setpoint_swaps_ned_to_enu():
    msg = make_setpoint(None, "position", position=(10.0, 4.0, -3.0), heading=0.0)  # north 10, east 4, up 3
    assert (msg.position.x, msg.position.y, msg.position.z) == (4.0, 10.0, 3.0)
    assert msg.yaw == pytest.approx(math.pi / 2)  # facing north = ENU yaw 90 deg
    assert msg.type_mask & PositionTarget.IGNORE_VX and not msg.type_mask & PositionTarget.IGNORE_PX
    assert not msg.type_mask & PositionTarget.IGNORE_YAW and msg.type_mask & PositionTarget.IGNORE_YAW_RATE
    assert msg.coordinate_frame == PositionTarget.FRAME_LOCAL_NED


def test_velocity_setpoint_swaps_ned_to_enu_and_flips_yaw_rate():
    msg = make_setpoint(None, "velocity", velocity=(1.0, 2.0, 0.5), yaw_rate=0.3)  # north 1, east 2, down 0.5
    assert (msg.velocity.x, msg.velocity.y, msg.velocity.z) == (2.0, 1.0, -0.5)
    assert msg.yaw_rate == pytest.approx(-0.3)  # clockwise (NED) -> counter-clockwise positive (ENU)
    assert msg.type_mask & PositionTarget.IGNORE_PX and msg.type_mask & PositionTarget.IGNORE_YAW
    assert not msg.type_mask & PositionTarget.IGNORE_VX and not msg.type_mask & PositionTarget.IGNORE_YAW_RATE


def test_nan_heading_leaves_yaw_alone():
    msg = make_setpoint(None, "position", position=(0.0, 0.0, 0.0), heading=float("nan"))
    assert msg.type_mask & PositionTarget.IGNORE_YAW
