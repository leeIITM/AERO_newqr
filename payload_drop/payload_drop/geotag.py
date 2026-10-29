"""Geotagging and drop helpers without ROS (unit-tested).

EXIF layout identical to airdrop_survey's add_geotag_metadata: GPS latitude/longitude/altitude/image
direction, image width/height, and UserComment = "ASCII\\0\\0\\0" + JSON {"Pitch", "Roll", "Yaw"} in degrees
0-360. Written with piexif when it is installed (as airdrop_survey), else with Pillow (same tags).
"""
import json
import math
from typing import Callable, Optional

USER_COMMENT_PREFIX = b"ASCII\x00\x00\x00"


def decimal_to_dms(decimal):
    """Degrees -> EXIF rationals [(deg, 1), (min, 1), (sec*100, 100)] (airdrop_survey's conversion)."""
    dd = abs(decimal)
    degrees = int(dd)
    minutes_full = (dd - degrees) * 60
    minutes = int(minutes_full)
    seconds = round((minutes_full - minutes) * 60 * 100)
    if seconds >= 6000:  # rounding carried into the next minute
        seconds -= 6000
        minutes += 1
        if minutes >= 60:
            minutes -= 60
            degrees += 1
    return [(degrees, 1), (minutes, 1), (int(seconds), 100)]


def dms_to_decimal(value):
    d = float(value[0][0]) / float(value[0][1])
    m = float(value[1][0]) / float(value[1][1])
    s = float(value[2][0]) / float(value[2][1])
    return d + (m / 60.0) + (s / 3600.0)


def wrap_deg(angle_rad):
    """rad -> degrees 0-360, as airdrop_survey's imu_callback."""
    return (math.degrees(angle_rad) + 360.0) % 360.0


def comment_payload(pitch_deg, roll_deg, yaw_deg):
    return USER_COMMENT_PREFIX + json.dumps({"Pitch": round(pitch_deg, 2), "Roll": round(roll_deg, 2),
                                             "Yaw": round(yaw_deg, 2)}).encode("utf-8")


def write_geotag(path, width, height, lat, lon, alt, yaw_deg, pitch_deg, roll_deg):
    """Insert the GPS / attitude EXIF into an existing JPEG. Returns "piexif" or "pillow"."""
    try:
        import piexif
    except ImportError:
        piexif = None
    if piexif is not None:
        exif_dict = {
            "0th": {piexif.ImageIFD.ImageLength: int(height), piexif.ImageIFD.ImageWidth: int(width)},
            "Exif": {piexif.ExifIFD.UserComment: comment_payload(pitch_deg, roll_deg, yaw_deg)},
            "GPS": {
                piexif.GPSIFD.GPSLatitudeRef: "N" if lat >= 0 else "S",
                piexif.GPSIFD.GPSLatitude: decimal_to_dms(lat),
                piexif.GPSIFD.GPSLongitudeRef: "E" if lon >= 0 else "W",
                piexif.GPSIFD.GPSLongitude: decimal_to_dms(lon),
                piexif.GPSIFD.GPSAltitudeRef: 0 if alt >= 0 else 1,
                piexif.GPSIFD.GPSAltitude: (int(abs(alt * 100)), 100),
                piexif.GPSIFD.GPSImgDirectionRef: "T",
                piexif.GPSIFD.GPSImgDirection: (int(round(yaw_deg * 100)), 100),
            },
        }
        piexif.insert(piexif.dump(exif_dict), path)
        return "piexif"

    from PIL import Image
    from PIL.TiffImagePlugin import IFDRational

    def rationals(dms):
        return tuple(IFDRational(n, d) for n, d in dms)

    with Image.open(path) as img:
        img.load()
        exif = img.getexif()
        exif[0x0101] = int(height)                       # ImageLength
        exif[0x0100] = int(width)                        # ImageWidth
        exif.get_ifd(0x8769)[0x9286] = comment_payload(pitch_deg, roll_deg, yaw_deg)  # Exif: UserComment
        gps = exif.get_ifd(0x8825)
        gps[1] = "N" if lat >= 0 else "S"
        gps[2] = rationals(decimal_to_dms(lat))
        gps[3] = "E" if lon >= 0 else "W"
        gps[4] = rationals(decimal_to_dms(lon))
        gps[5] = b"\x00" if alt >= 0 else b"\x01"
        gps[6] = IFDRational(int(abs(alt * 100)), 100)
        gps[16] = "T"
        gps[17] = IFDRational(int(round(yaw_deg * 100)), 100)
        img.save(path, format="JPEG", quality="keep", exif=exif)
    return "pillow"


def read_geotag(path):
    """(lat, lon, alt, yaw_deg, comment dict) from a geotagged JPEG (Pillow), for checks and tools."""
    from PIL import Image
    with Image.open(path) as img:
        exif = img.getexif()
        gps = exif.get_ifd(0x8825)
        lat = dms_to_decimal([(float(v), 1) for v in gps[2]]) * (-1 if gps[1] == "S" else 1)
        lon = dms_to_decimal([(float(v), 1) for v in gps[4]]) * (-1 if gps[3] == "W" else 1)
        ref = gps.get(5, b"\x00")
        alt = float(gps[6]) * (-1 if (ref[0] if isinstance(ref, (bytes, bytearray)) else ref) == 1 else 1)
        raw = exif.get_ifd(0x8769).get(0x9286, b"")
        comment = json.loads(raw[len(USER_COMMENT_PREFIX):].decode("utf-8")) if raw else {}
        return lat, lon, alt, float(gps.get(17, 0.0)), comment


class SpacingTrigger:
    """True every `spacing` m of horizontal travel (the first call after reset() is a capture too)."""

    def __init__(self, spacing):
        self.spacing = float(spacing)
        self.last = None

    def reset(self):
        self.last = None

    def __call__(self, north, east):
        if self.last is None or math.hypot(north - self.last[0], east - self.last[1]) >= self.spacing:
            self.last = (north, east)
            return True
        return False


def drop_sequence(send: Callable[[int, int], bool], sleep: Callable[[float], bool], channel: int, open_pwm: int,
                  close_pwm: int, interval: float, log: Callable[[str], None] = print) -> Optional[str]:
    """The two PWM commands: open_pwm, wait `interval` s, close_pwm. `send(channel, pwm)` -> accepted,
    `sleep(s)` -> False if cancelled. Returns None on success, else the error. The close command is sent
    even when the wait is cancelled, so the mechanism is never left open."""
    log(f"[PAYLOAD] SERVO{channel} -> {open_pwm} us (release)")
    if not send(channel, open_pwm):
        return f"SERVO{channel} {open_pwm} us not accepted by the autopilot"
    cancelled = not sleep(interval)
    log(f"[PAYLOAD] SERVO{channel} -> {close_pwm} us (close) after {interval:.1f} s")
    if not send(channel, close_pwm):
        return f"SERVO{channel} {close_pwm} us not accepted by the autopilot"
    return "canceled" if cancelled else None
