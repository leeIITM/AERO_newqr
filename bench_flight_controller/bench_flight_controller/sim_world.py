"""Synthetic field and downward-camera renderer for bench tests (ROS-free).

Field (Mission 2 spec), in a field frame with x forward along the take-off heading and y to the right,
rotated by `field_yaw` about the take-off point:
    reference QR       2 m ahead of the pad (surge 1)
    green corridor     3.5 m (11.5 ft) wide, `corridor_length` long, `green_sway` m left of the take-off
                       line; arch over its entrance at `arch_height`, walls `wall_height` high
    green boards       arch board 11.5 x 1.5 ft on the entrance arch, ground board 11.5 x 1 ft on the
                       ground just in front of the arch, both #00B140 with a 1-inch white border
    blue corridor      next to the green one, `corridor_gap` m to its right, same length
    blue boards        the same pair (#0000FF) at the far end, facing back: upright text after the 180 deg yaw
    QR field           twin of the reference QR + decoys in the lawnmower area
Raised objects (walls, arch boards) are drawn with perspective: an object h m up is seen from alt - h.
Camera model is the pinhole model of bc_common.geometry.pixel_to_ned_offset (gimbal straight down,
image top = body forward), so a perfect detector reproduces the true ground positions.
"""
import math
from dataclasses import dataclass, fields
from typing import List, Optional

import cv2
import numpy as np

GREEN_BGR = (64, 177, 0)
BLUE_BGR = (255, 0, 0)
WHITE_BGR = (245, 245, 245)
FT = 0.3048
INCH = 0.0254


@dataclass
class FieldParams:
    green_sway: float = 7.5          # m left of the take-off line to the green ground board's centre
    green_forward: float = 4.3       # m ahead of the pad to the green ground board's centre
    corridor_length: float = 10.0    # m
    corridor_width: float = 11.5 * FT
    corridor_gap: float = 1.0        # m between the green corridor's right wall and the blue one's left wall
    wall_height: float = 3.8         # m: under the drone at 5 m, above it at 3 m
    arch_height: float = 4.0         # m: the drone passes under it at 3 m, not at 5 m
    board_offset: float = 0.4        # m from a ground board's centre to its arch (corridor end)
    field_yaw_deg: float = 0.0       # corridors rotated this much (clockwise) from the take-off heading

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


@dataclass
class WorldObject:
    name: str
    sprite: np.ndarray       # BGR
    north: float
    east: float
    length: float            # m along `heading` (sprite x)
    width: float             # m across (sprite y)
    heading: float           # rad, direction of sprite +x
    alpha: Optional[np.ndarray] = None
    height: float = 0.0      # m above the ground


def banner_sprite(color_bgr, width_m, length_m=11.5 * FT, border_m=INCH, text="AeroTHON 2026", px_per_m=200):
    """Foam board: colour with a white border all round and white text."""
    w, h = int(length_m * px_per_m), int(width_m * px_per_m)
    b = max(1, int(round(border_m * px_per_m)))
    img = np.full((h, w, 3), WHITE_BGR, dtype=np.uint8)
    img[b:h - b, b:w - b] = color_bgr
    scale = (h - 2 * b) / 60.0
    thick = max(1, int(scale * 2))
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    cv2.putText(img, text, ((w - tw) // 2, (h + th) // 2), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255),
                thick, cv2.LINE_AA)
    return img


def qr_sprite(text, module_px=12):
    code = cv2.QRCodeEncoder.create().encode(text)  # includes the quiet zone, white = 255
    img = cv2.resize(code, (code.shape[1] * module_px, code.shape[0] * module_px), interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), code.shape[0]


class SimWorld:
    def __init__(self, objects: List[WorldObject] = None, seed=1, field: FieldParams = None):
        self.objects = objects if objects is not None else default_objects(field=field)
        self.objects = sorted(self.objects, key=lambda o: o.height)  # higher objects drawn on top
        rng = np.random.default_rng(seed)
        tex = rng.normal(0, 9, (256, 256, 3)) + np.array([70, 95, 105])  # dry brown/grey ground, low saturation
        self.ground = np.clip(cv2.GaussianBlur(tex, (0, 0), 2.0), 0, 255).astype(np.uint8)

    def render(self, north, east, heading, alt, width=1280, height=720, hfov=1.22, noise=0.0):
        alt = max(0.3, alt)
        k = 2.0 * math.tan(hfov / 2.0) / width
        frame = self._ground(north, east, heading, alt * k, width, height)
        for obj in self.objects:
            dist = alt - obj.height
            if dist < 0.3:
                continue  # level with or above the camera
            mpp = dist * k
            reach = 0.5 * math.hypot(width, height) * mpp
            if math.hypot(obj.north - north, obj.east - east) > reach + 0.5 * max(obj.length, obj.width):
                continue
            self._draw(frame, obj, north, east, heading, mpp)
        if noise > 0:
            frame = np.clip(frame.astype(np.float32) + np.random.normal(0, noise, frame.shape), 0, 255).astype(np.uint8)
        return frame

    @staticmethod
    def world_to_image_affine(north, east, heading, mpp, width, height):
        """2x3 affine: (n, e) -> (u, v)."""
        c, s = math.cos(heading), math.sin(heading)
        # fwd = dn c + de s ; right = -dn s + de c ; u = w/2 + right/mpp ; v = h/2 - fwd/mpp
        a = np.array([[-s / mpp, c / mpp], [-c / mpp, -s / mpp]])
        b = np.array([width / 2.0, height / 2.0]) - a @ np.array([north, east])
        return np.hstack([a, b.reshape(2, 1)])

    def _sprite_affine(self, obj, world_aff):
        h_px, w_px = obj.sprite.shape[:2]
        dx = np.array([math.cos(obj.heading), math.sin(obj.heading)]) * (obj.length / w_px)
        dy = np.array([math.cos(obj.heading + math.pi / 2), math.sin(obj.heading + math.pi / 2)]) * (obj.width / h_px)
        origin = np.array([obj.north, obj.east]) - dx * (w_px / 2.0) - dy * (h_px / 2.0)
        sprite_to_world = np.array([[dx[0], dy[0], origin[0]], [dx[1], dy[1], origin[1]], [0, 0, 1]])
        return world_aff @ sprite_to_world

    def _draw(self, frame, obj, north, east, heading, mpp):
        h, w = frame.shape[:2]
        aff = self._sprite_affine(obj, self.world_to_image_affine(north, east, heading, mpp, w, h))
        warped = cv2.warpAffine(obj.sprite, aff, (w, h), flags=cv2.INTER_AREA, borderMode=cv2.BORDER_CONSTANT)
        alpha = obj.alpha if obj.alpha is not None else np.full(obj.sprite.shape[:2], 255, np.uint8)
        mask = cv2.warpAffine(alpha, aff, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        m = (mask.astype(np.float32) / 255.0)[..., None]
        frame[:] = (frame * (1 - m) + warped * m).astype(np.uint8)

    def _ground(self, north, east, heading, mpp, width, height):
        # ground texture tiled every 4 m in the world
        tile_m = 4.0
        aff = self.world_to_image_affine(north, east, heading, mpp, width, height)
        px = tile_m / self.ground.shape[1]
        # sprite pixel (sx, sy) -> world (n = -sy*px, e = sx*px), then to image
        tex_to_world = np.array([[0, -px, 0], [px, 0, 0], [0, 0, 1]])
        full = np.vstack([aff, [0, 0, 1]]) @ tex_to_world
        inv = cv2.invertAffineTransform(full[:2])
        return cv2.warpAffine(self.ground, inv, (width, height), flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_WRAP)


def field_layout(f: FieldParams):
    """Key positions in local NED (take-off at the origin heading north), for scoring runs."""
    yaw = math.radians(f.field_yaw_deg)

    def to_ned(x, y):
        return x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)

    entrance = f.green_forward + f.board_offset
    blue_y = -f.green_sway + f.corridor_width + f.corridor_gap
    return {
        "yaw": yaw,
        "to_ned": to_ned,
        "entrance_x": entrance,
        "exit_x": entrance + f.corridor_length,
        "green_y": -f.green_sway,
        "blue_y": blue_y,
        "green_board": to_ned(f.green_forward, -f.green_sway),
        "blue_board": to_ned(entrance + f.corridor_length + f.board_offset, blue_y),
    }


def default_objects(reference_text="AEROTHON-7", decoys=("AEROTHON-3", "AEROTHON-5", "AEROTHON-9"),
                    field: FieldParams = None):
    f = field or FieldParams()
    lay = field_layout(f)
    yaw, to_ned = lay["yaw"], lay["to_ned"]
    objs: List[WorldObject] = []

    def add(name, sprite, x, y, length, width, heading, height=0.0):
        n, e = to_ned(x, y)
        objs.append(WorldObject(name, sprite, n, e, length, width, heading + yaw, height=height))

    length_m = 11.5 * FT
    entrance, exit_x = lay["entrance_x"], lay["exit_x"]
    # boards: long axis across the corridor. Green text upright facing forward (sprite +x to the right),
    # blue turned 180 deg: upright facing back after the yaw.
    for color, bgr, y, ground_x, arch_x, heading in (
        ("green", GREEN_BGR, lay["green_y"], f.green_forward, entrance, math.pi / 2),
        ("blue", BLUE_BGR, lay["blue_y"], exit_x + f.board_offset, exit_x, -math.pi / 2),
    ):
        add(f"{color}_banner", banner_sprite(bgr, 1.0 * FT), ground_x, y, length_m, 1.0 * FT, heading)
        add(f"{color}_arch_banner", banner_sprite(bgr, 1.5 * FT), arch_x, y, length_m, 1.5 * FT, heading,
            height=f.arch_height)
    wall = np.full((8, 200, 3), WHITE_BGR, dtype=np.uint8)
    half = f.corridor_width / 2.0
    for y in (lay["green_y"] - half, lay["green_y"] + half, lay["blue_y"] - half, lay["blue_y"] + half):
        add("wall", wall, entrance + f.corridor_length / 2.0, y, f.corridor_length, 0.1, 0.0, height=f.wall_height)

    def qr(text, x, y, rot_deg=0.0, size=0.8):
        img, _ = qr_sprite(text)
        add(f"qr:{text}", img, x, y, size, size, math.radians(rot_deg))

    qr(reference_text, 2.0, 0.0)            # reference, under the drone after surge 1
    qr(reference_text, 24.0, -11.3, 90.0)   # twin in the QR field (rotated: grid match is 4-rotation)
    qr(decoys[0], 20.0, -15.4, 0.0)
    qr(decoys[1], 30.0, -7.2, 180.0)
    qr(decoys[2], 17.5, -19.4, 270.0)
    return objs
