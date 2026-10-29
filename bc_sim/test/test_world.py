"""Consistency checks of the Gazebo world, its models and the sim configuration (no Gazebo needed)."""
import math
import os
import xml.etree.ElementTree as ET

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(PKG, "worlds", "aerothon_mission2_ardupilot.sdf")
M_PER_DEG = 111318.84502145034
ORIGIN = (13.0827, 80.2707)
EXTERNAL = {"iris_with_ardupilot"}  # from ardupilot_gazebo/models


def includes():
    root = ET.parse(WORLD).getroot()
    out = {}
    for inc in root.iter("include"):
        uri = inc.findtext("uri").replace("model://", "")
        pose = [float(v) for v in (inc.findtext("pose") or "0 0 0 0 0 0").split()]
        out[inc.findtext("name") or uri] = (uri, pose)
    return root, out


def test_every_model_uri_resolves():
    root, inc = includes()
    for name, (uri, _) in inc.items():
        assert os.path.isfile(os.path.join(PKG, "models", uri, "model.sdf")), f"{name}: model://{uri} missing"
        assert os.path.isfile(os.path.join(PKG, "models", uri, "model.config")), uri
    for path in (os.path.join(PKG, "models", d, "model.sdf") for d in os.listdir(os.path.join(PKG, "models"))):
        m = ET.parse(path).getroot()
        for sub in m.iter("uri"):
            u = sub.text.replace("model://", "")
            assert u in EXTERNAL or os.path.isdir(os.path.join(PKG, "models", u)), f"{path}: {sub.text}"
        for tex in m.iter("albedo_map"):
            assert os.path.isfile(os.path.join(os.path.dirname(path), tex.text)), f"{path}: {tex.text}"


def test_overhead_banners_removed_ground_banners_present():
    _, inc = includes()
    uris = {u for u, _ in inc.values()}
    assert "banner_forward" not in uris and "banner_return" not in uris
    gx, gy, gz, _, _, gyaw = inc["green_banner_entry"][1]
    bx, by, bz, _, _, byaw = inc["blue_banner_exit"][1]
    assert gz < 0.01 and bz < 0.01                          # on the ground
    assert abs(gx - 2.8) < 1e-6 and abs(gy - 21.75) < 1e-6  # forward lane Y 20-23.5, before X 3
    assert abs(bx - 13.2) < 1e-6 and abs(by - 18.25) < 1e-6  # return lane Y 16.5-20, past X 13
    assert abs(abs(gyaw) - math.pi / 2) < 1e-3 and abs(abs(byaw) - math.pi / 2) < 1e-3
    assert abs(math.remainder(gyaw - byaw, 2 * math.pi)) > 3.1    # turned 180 deg from each other
    # board footprint (3.5052 x 0.3048, long side across the lane) inside the lane, clear of the start QR
    assert 2.8 + 0.1524 < 3.0 and 2.8 - 0.1524 > 2.5
    assert abs(3.5052 / 2 - 1.75) < 0.01


def test_banner_colours():
    import cv2
    for name, bgr in (("green_banner_ground", (0x40, 0xB1, 0x00)), ("blue_banner_ground", (0xFF, 0x00, 0x00))):
        img = cv2.imread(os.path.join(PKG, "models", name, "materials", "textures", f"{name}.png"))
        h, w = img.shape[:2]
        assert abs(w / h - 3.5052 / 0.3048) < 0.1
        assert tuple(int(v) for v in img[h // 2, int(0.05 * w)]) == bgr   # colour field
        assert tuple(int(v) for v in img[2, w // 2]) == (255, 255, 255)   # white border


def test_vehicle_and_camera():
    _, inc = includes()
    uri, pose = inc["iris"]
    assert uri == "iris_down_cam"
    assert pose[:2] == [0.0, 20.0] and pose[5] == 0.0       # on the pad, facing +X (the corridor)
    m = ET.parse(os.path.join(PKG, "models", "iris_down_cam", "model.sdf")).getroot()
    cam = next(m.iter("camera"))
    assert abs(float(cam.findtext("horizontal_fov")) - 1.22) < 1e-6
    mission = yaml.safe_load(open(os.path.join(PKG, "..", "bc_bringup", "config", "mission.yaml")))
    assert abs(mission["/**"]["ros__parameters"]["hfov_rad"] - 1.22) < 1e-6
    link_pose = [float(v) for v in next(m.iter("link")).findtext("pose").split()]
    assert abs(link_pose[4] - math.pi / 2) < 1e-3          # pitched down: image top = body forward


def test_physics_and_plugins_for_ardupilot():
    root, _ = includes()
    world = root.find("world")
    assert float(world.find("physics").findtext("max_step_size")) == 0.001
    plugins = {p.get("filename") for p in world.findall("plugin")}
    for need in ("gz-sim-physics-system", "gz-sim-sensors-system", "gz-sim-imu-system",
                 "gz-sim-scene-broadcaster-system", "gz-sim-user-commands-system"):
        assert need in plugins
    sc = world.find("spherical_coordinates")
    assert (float(sc.findtext("latitude_deg")), float(sc.findtext("longitude_deg"))) == ORIGIN


def test_geofence_is_the_room():
    pts = []
    for line in open(os.path.join(PKG, "config", "delivery_zone.gazebo.txt")):
        line = line.strip()
        if line and not line.startswith("#"):
            lat, lon = (float(v) for v in line.split(","))
            north = (lat - ORIGIN[0]) * M_PER_DEG
            east = (lon - ORIGIN[1]) * M_PER_DEG * math.cos(math.radians(ORIGIN[0]))
            pts.append((east, north))                       # Gazebo X = east, Y = north
    for x, y in pts:
        assert min(abs(x - 13), abs(x - 53)) < 0.01 and min(abs(y - 0), abs(y - 40)) < 0.01, (x, y)
    assert len(pts) == 4


def test_sitl_home_is_the_world_origin():
    text = open(os.path.join(PKG, "launch", "sitl.launch.py")).read()
    assert f'HOME = "{ORIGIN[0]},{ORIGIN[1]},10,0"' in text
