"""SearchMission against a simulated drone + camera (no ROS, simulated time).

The drone chases the streamed carrot point with limited speed; every 1/15 s the "camera" reports the red
square's outline and the QR tiles in view (exact pinhole projection), stamped with the frame time.
Checks: the matched QR is found, the drone never enters the red, a decoy never matches, no-match
falls back to the best candidate, rotated geofences work, SafeGoto detours round the red.
"""
import math

from target_search import geometry as geo
from target_search import search_logic as sl

W, H, HFOV = 1280, 720, 1.22
REF = (13.0827, 80.2707)


def ned_to_pixel(n, e, drone):
    north, east, alt, heading = drone
    mpp = 2.0 * alt * math.tan(HFOV / 2.0) / W
    dn, de = n - north, e - east
    fwd = dn * math.cos(heading) + de * math.sin(heading)
    right = -dn * math.sin(heading) + de * math.cos(heading)
    return W / 2.0 + right / mpp, H / 2.0 - fwd / mpp


def clip(poly, w, h):
    """Sutherland-Hodgman clip of a pixel polygon to the image."""
    def edge(pts, inside, cut):
        out = []
        for i, p in enumerate(pts):
            q = pts[i - 1]
            if inside(p):
                if not inside(q):
                    out.append(cut(q, p))
                out.append(p)
            elif inside(q):
                out.append(cut(q, p))
        return out

    def cx(x0):
        return lambda a, b: (x0, a[1] + (b[1] - a[1]) * (x0 - a[0]) / (b[0] - a[0]))

    def cy(y0):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (y0 - a[1]) / (b[1] - a[1]), y0)

    for inside, cut in ((lambda p: p[0] >= 0, cx(0)), (lambda p: p[0] <= w, cx(w)),
                        (lambda p: p[1] >= 0, cy(0)), (lambda p: p[1] <= h, cy(h))):
        poly = edge(poly, inside, cut) if poly else poly
    return poly


class World:
    def __init__(self, red_squares, qrs, zone_corners):
        self.red = red_squares            # [(n, e, half)]
        self.qrs = qrs                    # [(n, e, text)]
        self.zone = zone_corners          # NED corners

    def geofence(self):
        return [geo.ned_to_latlon(n, e, *REF) for n, e in self.zone]

    def red_dist(self, n, e):
        """Distance to the nearest red square (0 inside)."""
        return min((math.hypot(max(abs(n - rn) - h, 0.0), max(abs(e - re) - h, 0.0)) for rn, re, h in self.red),
                   default=1e9)


class SimPlatform:
    def __init__(self, world, start, heading=0.0):
        self.world = world
        self.t = 0.0
        self.pos = list(start)            # n, e, d (ground at 0)
        self.heading = heading
        self.sp = list(start)
        self.sp_yaw = heading
        self.mission = None
        self.next_frame = 0.0
        self.min_red = 1e9
        self.path = []
        self.logs = []
        self.gotos = 0

    def now(self):
        return self.t

    def position(self):
        return (self.pos[0], self.pos[1], self.pos[2], self.heading, -self.pos[2])

    def stream(self, n, e, d, yaw):
        self.sp = [n, e, d]
        if math.isfinite(yaw):
            self.sp_yaw = yaw

    def stop_stream(self):
        self.sp = list(self.pos)

    def goto(self, n, e, d, yaw, speed):
        self.gotos += 1
        target = [n, e, d]
        for _ in range(4000):
            self.sp = [p + max(-0.05 * speed, min(0.05 * speed, t - p)) for p, t in zip(self.pos, target)]
            self.sleep(0.05)
            if math.dist(self.pos, target) < 0.05:
                return True, "ok"
        return False, "timeout"

    def cancel_requested(self):
        return False

    def feedback(self, **fields):
        pass

    def log(self, level, text):
        self.logs.append((level, text))

    def sleep(self, dt):
        steps = max(1, int(round(dt / 0.01)))
        for _ in range(steps):
            self._step(dt / steps)

    def _step(self, dt):
        vmax, vz = 2.5, 1.0
        d = [s - p for s, p in zip(self.sp, self.pos)]
        h = math.hypot(d[0], d[1])
        k = min(1.0, vmax * dt / h) if h > 1e-9 else 0.0
        self.pos[0] += d[0] * k
        self.pos[1] += d[1] * k
        self.pos[2] += max(-vz * dt, min(vz * dt, d[2]))
        self.heading = self.sp_yaw
        self.t += dt
        self.min_red = min(self.min_red, self.world.red_dist(self.pos[0], self.pos[1]))
        self.path.append((self.pos[0], self.pos[1]))
        m = self.mission
        if m is None:
            return
        m.on_pose(self.t, self.pos[0], self.pos[1], -self.pos[2], self.heading)
        if self.t >= self.next_frame:
            self.next_frame = self.t + 1.0 / 15.0
            drone = (self.pos[0], self.pos[1], -self.pos[2], self.heading)
            outlines = []
            for rn, re, hh in self.world.red:
                poly = [ned_to_pixel(rn + a * hh, re + b * hh, drone) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                poly = clip(poly, W, H)
                if len(poly) >= 3:
                    outlines.append(poly)
            m.on_red(self.t, W, H, outlines)
            for qn, qe, text in self.world.qrs:
                u, v = ned_to_pixel(qn, qe, drone)
                if 30 < u < W - 30 and 30 < v < H - 30:
                    matched = text == "REF"
                    m.on_qr(self.t, W, H, u, v, matched, 1.0 if matched else 0.6, text)


def run_search(world, start, req_kw=None, heading=0.0):
    p = SimPlatform(world, start, heading)
    mission = sl.SearchMission(p, sl.Config(hfov=HFOV))
    p.mission = mission
    kw = dict(altitude=5.0, yaw=math.nan, speed=1.5)
    kw.update(req_kw or {})
    out = mission.run(sl.Request(**kw), world.geofence(), REF)
    return out, p, mission


ZONE = [(13.0, -21.0), (35.0, -21.0), (35.0, -1.0), (13.0, -1.0)]


def test_finds_the_twin_round_the_red():
    world = World([(29.0, -15.0, 2.0)], [(24.0, -11.3, "REF"), (20.0, -15.4, "D3"), (30.0, -7.2, "D5")], ZONE)
    out, p, _ = run_search(world, (15.1, -19.0, -5.0))
    assert out.status == "success" and out.matched, out
    assert math.hypot(out.target[0] - 24.0, out.target[1] + 11.3) < 0.5, out.target
    assert math.hypot(p.pos[0] - 24.0, p.pos[1] + 11.3) < 1.0
    assert p.min_red > 0.5, f"came within {p.min_red:.2f} m of the red"
    assert out.red_zones >= 1


class GridDecoyPlatform(SimPlatform):
    """QR text "GRID:<text>": reported like an undecoded code that passes the grid threshold (matched,
    score 0.94, no text) except every 4th frame, decoded as <text> (not matched)."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.seen = {}

    def attach(self, mission):
        real = mission.on_qr

        def on_qr(t, w, h, u, v, matched, score, text):
            if text.startswith("GRID:"):
                n = self.seen[text] = self.seen.get(text, 0) + 1
                if n % 4 == 0:
                    return real(t, w, h, u, v, False, 0.94, text[5:])
                return real(t, w, h, u, v, True, 0.94, "")
            return real(t, w, h, u, v, matched, score, text)
        mission.on_qr = on_qr
        self.mission = mission


def test_undecoded_grid_match_of_a_decoy_is_not_accepted():
    # the decoy is seen first (next to the start); it is a grid "match" on 3 of 4 frames, decoded as D9 on
    # the 4th: the sweep must go on to the real twin
    world = World([], [(16.0, -17.0, "GRID:D9"), (24.0, -11.3, "REF")], ZONE)
    p = GridDecoyPlatform(world, (15.1, -19.0, -5.0))
    mission = sl.SearchMission(p, sl.Config(hfov=HFOV))
    p.attach(mission)
    out = mission.run(sl.Request(altitude=5.0, yaw=math.nan, speed=1.5), world.geofence(), REF)
    assert out.status == "success" and out.matched, out
    assert math.hypot(out.target[0] - 24.0, out.target[1] + 11.3) < 0.5, out.target
    assert any("not the reference" in text for _, text in p.logs)


def test_skip_fly_reports_the_twin_without_flying_there():
    world = World([(29.0, -15.0, 2.0)], [(24.0, -11.3, "REF"), (20.0, -15.4, "D3")], ZONE)
    out, p, _ = run_search(world, (15.1, -19.0, -5.0), {"skip_fly": True})
    assert out.status == "success" and out.matched, out
    assert math.hypot(out.target[0] - 24.0, out.target[1] + 11.3) < 0.5, out.target
    assert math.hypot(p.pos[0] - 24.0, p.pos[1] + 11.3) > 1.0       # target_navigator does the last leg
    assert any("handing over to target_navigator" in text for _, text in p.logs)


def test_no_match_goes_to_best_candidate():
    world = World([(29.0, -15.0, 2.0)], [(20.0, -15.4, "D3"), (30.0, -7.2, "D5")], ZONE)
    out, p, _ = run_search(world, (15.1, -19.0, -5.0))
    assert out.status == "success" and not out.matched
    assert min(math.hypot(out.target[0] - n, out.target[1] - e) for n, e in ((20.0, -15.4), (30.0, -7.2))) < 0.5
    assert p.min_red > 0.5


def test_twin_inside_red_is_ignored():
    world = World([(29.0, -15.0, 2.0)], [(29.0, -15.0, "REF"), (20.0, -15.4, "D3")], ZONE)
    out, p, _ = run_search(world, (15.1, -19.0, -5.0))
    assert out.status == "success" and not out.matched     # the twin in the red is never flown to
    assert math.hypot(out.target[0] - 20.0, out.target[1] + 15.4) < 0.5
    assert p.min_red > 0.5


def test_rotated_geofence():
    """Zone at a 35 deg bearing: rows run along its long side, red avoided, twin found."""
    c, s = math.cos(math.radians(35)), math.sin(math.radians(35))
    centre = (24.0, -11.0)

    def at(a, b):   # a along the long side, b across
        return (centre[0] + a * c - b * s, centre[1] + a * s + b * c)
    zone = [at(-11, -9), at(11, -9), at(11, 9), at(-11, 9)]
    red = at(5.0, -4.0)
    twin = at(2.0, 4.0)
    world = World([(red[0], red[1], 1.5)], [(twin[0], twin[1], "REF")], zone)
    out, p, _ = run_search(world, (at(-10, -8)[0], at(-10, -8)[1], -5.0), heading=math.radians(35))
    assert out.status == "success" and out.matched, out
    assert math.hypot(out.target[0] - twin[0], out.target[1] - twin[1]) < 0.5
    assert p.min_red > 0.5


def test_safe_goto_detours_round_mapped_red():
    world = World([(24.0, -11.0, 2.0)], [], ZONE)
    p = SimPlatform(world, (15.0, -11.0, -5.0))
    mission = sl.SearchMission(p, sl.Config(hfov=HFOV))
    p.mission = mission
    mission.setup_zone(world.geofence(), REF, 1.0)
    mission.margin = 2.0
    # red mapped earlier in the search: N 22..26, E -13..-9 (zone frame: x along north, y = west)
    x0, y0 = mission.zone.frame.to_zone(22.0, -9.0)
    x1, y1 = mission.zone.frame.to_zone(26.0, -13.0)
    mission.zone.red.add_box((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
    assert mission.red_zones() == 1
    p.min_red = 1e9
    ok, msg, detoured = mission.safe_goto(32.0, -11.0, -5.0, 0.0, 1.5)
    assert ok and detoured, msg
    assert math.hypot(p.pos[0] - 32.0, p.pos[1] + 11.0) < 0.1
    assert p.min_red > 0.5, p.min_red
    # a clear line is flown straight with one GoTo
    gotos = p.gotos
    ok, msg, detoured = mission.safe_goto(32.0, -3.0, -5.0, 0.0, 1.5)
    assert ok and not detoured and p.gotos == gotos + 1


def test_corridor_heading_mode_still_avoids_red():
    """Holding the corridor heading (camera sees only 1.95 m ahead) still keeps out of the red itself."""
    world = World([(29.0, -15.0, 2.0)], [(24.0, -11.3, "REF")], ZONE)
    out, p, _ = run_search(world, (15.1, -19.0, -5.0), req_kw=dict(yaw=0.0))
    assert out.status == "success" and out.matched
    assert p.min_red > 0.5


def test_wide_ahead_sees_red_before_the_margin():
    world = World([(29.0, -15.0, 2.0)], [(24.0, -11.3, "REF")], ZONE)
    out, p, _ = run_search(world, (15.1, -19.0, -5.0))
    assert out.status == "success" and out.matched
    assert not any("escape" in text for _, text in p.logs), "had to escape from inside the margin"
    assert any("sees 3.5 m ahead" in text for _, text in p.logs)
    assert p.min_red > 1.5, p.min_red


def test_geofence_parsing_and_latlon():
    text = "# zone\n13.0,80.0\n13.001;80.0\n\n13.001 80.001\n13.0,80.001\n"
    pts = geo.parse_geofence(text)
    assert len(pts) == 4
    n, e = geo.latlon_to_ned(*geo.ned_to_latlon(12.3, -4.5, *REF), *REF)
    assert abs(n - 12.3) < 1e-6 and abs(e + 4.5) < 1e-6
    for bad in ("13.0,80.0\n13.1,80.0\n", "13.0\n1,2\n3,4\n", "91,0\n1,1\n2,2\n"):
        try:
            geo.parse_geofence(bad)
            raise AssertionError(f"accepted {bad!r}")
        except ValueError:
            pass


# ---------------------------------------------------------------------- search_mode "auto"

class AutoSimPlatform(SimPlatform):
    """SimPlatform plus flight_controller's ~/survey: the AUTO mission walks its waypoints at `speed`;
    stopping it switches to GUIDED after GUIDED_DELAY s of AUTO flight (mode change), then holds."""
    GUIDED_DELAY = 0.4

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.auto = None            # [points, index, speed, state]
        self.surveys = []           # waypoint lists pushed
        self.streams_in_auto = 0

    def survey_start(self, points, down, yaw, speed):
        self.surveys.append([tuple(p) for p in points])
        self.auto = [[(n, e, down) for n, e in points], 0, speed, "running"]
        if math.isfinite(yaw):
            self.sp_yaw = yaw
        return True, "ok"

    def survey_poll(self):
        if self.auto is None:
            return "failed", 0, "no survey"
        pts, i, _, state = self.auto
        return state, (len(pts) if state == "done" else i), ""

    def survey_stop(self):
        if self.auto is None:
            return 0
        if self.auto[3] == "running":
            self.sleep(self.GUIDED_DELAY)           # still flying in AUTO until GUIDED is engaged
        i = self.auto[1]
        self.auto = None
        self.sp = list(self.pos)                    # GUIDED: hold where it is
        return i

    def stream(self, n, e, d, yaw):
        if self.auto is not None and self.auto[3] == "running":
            self.streams_in_auto += 1
        super().stream(n, e, d, yaw)

    def _step(self, dt):
        if self.auto is not None and self.auto[3] == "running":
            pts, i, speed, _ = self.auto
            target = pts[i]
            if math.dist(self.pos, target) < 0.15:
                if i + 1 < len(pts):
                    self.auto[1] = i + 1
                    target = pts[i + 1]
                else:
                    self.auto[3] = "done"
            d = [t - p for t, p in zip(target, self.pos)]
            dist = math.sqrt(sum(x * x for x in d))
            k = min(1.0, speed * dt / dist) if dist > 1e-9 else 0.0
            self.sp = [p + x * k for p, x in zip(self.pos, d)]
        super()._step(dt)


def run_auto(world, start, req_kw=None, heading=0.0):
    p = AutoSimPlatform(world, start, heading)
    mission = sl.SearchMission(p, sl.Config(hfov=HFOV, search_mode="auto"))
    p.mission = mission
    kw = dict(altitude=5.0, yaw=math.nan, speed=1.5)
    kw.update(req_kw or {})
    out = mission.run(sl.Request(**kw), world.geofence(), REF)
    return out, p, mission


def test_auto_sweep_stops_for_red_detours_and_resumes_in_auto():
    world = World([(29.0, -15.0, 2.0)], [(24.0, -11.3, "REF"), (20.0, -15.4, "D3"), (30.0, -7.2, "D5")], ZONE)
    out, p, _ = run_auto(world, (15.1, -19.0, -5.0))
    assert out.status == "success" and out.matched, out
    assert math.hypot(out.target[0] - 24.0, out.target[1] + 11.3) < 0.5, out.target
    assert p.min_red > 0.5, f"came within {p.min_red:.2f} m of the red"
    assert len(p.surveys) >= 2, "the rest of the rows was not pushed again after the red"
    assert any("red ahead on the AUTO stretch" in t or "red on the row" in t for _, t in p.logs), p.logs
    assert p.streams_in_auto == 0, "setpoints streamed while in AUTO"


def test_auto_sweep_without_red_is_one_mission():
    world = World([], [(20.0, -15.4, "D3")], ZONE)             # no twin: the whole pattern, then best candidate
    out, p, _ = run_auto(world, (15.1, -19.0, -5.0))
    assert out.status == "success" and not out.matched, out
    assert len(p.surveys) == 1
    assert not any("detour" in t for _, t in p.logs)


def test_auto_cuts_rows_at_mapped_red():
    world = World([], [], ZONE)
    p = AutoSimPlatform(world, (15.0, -11.0, -5.0))
    mission = sl.SearchMission(p, sl.Config(hfov=HFOV, search_mode="auto"))
    mission.setup_zone(world.geofence(), REF, 1.0)
    mission.margin = 2.0
    mission.zone.red.add_box((-1.0, -1.0, 1.0, 1.0))           # red at the zone centre, 2 x 2 m
    with mission.lock:
        run, gap, rest = mission._split_at_red((-9.0, 0.0), [(9.0, 0.0), (9.0, 3.0), (-9.0, 3.0)])
    # the row through the centre stops ~3 m short of it (1 m half size + 2 m margin), resumes ~3.5 m past it
    assert len(run) == 1 and -3.6 < run[0][0] < -2.6 and abs(run[0][1]) < 1e-6, run
    assert gap is not None and 3.1 < gap[0] < 4.1 and abs(gap[1]) < 1e-6, gap   # + GAP_PAST 0.5 m
    assert rest == [(9.0, 0.0), (9.0, 3.0), (-9.0, 3.0)][0:1] + [(9.0, 3.0), (-9.0, 3.0)]
    with mission.lock:
        run2, gap2, _ = mission._split_at_red((-9.0, 6.0), [(9.0, 6.0)])   # 6 m off: clear of red + margin
    assert run2 == [(9.0, 6.0)] and gap2 is None
