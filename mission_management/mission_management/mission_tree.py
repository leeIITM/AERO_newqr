"""AeroTHON 2026 banner corridor + QR mission (blue_green_qr_fixed.py) as a py_trees tree.

Each step calls an action on flight_controller (Takeoff / GoTo / Land) or on a task node:
    qr_detector         /qr_detector/capture_reference   stage 2   reference QR from the camera
    banner_search       /search_banner                   stage 4 (green), stage 11 (blue): sway left
    banner_align        /align_banner                    stage 5 (green), stage 11 (blue): PBVS + heading
    corridor_navigator  /traverse_corridor               stages 6-7 and stage 11: descend, surge, climb
    target_search       /search_target                   stage 9   geofenced QR search round red zones
                                                                   (qr_search: target_search, the default)
    target_navigator    /navigate_to_target              stage 9b  fly to the found QR's lat/lon (red-aware)
    lawnmower_search    /lawnmower_search                stage 9   serpentine QR search (qr_search: lawnmower)
    payload_drop        /payload_drop                    stage 10b two PWMs after centring (target_search only)
With qr_search target_search and payload_drop (default): search (geotagged frames every N m) -> navigator
to the QR coordinates -> centring + hold -> payload drop -> the mission goes on. Lawnmower: as before.
Red zone (only one, inside the delivery zone): red_zone_detector is switched ON when the target search
starts and OFF after the drop (or whenever stages 9-10 end); red avoidance (search detours, navigator's
/safe_goto, qr_centering clearance) only acts in between. Every leg after that is a plain GoTo. With
qr_search lawnmower red detection stays off.
    qr_centering        /center_on_qr                    stage 10  centring + 15 s hover hold
Green and blue banners go through exactly the same sway / align / corridor steps; the only blue-specific
step is getting there: 11 m from the green banner along the corridor (1 m past its exit), yaw 180 deg.

Heights: every target is `pad z - height` with the pad z recorded before take-off, so transit, banner
search and QR search are at `alt` (5 m) and both corridors at `corridor_alt` (3 m) above the pad.

Fallbacks, as in the script:
    reference QR               -> no timeout: hover, then growing squares round the surge-1 point until seen;
                                  only a failed capture (qr_detector abort) skips stages 9-10
    no matching QR found       -> target_search: centre + hold over the best-scoring QR seen;
                                  lawnmower: stage 10 skipped
    green banner not found     -> fly home at transit altitude and land
    blue banner not found      -> do not descend or surge blind: fly home at transit altitude and land
Any other failure after the green banner search (a GoTo that fails, a node that aborts) also ends in
"climb to transit altitude, fly home, land".

The whole tree runs once.
"""
import math

import py_trees
import py_trees_ros
import rclpy
from mission_interfaces.action import (
    AlignBanner, CenterOnQr, GoTo, Land, LawnmowerSearch, NavigateToTarget, PayloadDrop,
    SearchBanner, SearchTarget, Takeoff, TraverseCorridor,
)
from mission_interfaces.msg import MissionStatus

from bc_common.ros_utils import DETECTION_QOS, GOTO_ACTION, LAND_ACTION, MISSION_STATUS_TOPIC, RED_ACTIVE_SERVICE, \
    TAKEOFF_ACTION, from_point, to_point
from mission_management import mission_math as mm
from mission_management.steps import (
    ActionStep, Check, Compute, ContextSetup, Hold, MissionContext, ReferenceCapture, SetActive, WaitForVehicle,
)

Seq = py_trees.composites.Sequence
Sel = py_trees.composites.Selector
NAN = math.nan


# ---------------------------------------------------------------- step factories


def goto(name, ctx, target_fn, yaw_fn=lambda c: NAN, speed_fn=lambda c: c.cfg.speed, tol=0.0):
    def goal(c):
        n, e, d = target_fn(c)
        return GoTo.Goal(frame=GoTo.Goal.FRAME_LOCAL_NED, x=float(n), y=float(e), z=float(d),
                         yaw=float(yaw_fn(c)), speed=float(speed_fn(c)), acceptance_radius=float(tol))
    return ActionStep(name, ctx, GoTo, GOTO_ACTION, goal)


def when(name, ctx, pred, child):
    """Run `child` only if pred(ctx); otherwise succeed without it."""
    return Sel(name, memory=True, children=[Check(f"{name}: skip?", ctx, lambda c: not pred(c)), child])


def payload_mode(c):
    return c.cfg.qr_search == "target_search" and c.cfg.payload_drop


def climb_to_transit(name, ctx):
    return goto(name, ctx, lambda c: (NAN, NAN, c.z(c.cfg.alt)))


def mark_abort(c, label):
    c.abort_reason = label
    c.log().warn(f"[MISSION ABORT] {label}")


def return_home_and_land(ctx, label):
    """Climb straight up to transit altitude, fly over the walls to the pad, land."""
    return Seq(label, memory=True, children=[
        Compute(f"{label}: log", ctx, lambda c: mark_abort(c, label)),
        climb_to_transit(f"{label}: climb to transit altitude", ctx),
        goto(f"{label}: fly to take-off point", ctx,
             lambda c: (c.state.takeoff[0], c.state.takeoff[1], c.z(c.cfg.alt)),
             yaw_fn=lambda c: c.state.takeoff_yaw),
        ActionStep(f"{label}: land", ctx, Land, LAND_ACTION, lambda c: Land.Goal()),
    ])


def sway_search(name, ctx, color, start_fn, yaw_fn, max_sway_fn, on_result):
    """Stage 4 / 11a: sway LEFT of the held heading at sway_speed, detecting on every frame while moving,
    then hover at max sway to finish the persistence check."""
    def goal(c):
        end = mm.sway_end(start_fn(c), yaw_fn(c), max_sway_fn(c))
        return SearchBanner.Goal(color=color, waypoints=[to_point(end)], yaw=float(yaw_fn(c)),
                                 speed=float(c.cfg.sway_speed), dwell=[float(c.cfg.sway_end_hover)],
                                 detect_while_moving=True)
    return ActionStep(name, ctx, SearchBanner, "/search_banner", goal, on_result)


def align(name, ctx, color, initial_fn, expected_fn, on_result):
    """Stage 5 / 11b: fly onto the banner and PBVS-centre it, measuring the corridor heading."""
    def goal(c):
        cfg = c.cfg
        return AlignBanner.Goal(color=color, initial_target=to_point(initial_fn(c)),
                                expected_heading=float(expected_fn(c)),
                                max_heading_correction=math.radians(cfg.max_heading_correction_deg),
                                max_iters=int(cfg.center_iters), tol_px=float(cfg.center_tol),
                                gain=float(cfg.center_gain), settle_time=float(cfg.banner_settle))
    return ActionStep(name, ctx, AlignBanner, "/align_banner", goal, on_result)


def traverse(name, ctx, entry_fn, heading_fn, on_result=None):
    """Stages 6-7 / 11c: descend to corridor altitude over the banner, surge through, climb at the exit."""
    def goal(c):
        cfg = c.cfg
        return TraverseCorridor.Goal(entry=to_point(entry_fn(c)), heading=float(heading_fn(c)),
                                     altitude_z=float(c.z(cfg.corridor_alt)), length=float(cfg.corridor_surge),
                                     speed=float(cfg.corridor_speed), settle_time=float(cfg.corridor_settle),
                                     exit_altitude_z=float(c.z(cfg.alt)))
    return ActionStep(name, ctx, TraverseCorridor, "/traverse_corridor", goal, on_result)


# ---------------------------------------------------------------- Compute / result functions


def record_ground(c):
    p = c.pose()
    if p.airborne:
        c.log().error("Vehicle is already airborne: refusing to start the mission.")
        return False
    c.state.ground_z = p.z
    c.log().info(f"[REFERENCE FRAME] Pad NED z = {p.z:+.2f} m (every height is measured from here)")


def record_takeoff_frame(c):
    p = c.pose()
    s = c.state
    s.takeoff = [p.x, p.y, c.z(c.cfg.alt)]
    s.takeoff_yaw = p.heading
    s.sp_surge1 = mm.surge_target(s.takeoff, s.takeoff_yaw, c.cfg.surge1, c.z(c.cfg.alt))
    s.sp_surge2 = mm.surge_target(s.takeoff, s.takeoff_yaw, c.cfg.surge1 + c.cfg.surge2, c.z(c.cfg.alt))
    c.log().info(f"[REFERENCE FRAME] Takeoff origin N={p.x:+.2f} E={p.y:+.2f}, "
                 f"heading {math.degrees(p.heading):+.1f} deg, height {s.ground_z - p.z:.2f} m")


def store_reference(c, result, ok):
    c.state.ref_ok = ok
    c.state.ref_text = result.decoded_text if ok else ""
    if not ok:
        c.log().warn(f"[WARN] Reference QR capture failed ({result.message}): the QR search (stage 9) and "
                     "the hover hold (stage 10) will be SKIPPED.")


def banner_estimate(c, result):
    return [result.banner_estimate.x, result.banner_estimate.y, c.z(c.cfg.alt)]


def store_green_search(c, result, ok):
    if ok:
        c.state.green_estimate = banner_estimate(c, result)


def store_green_align(c, result, ok):
    s = c.state
    if not ok:
        return
    s.green_banner = from_point(result.banner_position)
    s.corridor_heading = result.corridor_heading
    s.sway_x = mm.sway_distance(s.green_banner, s.takeoff, s.takeoff_yaw)
    c.log().info(f"[MEASUREMENT] Green banner N={s.green_banner[0]:+.2f} E={s.green_banner[1]:+.2f} | "
                 f"corridor heading {math.degrees(s.corridor_heading):+.1f} deg "
                 f"({'measured' if result.heading_measured else 'expected, not measured'}) | "
                 f"sway x = {s.sway_x:.2f} m | lock: {result.locked}")


def store_corridor_exit(c, result, ok):
    if ok:
        c.state.corridor_exit = from_point(result.exit_position)


def compute_wall_target(c):
    s, cfg = c.state, c.cfg
    s.sp_wall = mm.wall_target(s.corridor_exit, s.corridor_heading, cfg.wall_offset, s.sway_x,
                               cfg.wall_forward, c.z(cfg.alt))
    c.log().info(f"[STAGE 8] lateral shift left by ({cfg.wall_offset:.1f} - x) = {cfg.wall_offset - s.sway_x:.2f} m "
                 f"-> N={s.sp_wall[0]:+.2f} E={s.sp_wall[1]:+.2f} (1 m from the 20 m wall)")


def lawnmower_goal(c):
    s, cfg = c.state, c.cfg
    return LawnmowerSearch.Goal(start=to_point(s.sp_wall), heading=float(s.corridor_heading),
                                length=float(cfg.lawnmower_length), row_step=float(cfg.lawnmower_step),
                                rows=int(cfg.lawnmower_rows), speed=float(cfg.speed),
                                waypoint_tolerance=float(cfg.lawnmower_tolerance))


def store_qr_target(c, result, ok):
    if ok:
        c.state.target_qr = from_point(result.target)
        c.state.qr_matched = True
        c.log().info(f"[STAGE 9] MATCHED QR '{result.decoded_text}' score {result.score:.2f} at "
                     f"N={result.target.x:+.2f} E={result.target.y:+.2f}")


def target_search_goal(c):
    s, cfg = c.state, c.cfg
    return SearchTarget.Goal(
        geofence_file=cfg.geofence_file, altitude=float(cfg.alt),
        yaw=float(s.corridor_heading if cfg.search_yaw_mode == "corridor" else NAN),
        wall_margin=float(cfg.search_wall_margin), overlap=float(cfg.search_overlap), rows=int(cfg.search_rows),
        speed=float(cfg.speed), red_zone_margin=float(cfg.red_zone_margin),
        confirm_matches=int(cfg.confirm_matches), max_revisits=int(cfg.max_revisits),
        revisit_time=float(cfg.revisit_time), skip_fly_to_target=bool(cfg.payload_drop))


def store_search_target(c, result, ok):
    if ok:
        c.state.target_qr = from_point(result.target)
        c.state.qr_matched = bool(result.matched)
        c.state.target_latlon = [result.latitude, result.longitude]
        c.log().info(f"[STAGE 9] {'MATCHED QR' if result.matched else 'BEST CANDIDATE (no match)'} "
                     f"'{result.decoded_text}' score {result.score:.2f} at N={result.target.x:+.2f} "
                     f"E={result.target.y:+.2f} ({result.latitude:.7f}, {result.longitude:.7f}) | "
                     f"red zones mapped: {result.red_zones} | coverage {100 * result.coverage:.0f} %")


def navigate_goal(c):
    lat, lon = c.state.target_latlon
    return NavigateToTarget.Goal(latitude=float(lat), longitude=float(lon), z=float(c.z(c.cfg.alt)),
                                 yaw=float(c.state.corridor_heading), speed=float(c.cfg.speed), tolerance=0.0)


def store_navigation(c, result, ok):
    if ok:
        c.state.target_qr = from_point(result.target)


def store_drop(c, result, ok):
    if ok:
        c.log().info(f"[STAGE 10] PAYLOAD DROPPED ({result.payloads_delivered} delivered)")
    else:
        c.log().warn(f"[STAGE 10] payload drop failed: {result.message}; continuing the mission")


def centre_goal(c):
    cfg = c.cfg
    return CenterOnQr.Goal(target=to_point(c.state.target_qr), yaw=float(c.state.corridor_heading),
                           max_iters=int(cfg.center_iters), tol_px=float(cfg.center_tol),
                           gain=float(cfg.center_gain), approach_tolerance=float(cfg.qr_approach_tolerance),
                           hold_time=float(cfg.align_duration), hold_gain=float(cfg.hold_gain),
                           any_qr=not c.state.qr_matched)


def hold_countdown(c, feedback):
    c.countdown = float(feedback.hold_remaining)


def compute_blue_start(c):
    s, cfg = c.state, c.cfg
    s.blue_start, s.return_heading = mm.blue_search_start(
        s.green_banner, s.corridor_heading, cfg.corridor_surge + cfg.return_exit_gap, c.z(cfg.alt))
    c.log().info(f"[RETURN] Blue search start {cfg.corridor_surge + cfg.return_exit_gap:.1f} m from the green "
                 f"banner along the corridor: N={s.blue_start[0]:+.2f} E={s.blue_start[1]:+.2f} | "
                 f"return heading {math.degrees(s.return_heading):+.1f} deg, sway LEFT up to "
                 f"{cfg.blue_max_sway:.1f} m")


def store_blue_search(c, result, ok):
    if ok:
        c.state.blue_estimate = banner_estimate(c, result)


def store_blue_align(c, result, ok):
    s = c.state
    if not ok:
        return
    s.blue_banner = from_point(result.banner_position)
    s.return_yaw = result.corridor_heading
    c.log().info(f"[MEASUREMENT] Blue banner N={s.blue_banner[0]:+.2f} E={s.blue_banner[1]:+.2f} | "
                 f"return corridor heading {math.degrees(s.return_yaw):+.1f} deg "
                 f"({'measured' if result.heading_measured else 'expected, not measured'}) | lock: {result.locked}")


# ---------------------------------------------------------------- tree


def create_root(ctx: MissionContext):
    stage_1 = [
        ContextSetup(ctx),
        WaitForVehicle(ctx),
        Compute("STAGE 1: record pad height", ctx, record_ground),
        ActionStep("STAGE 1: TAKEOFF", ctx, Takeoff, TAKEOFF_ACTION, lambda c: Takeoff.Goal(altitude=c.cfg.alt)),
        Compute("STAGE 1: record take-off frame", ctx, record_takeoff_frame),
        goto("STAGE 1: stabilise over the pad", ctx, lambda c: c.state.takeoff, lambda c: c.state.takeoff_yaw),
        Hold("STAGE 1: hover", ctx, lambda c: c.cfg.stabilise_hold),
    ]
    stage_2_3 = [
        goto("STAGE 2: SURGE 1", ctx, lambda c: c.state.sp_surge1, lambda c: c.state.takeoff_yaw),
        Sel("STAGE 2: reference QR (optional)", memory=True, children=[
            # no timeout: hover, then growing squares round the surge-1 point until the QR is seen
            ReferenceCapture("STAGE 2: SCANNING REFERENCE QR (no timeout)", ctx, lambda c: c.state.sp_surge1,
                             lambda c: c.state.takeoff_yaw, store_reference),
            Compute("STAGE 2: no reference QR, skip stages 9-10", ctx, lambda c: None),
        ]),
        goto("STAGE 3: SURGE 2", ctx, lambda c: c.state.sp_surge2, lambda c: c.state.takeoff_yaw),
    ]

    qr_recon_inner = py_trees.decorators.FailureIsSuccess("STAGES 9-10 (optional)", child=Seq(
        "STAGES 9-10: QR reconnaissance", memory=True, children=[
            Check("STAGE 9: reference QR captured?", ctx, lambda c: c.state.ref_ok),
            Sel("STAGE 9: QR search", memory=True, children=[
                Seq("STAGE 9: target_search", memory=True, children=[
                    Check("STAGE 9: qr_search == target_search?", ctx,
                          lambda c: c.cfg.qr_search == "target_search"),
                    SetActive("STAGE 9: in the delivery zone: red-zone detection ON", ctx, RED_ACTIVE_SERVICE, True),
                    ActionStep("STAGE 9: TARGET SEARCH (RED-ZONE AWARE)", ctx, SearchTarget, "/search_target",
                               target_search_goal, store_search_target),
                    when("STAGE 9: navigator", ctx, lambda c: c.cfg.payload_drop,
                         ActionStep("STAGE 9: NAVIGATE TO THE TARGET QR", ctx, NavigateToTarget,
                                    "/navigate_to_target", navigate_goal, store_navigation)),
                ]),
                Seq("STAGE 9: lawnmower", memory=True, children=[
                    Check("STAGE 9: qr_search == lawnmower?", ctx, lambda c: c.cfg.qr_search == "lawnmower"),
                    ActionStep("STAGE 9: LAWNMOWER QR SEARCH", ctx, LawnmowerSearch, "/lawnmower_search",
                               lawnmower_goal, store_qr_target),
                ]),
            ]),
            ActionStep("STAGE 10: QR CENTERING + HOVER HOLD", ctx, CenterOnQr, "/center_on_qr", centre_goal,
                       on_feedback=hold_countdown),
            when("STAGE 10: payload", ctx, payload_mode,
                 py_trees.decorators.FailureIsSuccess("STAGE 10: drop failure does not stop the mission", child=
                     ActionStep("STAGE 10: PAYLOAD DROP", ctx, PayloadDrop, "/payload_drop",
                                lambda c: PayloadDrop.Goal(), store_drop))),
        ]))
    qr_recon = Seq("STAGES 9-10", memory=True, children=[
        qr_recon_inner,
        # also after a failed search / navigation / centring: the red zone is only in the delivery zone
        when("STAGE 10: leaving the delivery zone", ctx, lambda c: c.cfg.qr_search == "target_search",
             py_trees.decorators.FailureIsSuccess("STAGE 10: red off (best effort)", child=SetActive(
                 "STAGE 10: leaving the delivery zone: red-zone detection OFF", ctx, RED_ACTIVE_SERVICE, False))),
    ])

    return_lap = Seq("STAGES 11-12: return lap", memory=True, children=[
        sway_search("STAGE 11: SWAY LEFT (SEARCH BLUE BANNER)", ctx, "blue", lambda c: c.state.blue_start,
                    lambda c: c.state.return_heading, lambda c: c.cfg.blue_max_sway, store_blue_search),
        align("STAGE 11: BLUE BANNER ALIGNMENT", ctx, "blue", lambda c: c.state.blue_estimate,
              lambda c: c.state.return_heading, store_blue_align),
        traverse("STAGE 11: DESCEND + SURGE RETURN CORRIDOR + CLIMB", ctx, lambda c: c.state.blue_banner,
                 lambda c: c.state.return_yaw),
        goto("STAGE 12: RETURNING TO TAKEOFF PAD", ctx,
             lambda c: (c.state.takeoff[0], c.state.takeoff[1], c.z(c.cfg.alt)), lambda c: c.state.takeoff_yaw),
        goto("STAGE 12: low hover over the pad", ctx,
             lambda c: (c.state.takeoff[0], c.state.takeoff[1], c.z(c.cfg.home_low_alt)),
             lambda c: c.state.takeoff_yaw),
        Compute("STAGE 12: log", ctx, lambda c: c.log().info("MISSION OBJECTIVES ACHIEVED! COMMANDING LAND...")),
        ActionStep("STAGE 12: LANDING", ctx, Land, LAND_ACTION, lambda c: Land.Goal()),
    ])

    forward_lap = Seq("STAGES 4-12", memory=True, children=[
        sway_search("STAGE 4: SWAY LEFT (SEARCH GREEN BANNER)", ctx, "green", lambda c: c.state.sp_surge2,
                    lambda c: c.state.takeoff_yaw, lambda c: c.cfg.max_sway, store_green_search),
        align("STAGE 5: GREEN BANNER ALIGNMENT", ctx, "green", lambda c: c.state.green_estimate,
              lambda c: c.state.takeoff_yaw, store_green_align),
        traverse("STAGES 6-7: DESCEND + SURGE CORRIDOR + CLIMB", ctx, lambda c: c.state.green_banner,
                 lambda c: c.state.corridor_heading, store_corridor_exit),
        Compute("STAGE 8: compute wall offset target", ctx, compute_wall_target),
        goto("STAGE 8: MOVE LEFT (19 - x)", ctx, lambda c: c.state.sp_wall, lambda c: c.state.corridor_heading),
        qr_recon,
        Compute("STAGE 11: compute blue search start", ctx, compute_blue_start),
        goto("STAGE 11: FLY TO 1 m PAST THE CORRIDOR EXIT", ctx, lambda c: c.state.blue_start,
             lambda c: c.state.corridor_heading),
        goto("STAGE 11: TURNING 180 deg TO FACE THE CORRIDORS", ctx, lambda c: c.state.blue_start,
             lambda c: c.state.return_heading),
        Hold("STAGE 11: turn settle", ctx, lambda c: c.cfg.turn_hold),
        Sel("STAGE 11: blue banner or home", memory=True, children=[
            return_lap,
            return_home_and_land(ctx, "BLUE BANNER NOT FOUND - RETURNING HOME"),
        ]),
    ])

    mission = Seq("AeroTHON banner corridor mission", memory=True, children=stage_1 + stage_2_3 + [
        Sel("STAGE 4+: green banner or home", memory=True, children=[
            forward_lap,
            return_home_and_land(ctx, "GREEN BANNER NOT FOUND - RETURNING HOME"),
        ]),
    ])
    return py_trees.decorators.OneShot(name="Run once", child=mission,
                                       policy=py_trees.common.OneShotPolicy.ON_COMPLETION)


def main(args=None):
    rclpy.init(args=args)
    ctx = MissionContext()
    tree = py_trees_ros.trees.BehaviourTree(create_root(ctx))
    reported = []
    status_pub = []

    def post_tick(t):
        if ctx.node is None:
            return
        if not status_pub:
            status_pub.append(ctx.node.create_publisher(MissionStatus, MISSION_STATUS_TOPIC, DETECTION_QOS))
        tip = t.root.tip()
        msg = MissionStatus()
        msg.header.stamp = ctx.node.get_clock().now().to_msg()
        msg.stage = tip.name if tip else ""
        msg.state_text = msg.stage
        msg.countdown = float(ctx.countdown)
        msg.reference_ready = bool(ctx.state.ref_ok)
        status_pub[0].publish(msg)
        status = t.root.status
        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE) and not reported:
            reported.append(status)
            mission = t.root.children[0]
            if mission.status == py_trees.common.Status.SUCCESS and ctx.abort_reason:
                ctx.log().warn(f"MISSION ABORTED ({ctx.abort_reason}): returned to the pad and landed.")
            elif mission.status == py_trees.common.Status.SUCCESS:
                ctx.log().info("MISSION COMPLETED SUCCESSFULLY!")
            else:
                ctx.log().error(f"MISSION FAILED at '{tip.name if tip else '?'}'.")

    try:
        tree.setup(node_name="mission_tree", timeout=ctx.cfg.server_timeout * 12)
        tree.tick_tock(period_ms=100, post_tick_handler=post_tick)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
