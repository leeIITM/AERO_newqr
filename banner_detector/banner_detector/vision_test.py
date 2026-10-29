"""Standalone vision tuning tool (section 8 of the original script). No ROS graph, no MAVLink, no flight.

    ros2 run banner_detector vision_test --color green
    ros2 run banner_detector vision_test --color blue --camera-src 0 --alt 5
    ros2 run banner_detector vision_test --color green --image field.jpg     # still image

Shows every colour region with its rotated rectangle, centre, ratio, fill, full-view status and verdict.
Keys: 'm' toggles the colour-mask overlay, 'q' / ESC quits.
"""
import argparse
import os
import time

import cv2
import numpy as np

from banner_detector.detector import BannerDetector, BannerParams
from bc_common.stream import SiyiStreamWorker, default_rtsp_url


def build_parser():
    p = argparse.ArgumentParser(description="AeroTHON banner HSV / shape tuning tool",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--color", choices=["green", "blue"], default="green")
    p.add_argument("--camera-src", default=None, help="RTSP URL or device index (default: SIYI A8 mini RTSP)")
    p.add_argument("--stream", choices=["main", "sub"], default="main")
    p.add_argument("--image", default=None, help="Run on a still image instead of a camera")
    p.add_argument("--alt", type=float, default=5.0, help="Altitude used for the physical size check (m)")
    p.add_argument("--hfov-rad", type=float, default=1.22)
    defaults = BannerParams()
    for name in BannerParams.names():
        value = getattr(defaults, name)
        flag = "--" + name.replace("_", "-")
        if isinstance(value, bool):
            p.add_argument(flag, type=lambda s: s.lower() in ("1", "true", "yes", "on"), default=value)
        else:
            p.add_argument(flag, type=type(value), default=value)
    return p


def draw(frame, banner, diags, mask, color, detector, show_mask):
    if show_mask:
        mask_bgr = cv2.cvtColor(cv2.resize(mask, (frame.shape[1], frame.shape[0])), cv2.COLOR_GRAY2BGR)
        vis = cv2.addWeighted(frame, 0.4, mask_bgr, 0.6, 0)
    else:
        vis = frame.copy()
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
    for diag in diags:
        quad = np.int32(diag["quad"]).reshape(-1, 2)
        is_banner = "BANNER" in diag["verdict"]
        col = (0, 255, 0) if is_banner else (0, 140, 255)
        cv2.polylines(vis, [quad], True, col, 3 if is_banner else 2)
        tcx, tcy = int(diag["center"][0]), int(diag["center"][1])
        cv2.circle(vis, (tcx, tcy), 5, (0, 0, 255), -1)
        edge = "CUT" if diag["touching_edge"] else "FULL"
        hue = f" H:{diag['hue_med']:.0f}" if diag.get("hue_med") is not None else ""
        txt = (f"R:{diag['ratio']:.1f} F:{diag['fill']:.2f} L:{diag['length_m']:.1f}m{hue} [{edge}] "
               f"-> {diag['verdict']}")
        cv2.putText(vis, txt, (tcx - 60, max(20, tcy - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        if not is_banner:
            cv2.putText(vis, diag["reason"], (tcx - 60, max(36, tcy + 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
    cv2.rectangle(vis, (0, 0), (w, 42), (20, 20, 20), -1)
    h_c, h_tol, s_min, v_min = detector.color_params(color)
    info = (f"COLOR: {color.upper()} | H: {h_c}+/-{h_tol} [{h_c - h_tol},{h_c + h_tol}] "
            f"S_min: {s_min} V_min: {v_min} | Mask: {'ON' if show_mask else 'OFF'} ('m' to toggle)")
    cv2.putText(vis, info, (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    if banner is not None:
        cv2.polylines(vis, [np.int32(banner["quad"]).reshape(-1, 2)], True, (0, 255, 0), 4)
        cv2.putText(vis, f"*** CLASSIFIED {color.upper()} BANNER ({banner['source']}) ***", (15, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return vis


def main(argv=None):
    args = build_parser().parse_args(argv)
    params = BannerParams(**{n: getattr(args, n) for n in BannerParams.names()})
    detector = BannerDetector(params, verbose=True)
    still = cv2.imread(args.image) if args.image else None
    if args.image and still is None:
        raise SystemExit(f"cannot read {args.image}")
    worker = None
    if still is None:
        src = args.camera_src or default_rtsp_url(args.stream)
        worker = SiyiStreamWorker(src)
    headless = not ("DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ)
    window = f"AeroTHON Vision Test - {args.color.upper()} Banner"
    if not headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 960, 540)
    show_mask = False
    try:
        while True:
            frame = still if still is not None else worker.get_frame()
            if frame is None:
                vis = np.zeros((540, 960, 3), dtype=np.uint8)
                cv2.putText(vis, "CONNECTING TO CAMERA...", (260, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # a still image is shown to the persistence check as banner_persistence identical frames
                for _ in range(params.banner_persistence if still is not None else 1):
                    banner, diags, mask = detector.detect(frame, color=args.color, alt_rel=args.alt,
                                                          hfov_rad=args.hfov_rad)
                vis = draw(frame, banner, diags, mask, args.color, detector, show_mask)
                if headless:
                    out = "vision_test_result.png"
                    cv2.imwrite(out, vis)
                    print(f"No display: wrote {out}. Banner: {banner is not None}")
                    for d in diags:
                        hue = f" hue={d['hue_med']:.0f}" if d.get("hue_med") is not None else ""
                        print(f"  {d['verdict']:15s} ratio={d['ratio']:.1f} fill={d['fill']:.2f} "
                              f"len={d['length_m']:.2f}m{hue} {d['reason']}")
                    if still is not None:
                        break
            if not headless:
                cv2.imshow(window, vis)
                key = cv2.waitKey(30 if still is not None else 1) & 0xFF
                if key == ord("m"):
                    show_mask = not show_mask
                elif key in (ord("q"), 27):
                    break
            elif still is None:
                time.sleep(0.5)
    finally:
        detector.stop()
        if worker is not None:
            worker.stop()
        if not headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
