"""HUD drawing (the original script's update_hud), ROS-free."""
import time

import cv2
import numpy as np


class HudOverlay:
    RETAIN = 1.2  # s a target box stays on screen after it was last seen

    def __init__(self, match_thresh=0.88):
        self.match_thresh = match_thresh
        self.cache = None  # (pts, type, score, subtext, time)

    def draw(self, frame, target_pts=None, target_type=None, score=0.0, subtext="", state_text="",
             pose=None, ref_thumb=None, countdown=0.0, motors=None, red_outlines=None):
        vis = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        h, w = vis.shape[:2]
        cx, cy = w // 2, h // 2
        if red_outlines:  # red / restricted zones in this frame: translucent fill + outline
            overlay = vis.copy()
            polys = [np.int32(np.asarray(o).reshape(-1, 2)) for o in red_outlines if len(o) >= 3]
            cv2.fillPoly(overlay, polys, (0, 0, 255))
            cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
            cv2.polylines(vis, polys, True, (0, 0, 255), 2)
            for poly in polys:
                c = poly.mean(axis=0).astype(int)
                cv2.putText(vis, "RED ZONE", (int(c[0]) - 40, int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
        cv2.drawMarker(vis, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
        cv2.circle(vis, (cx, cy), 18, (0, 255, 255), 1)

        now = time.monotonic()
        if target_pts is not None and len(target_pts) > 0:
            self.cache = (target_pts, target_type, score, subtext, now)
        elif self.cache is not None and now - self.cache[4] < self.RETAIN:
            target_pts, target_type, score, subtext, _ = self.cache

        if target_pts is not None and len(target_pts) > 0:
            p = np.int32(np.asarray(target_pts).reshape(-1, 2))
            if target_type == "QR":
                col = (0, 255, 0) if score >= self.match_thresh else (0, 165, 255)
                lbl = f"QR TARGET [Sim: {score * 100:.1f}%]" if score > 0 else f"QR CODE [{subtext}]"
            elif target_type == "BLUE_BANNER":
                col, lbl = (255, 140, 0), f"BLUE BANNER [{subtext}]"
            else:
                col, lbl = (0, 255, 0), f"GREEN BANNER [{subtext}]"
            cv2.polylines(vis, [p], True, col, 3)
            for pt, name, (dx, dy) in zip(p[:4], ("TL", "TR", "BR", "BL"), ((-25, -10), (10, -10), (10, 20), (-25, 20))):
                cv2.circle(vis, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
                cv2.circle(vis, (int(pt[0]), int(pt[1])), 7, col, 1)
                cv2.putText(vis, f"{name}({int(pt[0])},{int(pt[1])})", (int(pt[0]) + dx, int(pt[1]) + dy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
            tcx, tcy = int(np.mean(p[:, 0])), int(np.mean(p[:, 1]))
            cv2.circle(vis, (tcx, tcy), 7, (0, 0, 255), -1)
            cv2.circle(vis, (tcx, tcy), 12, (255, 255, 255), 1)
            cv2.line(vis, (cx, cy), (tcx, tcy), (255, 255, 0), 2)
            (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)
            lx, ly = max(10, min(w - tw - 15, tcx - tw // 2)), max(35, tcy - 20)
            cv2.rectangle(vis, (lx - 5, ly - th - 5), (lx + tw + 5, ly + 5), (20, 20, 20), -1)
            cv2.rectangle(vis, (lx - 5, ly - th - 5), (lx + tw + 5, ly + 5), col, 1)
            cv2.putText(vis, lbl, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.58, col, 2)
            cv2.putText(vis, f"C({tcx},{tcy}) Offset: dx={tcx - cx:+d}px dy={tcy - cy:+d}px", (lx, ly + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)

        cv2.rectangle(vis, (0, 0), (w, 38), (20, 20, 20), -1)
        cv2.putText(vis, f"MISSION: {state_text}"[:70], (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if pose is not None:
            alt_str = (f"Alt: {pose['alt']:.2f}m | NED: ({pose['x']:+.1f}, {pose['y']:+.1f}) | "
                       f"Hdg {pose['heading_deg']:+.0f} | {pose['mode']}")
            cv2.putText(vis, alt_str, (max(300, w - 520), 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

        if ref_thumb is not None:
            thumb = cv2.resize(ref_thumb, (80, 80))
            thumb = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR) if thumb.ndim == 2 else thumb
            vis[45:125, w - 90:w - 10] = thumb
            cv2.rectangle(vis, (w - 92, 43), (w - 8, 127), (0, 255, 0), 2)
            cv2.putText(vis, "REF QR", (w - 85, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)

        if countdown > 0:
            cv2.rectangle(vis, (cx - 160, h - 52), (cx + 160, h - 12), (0, 110, 0), -1)
            cv2.putText(vis, f"ALIGN HOLD: {countdown:.1f}s", (cx - 145, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 0), 2)

        if motors is not None:
            cv2.rectangle(vis, (10, h - 72), (560, h - 8), (20, 20, 20), -1)
            cv2.rectangle(vis, (10, h - 72), (560, h - 8), (0, 255, 255), 1)
            pwm = list(motors["pwm"])
            rps = [max(0, int((s - 1000) * 0.12)) for s in pwm]
            txt = "MOTORS: " + " ".join(f"M{i + 1}={s}(~{r}rps)" for i, (s, r) in enumerate(zip(pwm, rps)))
            tilt = (f"Cmd Tilt: Roll={motors['roll']:+.1f} deg | Pitch={motors['pitch']:+.1f} deg | "
                    f"Thrust {motors['thrust'] * 100:.0f}%")
            cv2.putText(vis, txt, (15, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)
            cv2.putText(vis, tilt, (15, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1)
        return vis
