import numpy as np

from mission_hud.overlay import HudOverlay


def test_overlay_draws_everything():
    hud = HudOverlay()
    frame = np.zeros((720, 1280, 3), np.uint8)
    out = hud.draw(frame, [(100, 100), (300, 100), (300, 140), (100, 140)], "GREEN_BANNER", 0.0, "SHAPE",
                   "STAGE 5", {"alt": 5.0, "x": 1.0, "y": 2.0, "heading_deg": 0.0, "mode": "GUIDED"},
                   np.zeros((210, 210), np.uint8), 12.3, {"pwm": [1200] * 4, "roll": 1.0, "pitch": -2.0, "thrust": 0.2})
    assert out.shape == frame.shape and out.any()
    # retained for 1.2 s after the target disappears
    out2 = hud.draw(frame, None, None)
    assert (out2[100:141, 100:301] != 0).any()
