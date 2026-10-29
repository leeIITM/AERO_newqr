"""Mission parameters of mission_tree (ROS parameters with blue_green_qr_fixed.py's option names/defaults)."""
from dataclasses import dataclass, fields


@dataclass
class MissionConfig:
    # flight stages (script --alt, --surge1, ...). Heights are metres above the take-off pad.
    alt: float = 5.0                    # take-off, transit, banner search and QR search altitude
    surge1: float = 2.0                 # m forward to the reference QR
    surge2: float = 2.0                 # m forward after the reference QR
    max_sway: float = 20.0              # m left along the take-off left axis searching for the green banner
    sway_speed: float = 0.5             # m/s
    sway_end_hover: float = 3.0         # s hover at max sway to finish the persistence check
    corridor_alt: float = 3.0           # altitude inside both corridors
    corridor_surge: float = 10.0        # m through each corridor
    corridor_speed: float = 0.9         # m/s through the corridors
    corridor_settle: float = 1.0        # s hover after descending into a corridor
    wall_offset: float = 19.0           # lateral shift after the corridor = wall_offset - sway x
    wall_forward: float = 1.0           # m forward with the lateral shift
    speed: float = 1.2                  # m/s general GoTo speed
    stabilise_hold: float = 1.5         # s hover over the pad after take-off
    # stage 2 reference QR: no timeout. Hover over the surge-1 point; if no QR is seen after ref_search_hover
    # s, fly growing squares round it (ref_search_step, 2x, ... up to ref_search_max m) until one is seen.
    ref_search_hover: float = 10.0      # s hovering over the surge-1 point before searching round it
    ref_search_step: float = 1.0        # m, first square's half-size and the growth per square
    ref_search_max: float = 3.0         # m, largest square's half-size (then it starts again at step)
    ref_search_dwell: float = 1.5       # s hover at each corner
    ref_search_speed: float = 0.5       # m/s
    # stage 9 QR search: "target_search" (geofenced search with red-zone detection + avoidance) or
    # "lawnmower" (the script's serpentine, no red avoidance)
    qr_search: str = "target_search"
    # target_search only: target_navigator flies to the found QR, qr_centering centres + holds, then
    # payload_drop sends the two PWMs (also over the best candidate). false = the search flies there itself
    # and nothing is dropped.
    payload_drop: bool = True
    # target_search (suas_aer target_search + Shiva007-RAndom red-zone avoidance)
    geofence_file: str = ""             # delivery zone corners, one "lat,lon" per line (required for target_search)
    search_wall_margin: float = 1.0     # m the flight area is kept inside the geofence
    search_overlap: float = 0.2         # overlap of neighbouring rows' camera footprints
    search_rows: int = 0                # 0 = as many as the zone needs
    search_yaw_mode: str = "wide_ahead"  # wide_ahead: camera's long side along the rows (sees red sooner);
                                        # corridor: hold the corridor heading
    red_zone_margin: float = 2.0        # m kept from any red (repo --margin)
    confirm_matches: int = 2            # matched detections of one QR needed (repo --confirm)
    max_revisits: int = 0               # fly over ground the camera missed; 0 = off
    revisit_time: float = 300.0         # s budget for those revisits
    # lawnmower (script --lawnmower-length, --step)
    lawnmower_length: float = 18.0      # m per row, along the corridor heading
    lawnmower_step: float = 4.0         # m between rows, stepping right
    lawnmower_rows: int = 5
    lawnmower_tolerance: float = 0.6    # m, row-end waypoint acceptance
    # stage 10 QR centring + hold (script --align-duration, --center-*)
    align_duration: float = 15.0        # s hover hold over the matched QR
    center_tol: float = 25.0            # px
    center_iters: int = 6
    center_gain: float = 0.7
    hold_gain: float = 0.5              # fraction of the offset per 1 Hz nudge during the hold
    qr_approach_tolerance: float = 0.25  # m
    # banner servoing (green and blue)
    banner_settle: float = 0.5          # s after each correction (the GoTo already waits for arrival)
    max_heading_correction_deg: float = 25.0
    # stage 11 return corridor (script --return-exit-gap, --blue-max-sway)
    return_exit_gap: float = 1.0        # blue search starts corridor_surge + this (11 m) from the green banner
    blue_max_sway: float = 20.0         # m left (facing back down the corridor) searching for the blue banner
    turn_hold: float = 3.0              # s turning to the return heading
    # landing
    home_low_alt: float = 2.0           # m: low hover over the pad before landing
    # plumbing
    server_timeout: float = 30.0        # s to wait for each action server at start-up

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]

    def load(self, node):
        for name in self.names():
            setattr(self, name, type(getattr(self, name))(node.declare_parameter(name, getattr(self, name)).value))
        self.validate()
        return self

    def validate(self):
        if self.qr_search not in ("target_search", "lawnmower"):
            raise ValueError("qr_search must be 'target_search' or 'lawnmower'")
        if self.search_yaw_mode not in ("wide_ahead", "corridor"):
            raise ValueError("search_yaw_mode must be 'wide_ahead' or 'corridor'")
        if self.qr_search == "target_search" and not self.geofence_file:
            raise ValueError("qr_search target_search needs geofence_file (delivery zone corners, lat,lon per line)")
