# Vehicle state published by flight_controller (MAVROS) or bench_flight_controller on
# /flight_controller/pose. Every mission node reads the drone pose from here, so the bench
# controller can stand in for the real one without any other node noticing.
std_msgs/Header header

bool valid            # local position is valid
bool ready            # autopilot connected, booted and has a GPS fix (bench: always true)
bool armed
bool airborne
string mode           # flight mode as reported by the autopilot ("GUIDED", "GUIDED_NOGPS", ...)
string controller     # "mavros" | "bench"

# Local NED position from the EKF origin (m), heading 0 = north clockwise (rad)
float32 x
float32 y
float32 z
float32 heading
float32 vx
float32 vy
float32 vz

float32 alt_rel       # m above home (take-off point); NaN if unknown

# Attitude (NED earth, FRD body), rad. Used to drop camera frames taken while banking hard.
float32 roll
float32 pitch

# Geographic position of the local frame's origin (geofence files are lat/lon)
bool xy_global
float64 ref_lat
float64 ref_lon
