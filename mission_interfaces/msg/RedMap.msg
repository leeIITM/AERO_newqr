# The red seen so far (target_search, /perception/red_map, latched). Grid in the search's zone frame:
#   x_z along `axis_heading` from (origin_north, origin_east), y_z 90 deg to the left of x_z
#   (a rotation of the local NED frame aligned with the delivery zone's long side).
# Consumers (qr_centering, target_search's SafeGoto) rebuild red_avoidance.RedMap from it.
std_msgs/Header header
float64 origin_north
float64 origin_east
float64 axis_heading      # rad, heading of +x_z
float32 cell              # m per cell
float32 grid_x0           # zone-frame coordinates of the grid's corner
float32 grid_y0
uint32 width              # cells along x_z
uint32 height             # cells along y_z
uint8[] data              # row-major (row = y index), 1 = red seen
uint32 version
float32 margin            # keep-out margin the search uses (m)
