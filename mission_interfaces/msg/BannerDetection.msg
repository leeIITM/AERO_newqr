# Output of banner_detector for one camera frame (published only while a colour is active).
std_msgs/Header header        # copied from the image

string color                  # "green" | "blue": the colour searched for in this frame
bool detected                 # a banner passed classification in this frame
string source                 # "SHAPE" (ratio/fill/full-view + persistence) | "OCR" (AeroTHON + 2026 read)
string text

float32 u                     # px, banner centre (reconstructed along the long axis for a partial OCR view)
float32 v
float32[8] quad               # TL, TR, BR, BL corners x0,y0,...,x3,y3 in px
float32 angle                 # rad, long-axis angle in the image (x right, y down), [-pi/2, pi/2)
bool partial                  # quad touches the image border
uint32 image_width
uint32 image_height
uint16 candidates             # colour regions above min area in this frame
uint16 consecutive_hits       # shape persistence counter

# Drone pose used for this frame (from /flight_controller/pose) and the banner position it gives
bool pose_valid
float32 alt_rel               # m above take-off
float32 drone_x
float32 drone_y
float32 drone_z
float32 drone_heading
geometry_msgs/Point position_estimate  # local NED of the banner centre (pinhole reprojection), z = drone_z
