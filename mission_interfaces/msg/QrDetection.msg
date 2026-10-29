# Output of qr_detector for one camera frame.
std_msgs/Header header        # copied from the image

bool detected                 # a QR code was found in this frame
float32[8] corners            # quad corners x0,y0,...,x3,y3 in px
float32 u                     # px, centre of the quad
float32 v
string decoded_text           # "" if it could not be decoded

bool reference_ready          # a reference QR has been captured from the camera
float32 score                 # 4-rotation module-grid similarity to the reference, 0..1 (1.0 for a text match)
bool matched                  # decoded text equals the reference text, or score >= match_threshold
float32 match_threshold
uint32 image_width
uint32 image_height

bool pose_valid
float32 alt_rel
float32 drone_x
float32 drone_y
float32 drone_z
float32 drone_heading
geometry_msgs/Point position_estimate  # local NED of the QR centre (pinhole reprojection), z = drone_z
