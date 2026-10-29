# Result of red-patch detection on one image (red_zone_detector, /perception/red_zone).
# Ported from qr_match_interfaces/RedZoneResult (Shiva007-RAndom/Qr-and-red-zone-detection).
# Published for EVERY processed frame (detected = false too): target_search uses the frames without
# red to know which ground the camera has looked at.
std_msgs/Header header          # copied from the image (camera time)

uint32 image_width
uint32 image_height
bool detected
uint16 num_patches
float32 total_area_fraction
RedPatch[] patches              # by descending area
float32 processing_time_ms
