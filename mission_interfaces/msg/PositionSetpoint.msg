# Streamed position target for flight_controller (~/setpoint), local NED. Used by target_search to
# follow the red-zone avoidance carrot point (published at ~20 Hz). Accepted only while GUIDED is
# engaged and no action goal is running; if the stream stops for 0.5 s the vehicle holds position.
std_msgs/Header header
float32 x            # north, m
float32 y            # east, m
float32 z            # down, m
float32 yaw          # heading, rad (0 = north, clockwise); NaN = keep the current heading
