# Published by mission_tree every tick (HUD header line and countdown).
std_msgs/Header header
string stage          # name of the running tree step
string state_text     # short text for the HUD
float32 countdown     # s left in a timed hold (0 = none)
bool reference_ready
