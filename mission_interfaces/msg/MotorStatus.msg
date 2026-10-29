# Bench motor telemetry (bench_flight_controller with spin_motors:=true), shown on the HUD.
std_msgs/Header header
uint16[4] servo_pwm   # SERVO_OUTPUT_RAW / RC out channels 1..4
float32 thrust        # 0..1 commanded collective thrust
float32 cmd_roll_deg
float32 cmd_pitch_deg
