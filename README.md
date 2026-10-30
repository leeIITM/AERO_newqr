# AeroTHON 2026 — Banner Corridor & Precision QR Mission

Autonomous navigation, visual banner servoing, and precision QR reconnaissance stack for ArduCopter & SIYI A8 mini on **Raspberry Pi (Ubuntu 24.04 LTS / ROS 2 Jazzy)** and PC (Ubuntu 22.04 LTS / ROS 2 Humble).

---

## 📋 System Requirements & Hardware Connections

* **Onboard Computer:** Raspberry Pi 5 (or Pi 4B) running Ubuntu 24.04 LTS (Noble) with ROS 2 Jazzy.
* **Camera:** SIYI A8 mini connected to Raspberry Pi via **Ethernet RJ45** (RTSP Stream).
  * Factory Camera IP: `192.168.144.25`
  * RTSP Stream URL: `rtsp://192.168.144.25:8554/main.264`
* **Flight Controller:** Cube Orange+ / Pixhawk running ArduCopter 4.5+ connected via USB CDC (`/dev/ttyACM0` or `/dev/serial/by-id/...`).

---

## 🚀 1. One-Time Setup on Raspberry Pi

SSH into the Raspberry Pi:
```bash
ssh pi@<raspberry-pi-ip>
```

### A. Add User to Dialout Group (for USB Pixhawk Access)
```bash
sudo usermod -aG dialout $USER
```
*(Log out and log back in for group permissions to take effect).*

### B. Install System & ROS 2 Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-mavros \
  ros-jazzy-mavros-msgs \
  ros-jazzy-mavros-extras \
  ros-jazzy-py-trees \
  ros-jazzy-py-trees-ros \
  ros-jazzy-geographic-msgs \
  python3-pyzbar \
  libzbar0 \
  python3-piexif \
  python3-opencv \
  python3-numpy \
  python3-colcon-common-extensions

# Install GeographicLib datasets for MAVROS (GPS coordinates/geoid height conversion)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

---

## 🌐 2. Network Configuration for SIYI Camera

The camera streams on `192.168.144.25`. The Pi's Ethernet interface (`eth0`) must be assigned an IP in the `192.168.144.0/24` subnet.

### Temporary Assignment:
```bash
sudo ip addr add 192.168.144.35/24 dev eth0
sudo ip link set eth0 up
```

### Permanent Assignment (Netplan):
Edit `/etc/netplan/01-netcfg.yaml` (or existing netplan yaml):
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.144.35/24
```
Apply the configuration:
```bash
sudo netplan apply
```

### Verify Camera Link:
```bash
ping -c 2 192.168.144.25
```

---

## 🔨 3. Build the Workspace

```bash
cd ~/AERO_newqr
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

*(Tip: Add `source /opt/ros/jazzy/setup.bash` and `source ~/AERO_newqr/install/setup.bash` to your `~/.bashrc`).*

---

## 🎯 4. Execution Workflows

### Mode 1: HITL Mission (Hardware-In-The-Loop)
**Use this for bench-testing the whole mission:** ArduCopter SITL runs locally on the Pi simulating GPS coordinates and flight dynamics, while the **real SIYI A8 mini camera** feeds live video to the visual detectors and state machine.

```bash
cd ~/AERO_newqr
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch bc_bringup hitl_mission.launch.py \
    mavros_terminal:=none \
    show_window:=false
```

---

### Mode 2: Real Drone Flight (Hardware Deployment)
**Use this for actual autonomous field flights:** MAVROS connects to the physical Cube/Pixhawk flight controller over USB, and the real SIYI camera feeds live visual tracking.

```bash
cd ~/AERO_newqr
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch bc_bringup drone.launch.py \
    fcu_url:=/dev/ttyACM0:115200 \
    geofence_file:=$(ros2 pkg prefix bc_bringup)/share/bc_bringup/config/delivery_zone.sim.txt \
    mavros_terminal:=none \
    show_window:=false
```

*Optional Real-Flight Arguments:*
* Forward MAVLink to Ground Control Station (QGC) over Wi-Fi:
  `gcs_url:=udp://@192.168.1.10:14550`
* Lawnmower QR search without geofence:
  `qr_search:=lawnmower`

---

### Mode 3: Vision Detectors Standalone (Camera Bench Test)
To verify camera streaming and visual detector recognition (point the SIYI camera at targets on the ground):

#### Terminal 1: Start Camera Driver
```bash
ros2 run siyi_camera rtsp_camera_node
```

#### Terminal 2: Start QR & Banner Detectors
```bash
# Start continuous QR detector
ros2 run qr_detector qr_detector_node --ros-args -p active:=true &

# Start Banner detector (detect green)
ros2 run banner_detector banner_detector_node --ros-args -p color:=green &
```

#### Inspect Live Output:
```bash
# View QR detection messages:
ros2 topic echo /perception/qr

# View Banner detection messages:
ros2 topic echo /perception/banner
```

---

## 💻 5. Remote GUI Monitoring from Ground Laptop

Because the Raspberry Pi runs headless over SSH, you can view the live OpenCV HUD window (camera feed + detection bounding quads + telemetry overlays) from your ground laptop over the same Wi-Fi network:

1. **On both the Raspberry Pi and Laptop**, set the same ROS Domain ID in `~/.bashrc`:
   ```bash
   export ROS_DOMAIN_ID=42
   ```

2. **On your Laptop**, run the HUD viewer:
   ```bash
   cd ~/AERO_newqr
   source /opt/ros/humble/setup.bash
   source install/setup.bash

   ros2 run mission_hud hud_node --ros-args -p show_window:=true
   ```
   *(The window will render the camera frames streamed over ROS 2 from the Pi in real-time).*

---

## 🛠️ Key Topics & Services

| Name | Type | Description |
| :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | Live BGR8 frames from SIYI camera (15–30 Hz). |
| `/flight_controller/pose` | `mission_interfaces/VehicleState` | Local NED position, velocity, armed state, and mode. |
| `/perception/qr` | `mission_interfaces/QrDetection` | Detected QR polygon, decoded payload, and similarity score. |
| `/perception/banner` | `mission_interfaces/BannerDetection` | Detected banner center, heading angle, and alignment error. |
| `/mission/status` | `mission_interfaces/MissionStatus` | Current active stage of the autonomous behavior tree. |
| `/qr_detector/capture_reference` | `mission_interfaces/CaptureReference` | Action to memorize the initial reference QR. |
| `/qr_detector/set_active` | `std_srvs/SetBool` | Enable/disable continuous frame processing. |
| `/banner_detector/set_color` | `mission_interfaces/SetBannerColor` | Set target banner color (`green`, `blue`, or `""`). |
