# Robot tank otonom penghindar rintangan dengan pemetaan dan navigasi (ROS 2 Jazzy)

Tugas Akhir, Teknik Fisika ITERA (Cyber Physical System Lab).

## Perangkat
- Robot: DFRobot Devastator (2 motor DC, tanpa encoder dan IMU), driver L298N
- Pengendali motor: Waveshare ESP32-S3 Nano (firmware di `firmware/`)
- Sensor: Slamtec RPLiDAR A2M12
- Komputer onboard: Raspberry Pi 3B+, Ubuntu Server 24.04, ROS 2 Jazzy Base
- Laptop: Ubuntu 24.04, ROS 2 Jazzy (SLAM, Nav2, RViz, kontrol manual)

## Struktur
- `laptop/`: program yang berjalan di laptop. `nav.launch.py` (rf2o, slam_toolbox, Nav2, RViz, zona terlarang), `manual.launch.py` (twist_mux, stik, keyboard), `goal_keeper.py`, `teleop_manual.py`, dan alat bantu.
- `pi/`: program yang berjalan di Raspberry Pi (LiDAR, jembatan ke ESP32, penghindar rintangan). Menyusul.
- `firmware/`: firmware ESP32-S3 (kendali motor dan pembacaan tegangan baterai).
- `maps/`: peta ruangan dan topeng zona terlarang.
- `docs/versi.txt`: versi paket yang dipakai.

## Ketergantungan
ROS 2 Jazzy, nav2_bringup, slam_toolbox, twist_mux, joy, rplidar_ros, rf2o_laser_odometry (dibangun dari sumber), pyserial, pygame.

Penghindar rintangan dan Nav2 tidak boleh berjalan bersamaan karena sama-sama menerbitkan /cmd_vel.
