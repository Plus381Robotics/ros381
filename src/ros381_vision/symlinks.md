Stable Camera Device Naming with udev

Purpose

Ensure that the CSI camera and USB webcam always have the same /dev/ symlinks, e.g., /dev/csi_camera and /dev/usb_camera, regardless of boot order.

1. Create udev rules file

Run:

sudo nano /etc/udev/rules.d/99-cameras.rules

Paste the following:

CSI camera (unicam)

SUBSYSTEM=="video4linux", ATTR{name}=="unicam-image", SYMLINK+="csi_camera"

USB webcam (uvcvideo)

SUBSYSTEM=="video4linux", ATTR{name}=="GENERAL WEBCAM: GENERAL WEBCAM", SYMLINK+="usb_camera"

ATTR{name} → exactly what udevadm info -a -p /sys/class/video4linux/videoX shows for ATTR{name}

SYMLINK+="..." → the stable device name you want

2. Reload udev rules

sudo udevadm control --reload-rules
sudo udevadm trigger

This will create /dev/csi_camera and /dev/usb_camera immediately.

3. Test

ls -l /dev/csi_camera
ls -l /dev/usb_camera

Expected output:

/dev/csi_camera -> video2
/dev/usb_camera -> video0

4. Use in ROS2 or FFplay

ROS2:

ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/csi_camera" -p image_size:="[640,480]" -p framerate:=30 -p output_encoding:="mono8"

FFplay:

ffplay -f v4l2 -video_size 640x480 -pixel_format yuyv422 /dev/csi_camera