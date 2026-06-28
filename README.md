# Rosbag2 stuff:
## record rosbag
ros2 bag record /ros381/camera_info /ros381/image_raw/compressed /ros381/odom
## rosbag to video
ros2 bag to_video bag/rosbag2_2026_06_28-17_25_44/ -t /ros381/image_raw/compressed -o output.mp4 --fps 30.0