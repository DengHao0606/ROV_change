#agx
ros2 run image_transport republish raw compressed --ros-args --remap in:=/image_data --remap out/compressed:=/image_data/compressed
#wsl
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/image_data/compressed --remap out:=/image_data_uncompressed