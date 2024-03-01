# modot

Robot for the visually impaired

## Setup & launch

### Requirements

- ROS 2 Humble
- [librealsense](https://github.com/IntelRealSense/librealsense)
  - Check [releases](https://github.com/IntelRealSense/realsense-ros/releases) for the supported version

```
$ mkdir -p ~/modot_ws/src
$ cd modot_ws
$ source <(wget -O - https://raw.githubusercontent.com/mitukou1109/modot/master/setup.sh)

# Edge TPU setup (optional)
$ source <(wget -O - https://raw.githubusercontent.com/mitukou1109/modot/master/setup_edgetpu.sh)

# known face registration (optional)
$ python3 src/face_recognition_ros/register_known_face.py <path to image> [name]
$ python3 src/modot/modot_notification/generate_face_sound.py

$ ros2 launch modot_bringup bringup.launch
```

## Related packages

- [ultralytics_ros](https://github.com/mitukou1109/ultralytics_ros)
- [face_recognition_ros](https://github.com/mitukou1109/face_recognition_ros)
- [modot_raspi](https://github.com/mitukou1109/modot_raspi)
- ~~[modot_uros](https://github.com/mitukou1109/modot_uros)~~
