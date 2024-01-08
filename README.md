# modot

Robot for the blind

## Setup & launch

```
$ mkdir -p ~/modot_ws/src
$ cd modot_ws
$ source <(wget -O - https://raw.githubusercontent.com/mitukou1109/modot/master/setup.sh)
$ python3 src/modot/face_recognition_ros/register_known_face.py <path to image> [name]
$ python3 src/modot/modot_notification/generate_face_sound.py
$ ros2 launch modot_bringup bringup.launch
```