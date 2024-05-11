[ -d src ] || { echo "$(pwd)/src not found, run this script in workspace root"; exit 1; }

vcs import src --input https://raw.githubusercontent.com/mitukou1109/modot/master/modot.repos

rosdep install -iyr --from-paths src

pip install -r src/face_recognition_ros/requirements.txt
pip install -r src/ultralytics_ros/requirements.txt

colcon build --symlink-install
source install/local_setup.sh

python3 src/modot/modot_notification/generate_sound.py

echo "Done!"
