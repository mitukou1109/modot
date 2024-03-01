[ -d src ] || { echo "$(pwd)/src not found, run this script in workspace root"; exit 1; }

[ -e /usr/share/keyrings/google-cloud.gpg ] || { curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo gpg --dearmor -o /usr/share/keyrings/google-cloud.gpg; }
[ -e /etc/apt/sources.list.d/coral-edgetpu.list ] || { echo "deb [signed-by=/usr/share/keyrings/google-cloud.gpg] https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list; }
sudo apt update
sudo apt install libedgetpu1-std edgetpu-compiler

vcs import src --input https://raw.githubusercontent.com/mitukou1109/modot/master/modot.repos

rosdep install -iyr --from-paths src

pip install -r src/face_recognition_ros/requirements.txt
pip install -r src/ultralytics_ros/requirements.txt
pip install -r src/ultralytics_ros/requirements_edgetpu.txt

colcon build --symlink-install
source install/local_setup.sh

python3 src/modot/modot_notification/generate_sound.py

sudo cp src/modot/modot_bringup/config/60-modot.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger

echo "Done!"
