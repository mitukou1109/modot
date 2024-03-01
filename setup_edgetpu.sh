[ -e /usr/share/keyrings/google-cloud.gpg ] || { curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo gpg --dearmor -o /usr/share/keyrings/google-cloud.gpg; }
[ -e /etc/apt/sources.list.d/coral-edgetpu.list ] || { echo "deb [signed-by=/usr/share/keyrings/google-cloud.gpg] https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list; }
sudo apt update
sudo apt install libedgetpu1-std edgetpu-compiler

if [[ -e src/ultralytics_ros/requirements_edgetpu.txt ]]; then
  pip install -r src/ultralytics_ros/requirements_edgetpu.txt
else
  echo "./src/ultralytics_ros/requirements_edgetpu.txt not found."
  echo "Install dependencies for ultralytics_ros with the following command before use:"
  echo "\tpip install -r <path to workspace>/ultralytics_ros/requirements_edgetpu.txt"
fi