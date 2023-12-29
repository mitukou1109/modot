import glob
import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.dirname(__file__))
from generate_sound import generate_sound

tts_messages: dict[str, tuple[str, str]] = {}

face_image_files = glob.glob(
    os.path.join(
        get_package_share_directory("face_recognition_ros"),
        "resource",
        "known_faces",
        "**",
    ),
    recursive=True,
)

if not face_image_files:
    print("No known face registered")
    exit()
else:
    print("Input the name of known faces")

for face_image_file in face_image_files:
    if os.path.isfile(face_image_file):
        face_filename = os.path.splitext(os.path.basename(face_image_file))[0]
        if face_name := input(f"{face_filename}: "):
            tts_messages[f"face_{face_filename}"] = (
                face_name,
                "ja",
            )

generate_sound(tts_messages)
