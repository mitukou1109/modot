import glob
import os

import ffmpeg
from ament_index_python.packages import get_package_share_directory
from gtts import gTTS

SOUND_DIR = os.path.join(
    get_package_share_directory("modot_notification"), "resource", "sound"
)

SOUND_EXT = ".wav"

tts_messages: dict[str, tuple[str, str]] = {
    "obstacle": ("障害物があります", "ja"),
    "signal_blue": ("青信号です", "ja"),
    "signal_red": ("赤信号です", "ja"),
    "direction_left": ("左に", "ja"),
    "direction_right": ("右に", "ja"),
    "direction_front": ("前に", "ja"),
    "exist": ("が、います", "ja"),
}

for known_face_image_file in glob.glob(
    os.path.join(
        get_package_share_directory("face_recognition_ros"),
        "resource",
        "known_faces",
        "**",
    ),
    recursive=True,
):
    if os.path.isfile(known_face_image_file):
        known_face_name = os.path.splitext(os.path.basename(known_face_image_file))[0]
        tts_messages[f"face_{known_face_name}"] = (known_face_name, "ja")

os.makedirs(SOUND_DIR, exist_ok=True)
sound_files = [
    os.path.basename(path)
    for path in glob.glob(os.path.join(SOUND_DIR, "*" + SOUND_EXT))
]
for filename, message in tts_messages.items():
    if not filename + SOUND_EXT in sound_files:
        print(f"Generating {filename}{SOUND_EXT}")
        tts = gTTS(message[0], lang=message[1])
        wav_f64 = os.path.join(SOUND_DIR, filename + "_f64" + SOUND_EXT)
        tts.save(wav_f64)
        ffmpeg.input(wav_f64).output(
            os.path.join(SOUND_DIR, filename + SOUND_EXT),
            acodec="pcm_s16le",
            loglevel="quiet",
        ).run()
        os.remove(wav_f64)
