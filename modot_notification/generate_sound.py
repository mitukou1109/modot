import glob
import os

import ffmpeg
from ament_index_python.packages import get_package_share_directory
from gtts import gTTS

SOUND_DIR = os.path.join(
    get_package_share_directory("modot_notification"), "resource", "sound"
)

SOUND_EXT = ".wav"


def generate_sound(tts_messages: dict[str, tuple[str, str]]):
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


if __name__ == "__main__":
    generate_sound(
        {
            "obstacle": ("障害物があります", "ja"),
            "yolo_crosswalk": ("横断歩道があります", "ja"),
            "yolo_stairs": ("階段があります", "ja"),
            "yolo_signal_blue": ("青信号です", "ja"),
            "yolo_signal_red": ("赤信号です", "ja"),
            "direction_left": ("左に", "ja"),
            "direction_right": ("右に", "ja"),
            "direction_front": ("前に", "ja"),
            "direction_from_left": ("左から", "ja"),
            "exist": ("が、います", "ja"),
        }
    )
