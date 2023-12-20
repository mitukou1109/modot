import glob
import os

import ffmpeg
from gtts import gTTS

SOUND_DIR = os.path.join(os.path.dirname(__file__), "resource", "sound")

SOUND_EXT = ".wav"

TTS_MESSAGES: dict[str, tuple[str, str]] = {
    "obstacle": ("障害物があります", "ja"),
    "signal_blue": ("青信号です", "ja"),
    "signal_red": ("赤信号です", "ja"),
}

os.makedirs(SOUND_DIR, exist_ok=True)
sound_files = [
    os.path.basename(path)
    for path in glob.glob(os.path.join(SOUND_DIR, "*" + SOUND_EXT))
]
for filename, message in TTS_MESSAGES.items():
    if not filename + SOUND_EXT in sound_files:
        print(f"Generate {filename}{SOUND_EXT}")
        tts = gTTS(message[0], lang=message[1])
        wav_f64 = os.path.join(SOUND_DIR, filename + "_" + SOUND_EXT)
        tts.save(wav_f64)
        ffmpeg.input(wav_f64).output(
            os.path.join(SOUND_DIR, filename + SOUND_EXT),
            acodec="pcm_s16le",
            loglevel="quiet",
        ).run()
        os.remove(wav_f64)
