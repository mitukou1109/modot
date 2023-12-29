import glob
import os

import geometry_msgs.msg
import rclpy
import rclpy.logging
import sensor_msgs.msg
import simpleaudio
import vision_msgs.msg
from ament_index_python.packages import get_package_share_directory
from modot_lib.stoppable_thread import StoppableThread
from rclpy.node import Node


class SoundNotifier(Node):
    SOUND_DIR = os.path.join(
        get_package_share_directory("modot_notification"), "resource", "sound"
    )

    def __init__(self) -> None:
        super().__init__("sound_notifier")

        self.obstacle_sound = SoundNotifier.create_sound_by_name("obstacle")
        self.yolo_sounds = SoundNotifier.create_sound_set_by_prefix("yolo")
        self.direction_sounds = SoundNotifier.create_sound_set_by_prefix("direction")
        self.face_sounds = SoundNotifier.create_sound_set_by_prefix("face")
        self.exist_sound = SoundNotifier.create_sound_by_name("exist")

        self.obstacle_sound_playback: simpleaudio.PlayObject = None
        self.misc_sound_playback: simpleaudio.PlayObject = None

        self.play_misc_sound_thread: StoppableThread = None
        self.face_identifier_result_image_width: int = None

        self.declare_parameter("face_front_range", 0.4)

        self.obstacle_centroid_sub = self.create_subscription(
            geometry_msgs.msg.PointStamped,
            "obstacle_detector/obstacle_centroid",
            self.obstacle_centroid_callback,
            1,
        )
        self.yolo_detections_sub = self.create_subscription(
            vision_msgs.msg.Detection2DArray,
            "yolo_detector/detections",
            self.yolo_detections_callback,
            1,
        )
        self.face_detections_sub = self.create_subscription(
            vision_msgs.msg.Detection2DArray,
            "face_identifier/detections",
            self.face_detections_callback,
            1,
        )
        self.face_identifier_result_image_sub = self.create_subscription(
            sensor_msgs.msg.Image,
            "face_identifier/result_image",
            self.face_identifier_result_image_callback,
            1,
        )

    def play_yolo_sound(self, ids: list[str]) -> None:
        index = 0
        while True:
            if self.play_misc_sound_thread.stopped():
                self.misc_sound_playback.stop()
                break
            if (
                self.misc_sound_playback is None
                or not self.misc_sound_playback.is_playing()
            ):
                if index >= len(ids):
                    break
                else:
                    self.misc_sound_playback = self.yolo_sounds[ids[index]].play()
                    index += 1

    def play_face_sound(self, direction: str, ids: list[str]) -> None:
        state = 0
        face_index = 0
        while True:
            if self.play_misc_sound_thread.stopped():
                self.misc_sound_playback.stop()
                break
            if (
                self.misc_sound_playback is None
                or not self.misc_sound_playback.is_playing()
            ):
                if state == 0:
                    self.misc_sound_playback = self.direction_sounds[direction].play()
                    state = 1
                elif state == 1:
                    self.misc_sound_playback = self.face_sounds[ids[face_index]].play()
                    state = 2
                elif state == 2:
                    face_index += 1
                    state = 3 if face_index >= len(ids) else 1
                elif state == 3:
                    self.misc_sound_playback = self.exist_sound.play()
                    state = 4
                elif state == 4:
                    break

    def obstacle_centroid_callback(self, msg: geometry_msgs.msg.PointStamped) -> None:
        if (
            self.obstacle_sound_playback is None
            or not self.obstacle_sound_playback.is_playing()
        ):
            if (
                self.play_misc_sound_thread is not None
                and self.play_misc_sound_thread.is_alive()
            ):
                self.play_misc_sound_thread.stop()

            self.obstacle_sound_playback = self.obstacle_sound.play()

    def yolo_detections_callback(self, msg: vision_msgs.msg.Detection2DArray) -> None:
        if not msg.detections:
            return

        if (
            self.obstacle_sound_playback is not None
            and self.obstacle_sound_playback.is_playing()
        ):
            return

        if (
            self.play_misc_sound_thread is None
            or not self.play_misc_sound_thread.is_alive()
        ):
            ids = []
            detection: vision_msgs.msg.Detection2D
            for detection in msg.detections:
                if detection.id in self.yolo_sounds.keys():
                    ids.append(detection.id)
            if ids:
                self.play_misc_sound_thread = StoppableThread(
                    target=self.play_yolo_sound, args=(ids,)
                )
                self.play_misc_sound_thread.start()

    def face_detections_callback(self, msg: vision_msgs.msg.Detection2DArray) -> None:
        if not msg.detections or not self.face_identifier_result_image_width:
            return

        if (
            self.obstacle_sound_playback is not None
            and self.obstacle_sound_playback.is_playing()
        ):
            return

        if (
            self.play_misc_sound_thread is None
            or not self.play_misc_sound_thread.is_alive()
        ):
            front_range = (
                self.get_parameter("face_front_range")
                .get_parameter_value()
                .double_value
            )
            ids = []
            detection: vision_msgs.msg.Detection2D
            for detection in msg.detections:
                normalized_x = (
                    detection.bbox.center.position.x
                    / self.face_identifier_result_image_width
                )
                if ids:
                    direction = "from_left"
                else:
                    if normalized_x < (1 - front_range) / 2:
                        direction = "left"
                    elif normalized_x > (1 + front_range) / 2:
                        direction = "right"
                    else:
                        direction = "front"
                ids.append(detection.id)
            if ids:
                self.play_misc_sound_thread = StoppableThread(
                    target=self.play_face_sound, args=(direction, ids)
                )
                self.play_misc_sound_thread.start()

    def face_identifier_result_image_callback(self, msg: sensor_msgs.msg.Image) -> None:
        self.face_identifier_result_image_width = msg.width
        self.destroy_subscription(self.face_identifier_result_image_sub)

    @staticmethod
    def create_sound_by_name(name: str) -> simpleaudio.WaveObject:
        return simpleaudio.WaveObject.from_wave_file(
            os.path.join(SoundNotifier.SOUND_DIR, name + ".wav")
        )

    @staticmethod
    def create_sound_set_by_prefix(prefix: str) -> dict[str, simpleaudio.WaveObject]:
        sounds: dict[str, simpleaudio.WaveObject] = {}
        for path in glob.glob(os.path.join(SoundNotifier.SOUND_DIR, f"{prefix}_*.wav")):
            sounds[
                os.path.splitext(os.path.basename(path))[0].replace(f"{prefix}_", "")
            ] = simpleaudio.WaveObject.from_wave_file(path)
        return sounds


def main(args: list[str] = None):
    rclpy.init(args=args)
    sound_notifier = SoundNotifier()
    rclpy.spin(sound_notifier)
    sound_notifier.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
