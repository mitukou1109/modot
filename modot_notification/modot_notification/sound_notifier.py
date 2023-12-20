import os

import geometry_msgs.msg as geometry_msgs
import rclpy
import rclpy.logging
import simpleaudio
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class SoundNotifier(Node):
    def __init__(self):
        super().__init__("sound_notifier")
        self.obstacle_centroid_sub = self.create_subscription(
            geometry_msgs.PointStamped,
            "obstacle_detector/obstacle_centroid",
            self.obstacle_centroid_callback,
            1,
        )
        self.sound_dir = os.path.join(
            get_package_share_directory("modot_notification"), "resource", "sound"
        )
        self.obstacle_sound = simpleaudio.WaveObject.from_wave_file(
            os.path.join(self.sound_dir, "obstacle.wav")
        )
        self.obstacle_sound_playback: simpleaudio.PlayObject = None

    def obstacle_centroid_callback(self, msg: geometry_msgs.PointStamped):
        if (
            self.obstacle_sound_playback is None
            or not self.obstacle_sound_playback.is_playing()
        ):
            self.obstacle_sound_playback = self.obstacle_sound.play()


def main(args: list[str] = None):
    rclpy.init(args=args)
    sound_notifier = SoundNotifier()
    rclpy.spin(sound_notifier)
    sound_notifier.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
