# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from simpleaudio import WaveObject
from ament_index_python.packages import get_package_share_directory
from autoware_external_api_msgs.srv import Engage
from rclpy.duration import Duration

# The higher the value, the higher the priority
PRIORITY_DICT = {
    "departure" : 4,
    "stop" : 4,
    "obstacle_detect": 3,
    "temporary_stop" : 2,
    "turning_left" : 1,
    "turning_right" : 1,
}

class AnnounceControllerProperty():
    def __init__(self, node, autoware_state_interface=None):
        super(AnnounceControllerProperty, self).__init__()
        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        # autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)
        autoware_state_interface.set_turn_signal_callback(self.check_turn_signal)
        autoware_state_interface.set_stop_reason_callback(self.sub_stop_reason)

        self._node = node
        self._in_driving_state = False
        self._in_emergency_state = False
        self._autoware_state = ""
        self._current_announce = ""
        self._pending_announce_list = []
        self._emergency_trigger_time = 0
        self._wav_object = None
        self._signal_announce_time = self._node.get_clock().now()
        self._stop_reason_announce_time = self._node.get_clock().now()
        self._package_path = get_package_share_directory('vehicle_voice_alert_system') + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(
            1,
            self.check_playing_callback)

    def process_pending_announce(self):
        try:
            for play_sound in self._pending_announce_list:
                self._pending_announce_list.remove(play_sound)
                current_time = self._node.get_clock().now().to_msg().sec
                if current_time - play_sound["requested_time"] <= 10:
                    self.send_announce(play_sound["message"])
                    break
        except Exception as e:
            self._node.get_logger().error("not able to check the pending playing list: " + str(e))

    def check_playing_callback(self):
        try:
            if not self._current_announce or not self._wav_object:
                self._current_announce = ""
                return

            if not self._wav_object.is_playing():
                self._current_announce = ""
                self.process_pending_announce()
        except Exception as e:
            self._node.get_logger().error("not able to check the current playing: " + str(e))

    def play_sound(self, message):
        sound = WaveObject.from_wave_file(self._package_path + message + ".wav")
        self._wav_object = sound.play()

    def send_announce(self, message):
        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if priority == 4:
            if self._wav_object:
                self._wav_object.stop()
            self.play_sound(message)
        elif priority > previous_priority:
                if self._wav_object:
                    self._wav_object.stop()
                self.play_sound(message)
        self._current_announce = message

    def sub_autoware_state(self, autoware_state):
        if autoware_state == "Driving" and not self._in_driving_state:
            self.send_announce("departure")
            self._in_driving_state = True
        elif autoware_state == "ArrivedGoal" and self._in_driving_state:
            self.send_announce("stop")
            self._in_driving_state = False
        self._autoware_state = autoware_state

    def sub_emergency(self, emergency_stopped):
        if emergency_stopped and not self._in_emergency_state:
            self.send_announce("temporary_stop")
            self._in_emergency_state = True
        elif not emergency_stopped and self._in_emergency_state:
            self.send_announce("departure")
            self._in_emergency_state = False

    def check_turn_signal(self, turn_signal):
        if self._node.get_clock().now() - self._signal_announce_time < Duration(seconds=5):
            return

        if turn_signal == 1:
            self.send_announce("turning_left")
        if turn_signal == 2:
            self.send_announce("turning_right")

        self._signal_announce_time = self._node.get_clock().now()
        

    # 停止する予定を取得
    def sub_stop_reason(self, stop_reason):
        stop_reasons = stop_reason.stop_reasons
        shortest_stop_reason = "test"
        shortest_distance = -1
        for stop_reason in stop_reasons:
            dist_to_stop_pose = -1
            for stop_factor in stop_reason.stop_factors:
                if dist_to_stop_pose > stop_factor.dist_to_stop_pose or dist_to_stop_pose == -1:
                    dist_to_stop_pose = stop_factor.dist_to_stop_pose
            if shortest_distance == -1 or shortest_distance > dist_to_stop_pose:
                shortest_distance = dist_to_stop_pose
                shortest_stop_reason = stop_reason.reason

        # 音声の通知
        if shortest_stop_reason != "test" and shortest_distance > -1 and shortest_distance < 2:
            if self._node.get_clock().now() - self._stop_reason_announce_time < Duration(seconds=5):
                return

            if shortest_stop_reason in ["ObstacleStop", "DetectionArea", "SurroundObstacleCheck", "BlindSpot", "BlockedByObstacles"]:
                self.send_announce("obstacle_detect")
            elif shortest_stop_reason == ["StopLine", "Walkway", "Crosswalk", "MergeFromPrivateRoad"]:
                self.send_announce("temporary_stop")

            self._stop_reason_announce_time = self._node.get_clock().now()