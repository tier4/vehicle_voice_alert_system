# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from os import path
from simpleaudio import WaveObject
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from tier4_hmi_msgs.srv import Announce

# The higher the value, the higher the priority
PRIORITY_DICT = {
    "emergency": 4,
    "departure": 4,
    "stop": 4,
    "restart_engage": 4,
    "obstacle_stop": 3,
    "in_emergency": 3,
    "temporary_stop": 2,
    "turning_left": 1,
    "turning_right": 1,
}


class AnnounceControllerProperty:
    def __init__(self, node, autoware_state_interface=None):
        super(AnnounceControllerProperty, self).__init__()
        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)
        autoware_state_interface.set_control_mode_callback(self.sub_control_mode)
        autoware_state_interface.set_turn_signal_callback(self.check_turn_signal)
        autoware_state_interface.set_stop_reason_callback(self.sub_stop_reason)
        autoware_state_interface.set_velocity_callback(self.sub_velocity)

        self._node = node
        self.is_auto_mode = False
        self._is_auto_running = False
        self._in_driving_state = False
        self._in_emergency_state = False
        self._velocity = None
        self._autoware_state = ""
        self._current_announce = ""
        self._pending_announce_list = []
        self._emergency_trigger_time = self._node.get_clock().now()
        self._wav_object = None
        self._music_object = None
        self._in_stop_status = False
        self._signal_announce_time = self._node.get_clock().now()
        self._stop_reason_announce_time = self._node.get_clock().now()
        self._start_request_announce_time = self._node.get_clock().now()
        self._bgm_announce_time = self._node.get_clock().now()

        self._node.declare_parameter("manual_driving_bgm", False)
        self._manual_driving_bgm = (
            self._node.get_parameter("manual_driving_bgm").get_parameter_value().bool_value
        )
        self._node.declare_parameter("skip_default_voice", False)
        self._skip_default_voice = (
            self._node.get_parameter("skip_default_voice").get_parameter_value().bool_value
        )

        self._node.declare_parameter("driving_velocity_threshold", 0.2)
        self._driving_velocity_threshold = (
            self._node.get_parameter("driving_velocity_threshold").get_parameter_value().double_value
        )

        self._node.declare_parameter("mute_timeout.restart_engage", 0.0)
        self._node.declare_parameter("mute_timeout.stop_reason", 0.0)
        self._node.declare_parameter("mute_timeout.turn_signal", 0.0)
        self._node.declare_parameter("mute_timeout.in_emergency", 0.0)
        self._node.declare_parameter("mute_timeout.driving_bgm", 0.0)
        mute_timeout_prefix = self._node.get_parameters_by_prefix("mute_timeout")

        self._mute_timeout = {}
        for key in mute_timeout_prefix.keys():
            self._mute_timeout[key] = mute_timeout_prefix[key].get_parameter_value().double_value

        self._node.declare_parameter("primary_voice_folder_path", "")
        self._primary_voice_folder_path = (
            self._node.get_parameter("primary_voice_folder_path").get_parameter_value().string_value
        )

        self._package_path = (
            get_package_share_directory("vehicle_voice_alert_system") + "/resource/sound"
        )

        self._running_bgm_file = ""
        if path.exists(self._primary_voice_folder_path + "/running_music.wav"):
            self._running_bgm_file = self._primary_voice_folder_path + "/running_music.wav"
        elif not self._skip_default_voice:
            self._running_bgm_file = self._package_path + "/running_music.wav"

        self._check_playing_timer = self._node.create_timer(1, self.check_playing_callback)
        self._srv = self._node.create_service(
            Announce, "/api/vehicle_voice/set/announce", self.announce_service
        )

    def announce_service(self, request, response):
        try:
            announce_type = request.kind
            if announce_type == 1:
                self.send_announce("departure")
            elif announce_type == 2 and self._is_auto_running:
                if self._node.get_clock().now() - self._start_request_announce_time > Duration(
                    seconds=self._mute_timeout["restart_engage"]
                ):
                    self._start_request_announce_time = self._node.get_clock().now()
                    self.send_announce("restart_engage")
                else:
                    self._node.get_logger().warning("skip announce restart engage")
                # To reset the stop reason announce, so that it can announce if vehicle reengage
                self._stop_reason_announce_time = self._node.get_clock().now()-Duration(seconds=self._mute_timeout["restart_engage"])
            if self._wav_object:
                if self._wav_object.is_playing():
                    self._wav_object.wait_done()
        except Exception as e:
            self._node.get_logger().error("not able to play the announce, ERROR: {}".format(str(e)))
        return response

    def process_running_music(self):
        try:
            if not self._running_bgm_file:
                return

            if self._node.get_clock().now() - self._bgm_announce_time < Duration(seconds=self._mute_timeout["driving_bgm"]):
                return

            if self._in_driving_state and not self._in_emergency_state:
                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self._music_object = sound.play()
            elif self._manual_driving_bgm and not self.is_auto_mode and (self._velocity > self._driving_velocity_threshold or self._velocity < - self._driving_velocity_threshold):
                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self._music_object = sound.play()
            else:
                if self._music_object and self._music_object.is_playing():
                    self._music_object.stop()
            self._bgm_announce_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().error("not able to check the pending playing list: " + str(e))

    def sub_control_mode(self, control_mode):
        self.is_auto_mode = control_mode == 1

    def check_playing_callback(self):
        try:
            self.process_running_music()
            if not self._wav_object:
                self._current_announce = ""
                return

            if not self._wav_object.is_playing():
                self._current_announce = ""
        except Exception as e:
            self._node.get_logger().error("not able to check the current playing: " + str(e))

    def play_sound(self, message):
        if path.exists("{}/{}.wav".format(self._primary_voice_folder_path, message)):
            sound = WaveObject.from_wave_file("{}/{}.wav".format(self._primary_voice_folder_path, message))
            self._wav_object = sound.play()
        elif not self._skip_default_voice:
            sound = WaveObject.from_wave_file("{}/{}.wav".format(self._package_path, message))
            self._wav_object = sound.play()
        else:
            self._node.get_logger().info("Didn't found the voice in the primary voice folder, and skip default voice is enabled")

    def send_announce(self, message):
        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if priority > previous_priority:
            if self._wav_object:
                self._wav_object.stop()
            self.play_sound(message)
        self._current_announce = message

    def sub_velocity(self, velocity):
        self._velocity = velocity
        if velocity > 0 and self.is_auto_mode and self._in_driving_state:
            self._is_auto_running = True

    def sub_autoware_state(self, autoware_state):
        if autoware_state == "Driving" and not self._in_driving_state:
            self._in_driving_state = True
        elif (
            autoware_state in ["WaitingForRoute", "WaitingForEngage", "ArrivedGoal", "Planning"]
            and self._in_driving_state
        ):
            if self.is_auto_mode:
                # Skip announce if is in manual driving
                self.send_announce("stop")
            self._is_auto_running = False
            self._in_driving_state = False
        self._autoware_state = autoware_state

    def sub_emergency(self, emergency_stopped):
        if emergency_stopped and not self._in_emergency_state:
            self.send_announce("emergency")
            self._in_emergency_state = True
            self._emergency_trigger_time = self._node.get_clock().now()
        elif not emergency_stopped and self._in_emergency_state:
            self._in_emergency_state = False
        elif emergency_stopped and self._in_emergency_state and not self._in_stop_status:
            if self._node.get_clock().now() - self._emergency_trigger_time > Duration(seconds=self._mute_timeout["in_emergency"]):
                self.send_announce("in_emergency")
                self._emergency_trigger_time = self._node.get_clock().now()

    def check_turn_signal(self, turn_signal):
        if self._node.get_clock().now() - self._signal_announce_time < Duration(seconds=self._mute_timeout["turn_signal"]):
            return
        elif self._in_emergency_state or self._in_stop_status:
            return

        if turn_signal == 1:
            self.send_announce("turning_left")
        if turn_signal == 2:
            self.send_announce("turning_right")

        self._signal_announce_time = self._node.get_clock().now()

    # 停止する予定を取得
    def sub_stop_reason(self, stop_reason):
        if self._autoware_state != "Driving":
            self._node.get_logger().warning("The vehicle is not in driving state, do not announce")
            return

        stop_reasons = stop_reason.stop_reasons
        shortest_stop_reason = ""
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
        if shortest_stop_reason != "" and shortest_distance > -1 and shortest_distance < 2:
            if self._node.get_clock().now() - self._stop_reason_announce_time < Duration(
                seconds=self._mute_timeout["stop_reason"]
            ):
                return

            if (
                shortest_stop_reason
                in [
                    "ObstacleStop",
                    "DetectionArea",
                    "SurroundObstacleCheck",
                    "BlindSpot",
                    "BlockedByObstacles",
                ]
                and self._velocity == 0
            ):
                self._in_stop_status = True
                self.send_announce("obstacle_stop")
                self._stop_reason_announce_time = self._node.get_clock().now()
            elif shortest_stop_reason in [
                "StopLine",
                "Walkway",
                "Crosswalk",
                "MergeFromPrivateRoad",
            ]:
                self.send_announce("temporary_stop")
                self._in_stop_status = True
                self._stop_reason_announce_time = self._node.get_clock().now()
            else:
                self._in_stop_status = False
        else:
            self._in_stop_status = False
