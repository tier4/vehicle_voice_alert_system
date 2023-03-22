# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from os import path
from simpleaudio import WaveObject
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration

from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
)

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
    def __init__(
        self,
        node,
        autoware_state_interface,
        ros_service_interface,
        parameter_interface,
        autoware_interface,
    ):
        super(AnnounceControllerProperty, self).__init__()
        self._node = node
        self._ros_service_interface = ros_service_interface
        self._parameter = parameter_interface.parameter
        self._mute_parameter = parameter_interface.mute_parameter
        self._autoware = autoware_interface.information
        self._in_emergency_state = False
        self._emergency_trigger_time = self._node.get_clock().now()
        self._prev_motion_state = 0
        self._accept_start_time = self._node.get_clock().now()
        self._current_announce = ""
        self._pending_announce_list = []
        self._wav_object = None
        self._music_object = None
        self._in_stop_status = False
        self._in_driving_state = False
        self._signal_announce_time = self._node.get_clock().now()
        self._stop_reason_announce_time = self._node.get_clock().now()
        self._bgm_announce_time = self._node.get_clock().now()
        self._restart_time = self._node.get_clock().now()

        self._package_path = (
            get_package_share_directory("vehicle_voice_alert_system") + "/resource/sound"
        )

        self._running_bgm_file = ""
        if path.exists(self._parameter.primary_voice_folder_path + "/running_music.wav"):
            self._running_bgm_file = (
                self._parameter.primary_voice_folder_path + "/running_music.wav"
            )
        elif not self._parameter.skip_default_voice:
            self._running_bgm_file = self._package_path + "/running_music.wav"

        self._node.create_timer(0.5, self.check_playing_callback)
        self._node.create_timer(0.5, self.turn_signal_callback)
        self._node.create_timer(0.5, self.emergency_checker_callback)
        self._node.create_timer(0.5, self.stop_reason_checker_callback)
        self._node.create_timer(0.2, self.announce_engage_when_starting)

    def check_timeout(self, trigger_time, duration):
        return self._node.get_clock().now() - trigger_time > Duration(seconds=duration)

    def check_in_autonomous(self):
        return self._autoware.operation_mode == OperationModeState.AUTONOMOUS

    def process_running_music(self):
        try:
            if not self._running_bgm_file:
                return

            if self._node.get_clock().now() - self._bgm_announce_time < Duration(
                seconds=self._mute_parameter.driving_bgm
            ):
                return

            if (
                self._parameter.mute_overlap_bgm
                and self._wav_object
                and self._wav_object.is_playing()
            ):
                self._bgm_announce_time = self._node.get_clock().now()
                return

            if self.check_in_autonomous() and not self._in_emergency_state:
                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self._music_object = sound.play()
            elif (
                self._parameter.manual_driving_bgm
                and not self._autoware.autoware_control
                and not self.in_range(
                    self._autoware.velocity, self._parameter.driving_velocity_threshold
                )
            ):
                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self._music_object = sound.play()
            else:
                if self._music_object and self._music_object.is_playing():
                    self._music_object.stop()

                if self._autoware.autoware_control and self._in_driving_state:
                    # Skip announce if is in manual driving
                    self.send_announce("stop")

            self._in_driving_state = self.check_in_autonomous()
            self._bgm_announce_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().error("not able to check the pending playing list: " + str(e))

    def in_range(self, input_value, range_value):
        return -range_value <= input_value <= range_value

    def announce_engage_when_starting(self):
        try:
            if (
                self._autoware.localization_init_state
                == LocalizationInitializationState.UNINITIALIZED
            ):
                self._prev_motion_state = 0
                return

            if (
                self._autoware.motion_state in [MotionState.STARTING, MotionState.MOVING]
                and self._prev_motion_state == 1
            ):
                self.send_announce("departure")

                if self._autoware.motion_state == MotionState.STARTING:
                    self._service_interface.accept_start()

            # Check to see if it has not stopped waiting for start acceptance
            if self._autoware.motion_state != MotionState.STARTING:
                self._accept_start_time = self._node.get_clock().now()

            # Send again when stopped in starting state for a certain period of time
            if self._autoware.motion_state == MotionState.STARTING and self.check_timeout(
                self._accept_start_time, self._parameter.accept_start
            ):
                self._service_interface.accept_start()

            self._prev_motion_state = self._autoware.motion_state
        except Exception as e:
            self._node.get_logger().error("not able to play the announce, ERROR: {}".format(str(e)))

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
        if (
            self._parameter.mute_overlap_bgm
            and self._music_object
            and self._music_object.is_playing()
        ):
            self._music_object.stop()

        if path.exists("{}/{}.wav".format(self._parameter.primary_voice_folder_path, message)):
            sound = WaveObject.from_wave_file(
                "{}/{}.wav".format(self._parameter.primary_voice_folder_path, message)
            )
            self._wav_object = sound.play()
        elif not self._parameter.skip_default_voice:
            sound = WaveObject.from_wave_file("{}/{}.wav".format(self._package_path, message))
            self._wav_object = sound.play()
        else:
            self._node.get_logger().info(
                "Didn't found the voice in the primary voice folder, and skip default voice is enabled"
            )

    def send_announce(self, message):
        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if priority > previous_priority:
            if self._wav_object:
                self._wav_object.stop()
            self.play_sound(message)
        self._current_announce = message

    def emergency_checker_callback(self):
        in_emergency = self._autoware.mrm_behavior == MrmState.EMERGENCY_STOP

        if in_emergency and not self._in_emergency_state:
            self.send_announce("emergency")
        elif in_emergency and self._in_emergency_state:
            if self.check_timeout(self._emergency_trigger_time, self._mute_parameter.in_emergency):
                self.send_announce("in_emergency")
                self._emergency_trigger_time = self._node.get_clock().now()

        self._in_emergency_state = in_emergency

    def turn_signal_callback(self):
        if not self.check_timeout(self._signal_announce_time, self._mute_parameter.turn_signal):
            return
        elif self._in_emergency_state or self._in_stop_status:
            return

        if self._autoware.turn_signal == 1:
            self.send_announce("turning_left")
        if self._autoware.turn_signal == 2:
            self.send_announce("turning_right")

        self._signal_announce_time = self._node.get_clock().now()

    # 停止する予定を取得
    def stop_reason_checker_callback(self):
        if not self.check_in_autonomous():
            self._node.get_logger().warning(
                "The vehicle is not in driving state, do not announce", throttle_duration_sec=10
            )
            return

        stop_reasons = self._autoware.stop_reasons
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
            if not self.check_timeout(
                self._stop_reason_announce_time, self._mute_parameter.stop_reason
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
                and self._autoware.velocity == 0
            ):
                self._in_stop_status = True
                self.send_announce("obstacle_stop")
                self._stop_reason_announce_time = self._node.get_clock().now()
            elif shortest_stop_reason in [
                "Intersection",
                "MergeFromPrivateRoad",
                "Crosswalk",
                "Walkway",
                "StopLine",
                "NoStoppingArea",
                "TrafficLight",
                "BlindSpot",
            ]:
                self.send_announce("temporary_stop")
                self._in_stop_status = True
                self._stop_reason_announce_time = self._node.get_clock().now()
            else:
                self._in_stop_status = False
        else:
            self._in_stop_status = False
