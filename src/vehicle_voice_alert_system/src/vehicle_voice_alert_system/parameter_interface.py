# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from dataclasses import dataclass


@dataclass
class GeneralParameter:
    manual_driving_bgm: bool = False
    skip_default_voice: bool = False
    mute_overlap_bgm: bool = False
    driving_velocity_threshold: float = 0.2
    primary_voice_folder_path: str = ""
    accept_start: float = 0.0


@dataclass
class MuteParameter:
    stop_reason: float = 0.0
    turn_signal: float = 0.0
    in_emergency: float = 0.0
    driving_bgm: float = 0.0


class ParameterInterface:
    def __init__(self, node):
        self.parameter = GeneralParameter()
        self.mute_parameter = MuteParameter()

        node.declare_parameter("manual_driving_bgm", False)
        node.declare_parameter("skip_default_voice", False)
        node.declare_parameter("mute_overlap_bgm", False)
        node.declare_parameter("driving_velocity_threshold", 0.2)
        node.declare_parameter("primary_voice_folder_path", "")
        node.declare_parameter("accept_start", 0.0)

        self.parameter.manual_driving_bgm = (
            node.get_parameter("manual_driving_bgm").get_parameter_value().bool_value
        )
        self.parameter.skip_default_voice = (
            node.get_parameter("skip_default_voice").get_parameter_value().bool_value
        )
        self.parameter.mute_overlap_bgm = (
            node.get_parameter("mute_overlap_bgm").get_parameter_value().bool_value
        )
        self.parameter.driving_velocity_threshold = (
            node.get_parameter("driving_velocity_threshold").get_parameter_value().double_value
        )
        self.parameter.primary_voice_folder_path = (
            node.get_parameter("primary_voice_folder_path").get_parameter_value().string_value
        )
        self.parameter.accept_start = (
            node.get_parameter("accept_start").get_parameter_value().double_value
        )

        node.declare_parameter("mute_timeout.stop_reason", 0.0)
        node.declare_parameter("mute_timeout.turn_signal", 0.0)
        node.declare_parameter("mute_timeout.in_emergency", 0.0)
        node.declare_parameter("mute_timeout.driving_bgm", 0.0)
        mute_timeout_prefix = node.get_parameters_by_prefix("mute_timeout")

        for key in mute_timeout_prefix.keys():
            setattr(
                self.mute_parameter,
                key,
                mute_timeout_prefix[key].get_parameter_value().double_value,
            )
