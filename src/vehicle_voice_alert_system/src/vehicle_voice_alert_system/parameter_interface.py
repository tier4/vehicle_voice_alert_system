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
    announce_arriving_distance: float = 10.0

@dataclass
class AnnounceIntervalParameter:
    stop_reason: float = 0.0
    turn_signal: float = 0.0
    in_emergency: float = 0.0
    driving_bgm: float = 0.0
    accept_start: float = 0.0

@dataclass
class AnnounceSettings:
    emergency: bool = False
    departure: bool = False
    stop: bool = False
    restart_engage: bool = False
    going_to_arrive: bool = False
    obstacle_stop: bool = False
    in_emergency: bool = False
    temporary_stop: bool = False
    turning_left: bool = False
    turning_right: bool = False
    bgm: bool = False

class ParameterInterface:
    def __init__(self, node):
        self.parameter = GeneralParameter()
        self.announce_interval_parameter = AnnounceIntervalParameter()
        self.announce_settings = AnnounceSettings()

        node.declare_parameter("manual_driving_bgm", False)
        node.declare_parameter("skip_default_voice", False)
        node.declare_parameter("mute_overlap_bgm", False)
        node.declare_parameter("driving_velocity_threshold", 0.2)
        node.declare_parameter("primary_voice_folder_path", "")
        node.declare_parameter("announce_arriving_distance", 10.0)

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
        self.parameter.announce_arriving_distance = (
            node.get_parameter("announce_arriving_distance").get_parameter_value().double_value
        )

        node.declare_parameter("announce_interval.stop_reason", 0.0)
        node.declare_parameter("announce_interval.turn_signal", 0.0)
        node.declare_parameter("announce_interval.in_emergency", 0.0)
        node.declare_parameter("announce_interval.driving_bgm", 0.0)
        node.declare_parameter("announce_interval.accept_start", 0.0)
        announce_interval_prefix = node.get_parameters_by_prefix("announce_interval")

        for key in announce_interval_prefix.keys():
            setattr(
                self.announce_interval_parameter,
                key,
                announce_interval_prefix[key].get_parameter_value().double_value,
            )

        node.declare_parameter("announce.emergency", False)
        node.declare_parameter("announce.departure", False)
        node.declare_parameter("announce.stop", False)
        node.declare_parameter("announce.restart_engage", False)
        node.declare_parameter("announce.going_to_arrive", False)
        node.declare_parameter("announce.obstacle_stop", False)
        node.declare_parameter("announce.in_emergency", False)
        node.declare_parameter("announce.temporary_stop", False)
        node.declare_parameter("announce.turning_left", False)
        node.declare_parameter("announce.turning_right", False)
        node.declare_parameter("announce.bgm", False)

        announce_prefix = node.get_parameters_by_prefix("announce")

        for key in announce_prefix.keys():
            setattr(
                self.announce_settings,
                key,
                announce_prefix[key].get_parameter_value().bool_value,
            )
