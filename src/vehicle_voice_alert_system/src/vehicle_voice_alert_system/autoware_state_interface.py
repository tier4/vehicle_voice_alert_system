# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.duration import Duration
from tier4_api_msgs.msg import AwapiAutowareStatus, AwapiVehicleStatus
from autoware_adapi_v1_msgs.msg import MotionState, LocalizationInitializationState
from autoware_auto_system_msgs.msg import HazardStatusStamped

class AutowareStateInterface:
    def __init__(self, node):
        self.autoware_state_callback_list = []
        self.control_mode_callback_list = []
        self.emergency_stopped_callback_list = []
        self.stop_reason_callback_list = []
        self.turn_signal_callback_list = []
        self.velocity_callback_list = []
        self.motion_state_callback_list = []
        self.localization_initialization_state_callback_list = []

        self._node = node

        self._node.declare_parameter("ignore_emergency_stoppped", False)
        self._ignore_emergency_stoppped = (
            self._node.get_parameter("ignore_emergency_stoppped").get_parameter_value().bool_value
        )

        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub_autoware_state = node.create_subscription(
            AwapiAutowareStatus, "/awapi/autoware/get/status", self.autoware_state_callback, 10
        )
        self._sub_vehicle_state = node.create_subscription(
            AwapiVehicleStatus, "/awapi/vehicle/get/status", self.vehicle_state_callback, 10
        )
        self._sub_motion_state = node.create_subscription(
            MotionState, "/api/motion/state", self.motion_state_callback, api_qos
        )
        self._sub_hazard_status = node.create_subscription(
            HazardStatusStamped, "/system/emergency/hazard_status", self.sub_hazard_status_callback, 10
        )
        self._sub_localiztion_initializtion_state = node.create_subscription(
            LocalizationInitializationState, "/api/localization/initialization_state", self.sub_localization_initialization_state_callback, 10
        )
        self._autoware_status_time = self._node.get_clock().now()
        self._vehicle_status_time = self._node.get_clock().now()
        self._topic_checker = self._node.create_timer(1, self.topic_checker_callback)

    def topic_checker_callback(self):
        if self._node.get_clock().now() - self._autoware_status_time > Duration(seconds=5):
            for callback in self.autoware_state_callback_list:
                callback("")

            for callback in self.control_mode_callback_list:
                callback(0)

            for callback in self.emergency_stopped_callback_list:
                callback(False)

        if self._node.get_clock().now() - self._vehicle_status_time > Duration(seconds=5):
            for callback in self.velocity_callback_list:
                callback(0)

    # set callback
    def set_autoware_state_callback(self, callback):
        self.autoware_state_callback_list.append(callback)

    def set_stop_reason_callback(self, callback):
        self.stop_reason_callback_list.append(callback)

    def set_control_mode_callback(self, callback):
        self.control_mode_callback_list.append(callback)

    def set_emergency_stopped_callback(self, callback):
        self.emergency_stopped_callback_list.append(callback)

    def set_turn_signal_callback(self, callback):
        self.turn_signal_callback_list.append(callback)

    def set_velocity_callback(self, callback):
        self.velocity_callback_list.append(callback)

    def set_motion_state_callback(self, callback):
        self.motion_state_callback_list.append(callback)

    def set_localization_initialization_state_callback(self, callback):
        self.localization_initialization_state_callback_list.append(callback)

    # ros subscriber
    # autoware stateをsubしたときの処理
    def autoware_state_callback(self, topic):
        try:
            self._autoware_status_time = self._node.get_clock().now()
            autoware_state = topic.autoware_state
            control_mode = topic.control_mode
            stop_reason = topic.stop_reason

            if self._ignore_emergency_stoppped:
                emergency_stopped = False
            else:
                emergency_stopped = topic.emergency_stopped

            for callback in self.autoware_state_callback_list:
                callback(autoware_state)

            for callback in self.control_mode_callback_list:
                callback(control_mode)

            for callback in self.stop_reason_callback_list:
                callback(stop_reason)
        except Exception as e:
            self._node.get_logger().error("Unable to get the autoware state, ERROR: " + str(e))

    # vehicle stateをsubしたときの処理
    def vehicle_state_callback(self, topic):
        try:
            self._vehicle_status_time = self._node.get_clock().now()
            turn_signal = topic.turn_signal
            velocity = topic.velocity
            steering = topic.steering

            for callback in self.turn_signal_callback_list:
                callback(turn_signal)
            for callback in self.velocity_callback_list:
                callback(velocity)
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    # 発進時のmotion stateをsubしたときの処理
    def motion_state_callback(self, topic):
        try:
            state = topic.state

            for callback in self.motion_state_callback_list:
                callback(state)
        except Exception as e:
            self._node.get_logger().error("Unable to get the motion state, ERROR: " + str(e))

    def sub_hazard_status_callback(self, topic):
        try:
            emergency_stopped = topic.status.emergency
            for callback in self.emergency_stopped_callback_list:
                callback(emergency_stopped)
        except Exception as e:
            self._node.get_logger().error("Unable to get the hazard_status, ERROR: " + str(e))

    def sub_localization_initialization_state_callback(self, topic):
        try:
            initialization_state = topic.state
            for callback in self.localization_initialization_state_callback_list:
                callback(initialization_state)
        except Exception as e:
            self._node.get_logger().error("Unable to get the hazard_status, ERROR: " + str(e))
