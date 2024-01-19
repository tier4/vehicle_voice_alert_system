# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.duration import Duration
from dataclasses import dataclass
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
    VelocityFactorArray,
)
from tier4_api_msgs.msg import AwapiAutowareStatus, AwapiVehicleStatus
from tier4_debug_msgs.msg import Float64Stamped


@dataclass
class AutowareInformation:
    stop_reasons: list
    velocity_factors: list
    autoware_control: bool = False
    operation_mode: int = 0
    mrm_behavior: int = 0
    route_state: int = 0
    turn_signal: int = 0
    velocity: float = 0.0
    motion_state: int = 0
    localization_init_state: int = 0
    goal_distance: float = 1000.0


class AutowareInterface:
    def __init__(self, node):
        self._node = node
        self.information = AutowareInformation([], [])

        sub_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.SYSTEM_DEFAULT,
        )
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.sub_operation_mode_callback,
            api_qos,
        )
        node.create_subscription(
            RouteState,
            "/api/routing/state",
            self.sub_routing_state_callback,
            api_qos,
        )
        node.create_subscription(
            MrmState,
            "/supervisor/fail_safe/overall/mrm_state",
            self.sub_mrm_callback,
            sub_qos,
        )
        node.create_subscription(
            AwapiVehicleStatus,
            "/awapi/vehicle/get/status",
            self.sub_vehicle_state_callback,
            sub_qos,
        )
        node.create_subscription(
            AwapiAutowareStatus,
            "/awapi/autoware/get/status",
            self.sub_autoware_state_callback,
            sub_qos,
        )
        node.create_subscription(
            VelocityFactorArray,
            "/api/planning/velocity_factors",
            self.sub_velocity_factor_callback,
            sub_qos,
        )
        node.create_subscription(
            MotionState, "/api/motion/state", self.sub_motion_state_callback, api_qos
        )
        node.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self.sub_localization_initialization_state_callback,
            api_qos,
        )
        node.create_subscription(
            Float64Stamped,
            "/autoware_api/utils/path_distance_calculator/distance",
            self.sub_path_distance_callback,
            sub_qos,
        )
        self._autoware_connection_time = self._node.get_clock().now()
        self._node.create_timer(2, self.reset_timer)

    def reset_timer(self):
        if self._node.get_clock().now() - self._autoware_connection_time > Duration(seconds=10):
            self.information = AutowareInformation([], [])
            self._node.get_logger().error("Autoware disconnected", throttle_duration_sec=10)

    def sub_operation_mode_callback(self, msg):
        try:
            self.information.autoware_control = msg.is_autoware_control_enabled
            self.information.operation_mode = msg.mode
        except Exception as e:
            self._node.get_logger().error("Unable to get the operation mode, ERROR: " + str(e))

    def sub_routing_state_callback(self, msg):
        try:
            self.information.route_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the routing state, ERROR: " + str(e))

    def sub_mrm_callback(self, msg):
        try:
            self.information.mrm_behavior = msg.behavior
        except Exception as e:
            self._node.get_logger().error("Unable to get the mrm behavior, ERROR: " + str(e))

    def sub_vehicle_state_callback(self, msg):
        try:
            self.information.turn_signal = msg.turn_signal
            self.information.velocity = msg.velocity
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_autoware_state_callback(self, msg):
        try:
            self.information.stop_reasons = msg.stop_reason.stop_reasons
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_velocity_factor_callback(self, msg):
        try:
            self.information.velocity_factors = msg.factors
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_motion_state_callback(self, msg):
        try:
            self.information.motion_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the motion state, ERROR: " + str(e))

    def sub_localization_initialization_state_callback(self, msg):
        try:
            self.information.localization_init_state = msg.state
        except Exception as e:
            self._node.get_logger().error(
                "Unable to get the localization init state, ERROR: " + str(e)
            )

    def sub_path_distance_callback(self, msg):
        try:
            self._autoware_connection_time = self._node.get_clock().now()
            self.information.goal_distance = msg.data
        except Exception as e:
            self._node.get_logger().error("Unable to get the goal distance, ERROR: " + str(e))
