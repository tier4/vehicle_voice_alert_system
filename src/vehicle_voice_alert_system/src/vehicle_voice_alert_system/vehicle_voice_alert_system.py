# This Python file uses the following encoding: utf-8

import rclpy
from rclpy.node import Node

from vehicle_voice_alert_system.announce_controller import AnnounceControllerProperty
from vehicle_voice_alert_system.autoware_state_interface import AutowareStateInterface
from ament_index_python.packages import get_package_share_directory

def main(args=None):
    package_path = get_package_share_directory('vehicle_voice_alert_system')

    rclpy.init(args=args)
    node = Node("vehicle_voice_alert_system")

    autoware_state_interface = AutowareStateInterface(node)
    announceController = AnnounceControllerProperty(node, autoware_state_interface)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


