#!/bin/bash
# shellcheck disable=SC1091

source ../autoware/install/setup.bash
source install/setup.bash
ros2 launch vehicle_voice_alert_system vehicle_voice_alert_system.launch.xml
