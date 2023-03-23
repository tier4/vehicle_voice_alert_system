# Vehicle Voice Alert System

## Tested environments

| OS           | python     | ros           |
| ------------ | ---------- | ------------- |
| Ubuntu 20.04 | python3.8  | ros2 galactic |
| Ubuntu 22.04 | python3.10 | ros2 humble   |

## setup

### Install Autoware

refer to here

<https://autowarefoundation.github.io/autoware-documentation/main/installation/>

### Required library

- simpleaudio

```bash
sudo pip3 install simpleaudio
```

## build

```bash
cd vehicle_voice_alert_ystem
colcon build
```

## start

```bash
source {AUTOWARE_PATH}/install/setup.bash
bash start.sh
```

## rebuild

```bash
cd vehicle_voice_alert_ystem
colcon build
```

## Sound definition

- The sound playing currently only support **wav** format

| Name           | Announce Timing                                                                                                                                                         | Priority (Descending order) | Trigger Source                                                                                       | Note                                                                                                 |
| -------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------- | ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| departure      | When the vehicle engage                                                                                                                                                 | 4                           | Service: /api/vehicle_voice/set/announce                                                             |                                                                                                      |
| emergency      | When emergency is trigger                                                                                                                                               | 4                           | Topic: /awapi/autoware/get/status<br /> Data: emergency                                              |                                                                                                      |
| in_emergency   | During emergency mode                                                                                                                                                   | 3                           | Topic: /awapi/autoware/get/status<br /> Data: emergency<br /> Flag: self.\_in_emergency_state        | Only trigger every mute_timeout during emergency                                                     |
| obstacle_stop  | When obstacle is detected                                                                                                                                               | 3                           | Topic: /awapi/autoware/get/status <br /> Data: stop_reason                                           | Only trigger after the mute_timeout period is over                                                   |
| restart_engage | when the vehicle restart the engage from stop, and velocity is more that 0                                                                                              | 4                           | Service: /api/vehicle_voice/set/announce<br /> Topic: /awapi/vehicle/get/status<br /> Data: velocity | Only trigger after the mute_timeout period is over                                                   |
| running_music  | The BGM to show that the vehicle is running. Trigger when autoware_state is Driving and will keep loop play while running.                                              | -                           | Topic: /awapi/autoware/get/status<br /> Data: autoware_state<br /> Flag: self.\_in_driving_state     | Independent from other announce. Will play together with other announce. Stop when in emergency mode |
| stop           | When the autoware_state is in following state and self.\_in_driving_state is True<br /> - WaitingForRoute<br /> - WaitingForEngage<br /> - ArrivedGoal<br /> - Planning | 4                           | Topic: /awapi/autoware/get/status<br /> Data: autoware_state<br /> Flag: not self.\_in_driving_state |                                                                                                      |
| temporary_stop | When vehicle stop in:<br /> - stop line<br /> - walkway<br /> - crosswalk<br /> - Merge from private road                                                               | 2                           | Topic: /awapi/autoware/get/status<br /> Data: stop_reason                                            | Only trigger after the mute_timeout period is over                                                   |
| turning_left   | When vehicle is turning left                                                                                                                                            | 1                           | Topic: /awapi/vehicle/get/status<br /> Data: turn_signal                                             | Trigger every mute_timeout when turning                                                              |
| turning_right  | When vehicle is turning right                                                                                                                                           | 1                           | Topic: /awapi/vehicle/get/status<br /> Data: turn_signal                                             | Trigger every mute_timeout when turning                                                              |

## License

voice and music：魔王魂、jtalk
