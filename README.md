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

### requried library
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

## License

voice and music：魔王魂、jtalk
