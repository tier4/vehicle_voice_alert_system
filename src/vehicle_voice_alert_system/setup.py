#!/usr/bin/env python3

import os
from setuptools import setup


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


package_name = "vehicle_voice_alert_system"

setup(
    name=package_name,
    version="0.0.0",
    package_dir={"": "src"},
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/resource/sound", package_files("resource/sound")),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vehicle_voice_alert_system.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="khtan",
    maintainer_email="tkh.my.p@gmail.com",
    description="The gms8 vehicle voice alert system",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    license="TODO",
    entry_points={
        "console_scripts": [
            "vehicle_voice_alert_system = vehicle_voice_alert_system.vehicle_voice_alert_system:main",
        ]
    },
)
