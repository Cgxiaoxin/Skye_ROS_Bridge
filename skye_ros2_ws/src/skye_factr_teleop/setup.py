# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "skye_factr_teleop"
config_files = glob(os.path.join("config", "*.yaml"))
launch_files = glob(os.path.join("launch", "*.launch.py"))

data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    (
        os.path.join("share", package_name),
        ["package.xml", "LICENSE", "README.md", "THIRD_PARTY_NOTICES.md"],
    ),
]

if config_files:
    data_files.append(
        (os.path.join("share", package_name, "config"), config_files)
    )

if launch_files:
    data_files.append(
        (os.path.join("share", package_name, "launch"), launch_files)
    )


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lyy",
    maintainer_email="yy.liu.cn@outlook.com",
    description="FACTR-style leader-arm teleoperation for Skye/M6",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "skye_factr_teleop_driver = skye_factr_teleop.ros_driver:main",
            "skye_factr_dynamixel_readonly_check = skye_factr_teleop.readonly_check:main",
        ],
    },
)
