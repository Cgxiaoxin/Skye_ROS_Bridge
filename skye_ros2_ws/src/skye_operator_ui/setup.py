import os
from glob import glob

from setuptools import setup

package_name = "skye_operator_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "web"),
         glob(os.path.join("web", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tianji",
    maintainer_email="dev@tianji.local",
    description="FastAPI host console for teleop recording and HITL DAgger",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "operator_ui = skye_operator_ui.operator_ui_node:main",
        ],
    },
)
