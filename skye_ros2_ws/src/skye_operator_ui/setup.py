import os
from glob import glob

from setuptools import setup

package_name = "skye_operator_ui"


def _web_dist_data_files():
    dist_root = os.path.join("web", "dist")
    if not os.path.isdir(dist_root):
        return []
    entries = []
    for root, _, filenames in os.walk(dist_root):
        for filename in filenames:
            src = os.path.join(root, filename)
            rel_dir = os.path.relpath(root, dist_root)
            dest_dir = os.path.join("share", package_name, "web")
            if rel_dir != ".":
                dest_dir = os.path.join(dest_dir, rel_dir)
            entries.append((dest_dir, [src]))
    return entries


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
        *_web_dist_data_files(),
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
