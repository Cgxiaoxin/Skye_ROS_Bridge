from glob import glob
from setuptools import find_packages, setup


package_name = "skye_leader_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@todo.todo",
    description="Bridge leader-arm JointState commands to Skye arm and gripper topics.",
    license="TODO",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "leader_to_skye_bridge = skye_leader_bridge.node:main",
            "prepare_skye_impedance = skye_leader_bridge.prepare:main",
        ],
    },
)
