from setuptools import setup

package_name = "skye_data_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/data_recorder.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tianji",
    maintainer_email="dev@tianji.local",
    description="Teleop mcap recorder for applied actions",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "data_recorder = skye_data_recorder.data_recorder_node:main",
        ],
    },
    tests_require=["pytest"],
)
