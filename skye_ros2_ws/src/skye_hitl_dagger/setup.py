from setuptools import setup

package_name = 'skye_hitl_dagger'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianji',
    maintainer_email='dev@tianji.local',
    description='HITL DAgger control arbiter for Skye dual-arm teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
)
