from glob import glob

from setuptools import setup

package_name = 'kuka_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/krl', glob('krl/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jm',
    maintainer_email='josemiguelsantost@gmail.com',
    description='Puente ROS 2 - KUKA KR C4 via KUKAVARPROXY',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'kuka_bridge = kuka_bridge.kuka_bridge_node:main',
        'kuka_vars = kuka_bridge.read_vars:main',
        'scene_publisher = kuka_bridge.scene_publisher:main',
        'go_home = kuka_bridge.go_home:main',
    ]},
)
