from setuptools import setup

package_name = 'joy_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/config',
            ['config/joy_teleop.yaml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/joy_teleop.launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Endler',
    maintainer_email='endlemar@fel.cvut.cz',
    description='Joystick teleop for the gokart implemented in Python with pynput',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop_node = joy_teleop.joy_teleop_node:main'
        ],
    },
)
