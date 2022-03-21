from setuptools import setup

package_name = 'teleop'

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
            ['config/teleop_node.yaml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rithwik Udayagiri',
    maintainer_email='rithwiku@seas.upenn.edu',
    description='Keyboard teleop for the gokart implemented in Python with pynput',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = teleop.teleop_node:main'
        ],
    },
)
