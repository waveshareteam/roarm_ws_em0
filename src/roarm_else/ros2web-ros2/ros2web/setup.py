from setuptools import find_packages
from setuptools import setup

package_name = 'ros2web'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aiohttp'],
    zip_safe=True,
    maintainer='tygoto',
    maintainer_email='tygoto@me.com',
    description='The ros2web is software that forwards HTTP requests to ROS2 packages.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'web = ros2web.command.web:WebCommand',
        ],
        'ros2cli.extension_point': [
            'ros2web.verb = ros2web.verb:VerbExtension',
        ],
        'ros2web.verb': [
            'server = ros2web.verb.server:ServerVerb',
        ],
    },
)
