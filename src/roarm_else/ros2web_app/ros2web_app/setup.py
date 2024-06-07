from setuptools import find_packages, setup

package_name = 'ros2web_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tygoto',
    maintainer_email='tygoto@me.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'ros2web.handler': [
            'web_rtc = ros2web_app.handler.web_rtc:WebRtcHandler',
            'app = ros2web_app.handler.app:AppHandler',
        ],
        'ros2web.verb': [
            'create = ros2web_app.verb.create:CreateVerb',
        ],
    },
    package_data={
        package_name: [
            'data/**/*',
            'data/*',
        ],
    },
)
