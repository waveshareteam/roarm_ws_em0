from setuptools import setup

package_name = 'ros2web_widgets'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tygoto',
    maintainer_email='tygoto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'widgets = ros2web_widgets.widgets:main',
        ],
    },
    package_data={
        'ros2web_widgets': [
            'data/**/*',
        ],
    },
)
