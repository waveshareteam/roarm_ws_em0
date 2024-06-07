from setuptools import setup

package_name = 'ros2web_example_py'

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
    maintainer_email='tygoto@me.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api = ros2web_example_py.api:main',
            'form = ros2web_example_py.form:main',
        ],
    },
    package_data={
        'ros2web_example_py': [
            'data/**/*',
        ],
    },
)
