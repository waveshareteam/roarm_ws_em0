from setuptools import setup

package_name = 'roarm_web_app'

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
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'roarm_web_app = roarm_web_app.roarm_web_app:main',
        ],
    },
    package_data={
        'roarm_web_app': [
            'data/**/*',
        ],
    },
)
