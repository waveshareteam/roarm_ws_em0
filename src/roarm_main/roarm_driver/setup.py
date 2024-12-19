from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roarm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This registers the package as a resource.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Install the package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Install all launch files into the share/<package_name>/launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roarm_driver = roarm_driver.roarm_driver:main'
        ],
    },
)
