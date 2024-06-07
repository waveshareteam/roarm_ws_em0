from typing import Dict, NamedTuple
import getpass
import os
import shutil
import subprocess
# import pkg_resources
import os.path
import importlib.resources

import launch.logging

from catkin_pkg.package import Dependency, Export, Person
from ros2pkg.api.create import _expand_template

logger = launch.logging.get_logger(os.path.basename(__file__))


def _create_template_file(template_file_name, output_directory, output_file_name, template_config):
    if not os.path.exists(template_file_name):
        raise FileNotFoundError('template not found:', template_file_name)
    output_file_path = os.path.join(output_directory, output_file_name)
    _expand_template(template_file_name, template_config, output_file_path)


class Package(NamedTuple):
    path: str
    name: str
    resource: str
    src: str
    data: str


def _create_python_files(*, package: Package, maintainer: Person):
    with importlib.resources.path('ros2web_app', 'data') as path:
        template_path = path.joinpath('template')

    python_template_path = os.path.join(template_path, 'python')
    package_license = 'TODO: License declaration'
    package_description = 'TODO: Package description'

    _create_template_file(os.path.join(python_template_path, 'package.xml.em'), package.path,
                          'package.xml', {
                              'package_format': 3,
                              'package_name': package.name,
                              'package_description': package_description,
                              'maintainer_email': maintainer.email,
                              'maintainer_name': maintainer.name,
                              'package_license': package_license,
                              'dependencies': [Dependency(dep) for dep in ['ros2web', 'ros2web_app']],
                              'exports': [Export('build_type', content='ament_python')],
                          })

    app_module = f'{package.name}.{package.name}:main'
    _create_template_file(os.path.join(python_template_path, 'setup.py.em'), package.path,
                          'setup.py', {
                              'package_name': package.name,
                              'app_module': app_module,
                              'maintainer_email': maintainer.email,
                              'maintainer_name': maintainer.name,
                              'package_license': package_license,
                              'package_description': package_description,
                          })

    _create_template_file(os.path.join(python_template_path, 'setup.cfg.em'), package.path,
                          'setup.cfg', {
                              'package_name': package.name,
                          })

    _create_template_file(os.path.join(python_template_path, 'resource_file.em'),
                          package.resource, package.name, {})

    _create_template_file(os.path.join(python_template_path, '__init__.py.em'),
                          package.src, '__init__.py', {})

    app_file_name = f'{package.name}.py'
    _create_template_file(os.path.join(python_template_path, 'app.py.em'),
                          package.src,
                          app_file_name, {
                              'package_name': package.name
                          })
    _create_template_file(os.path.join(python_template_path, 'config.yml.em'),
                          package.data, 'config.yml', {})


def _create_maintainer() -> Person:
    maintainer = Person(getpass.getuser())
    git = shutil.which('git')
    if git is not None:
        p = subprocess.Popen(
            [git, 'config', '--global', 'user.email'],
            stdout=subprocess.PIPE)
        resp = p.communicate()
        email = resp[0].decode().rstrip()
        if email:
            maintainer.email = email
    if not maintainer.email:
        maintainer.email = maintainer.name + '@todo.todo'

    return maintainer


def _create_directory(package_name: str, destination_directory: str):
    package_path = os.path.join(destination_directory, package_name)

    if os.path.exists(package_path):
        raise RuntimeError('\nAborted!\nThe directory already exists: ' + package_path + '\nEither ' +
                           'remove the directory or choose a different destination directory or package name')

    resource_directory = os.path.join(package_path, 'resource')
    src_directory = os.path.join(package_path, package_name)
    data_directory = os.path.join(src_directory, 'data')

    os.mkdir(package_path)
    os.mkdir(resource_directory)
    os.mkdir(src_directory)
    os.mkdir(data_directory)

    return Package(
        path=package_path,
        name=package_name,
        resource=resource_directory,
        src=src_directory,
        data=data_directory
    )


def create_package(package_name: str, destination_directory):
    package_path = None
    try:
        package = _create_directory(
            package_name, destination_directory)
        package_path = package.path

        maintainer = _create_maintainer()
        _create_python_files(package=package, maintainer=maintainer)

        print(f'Successfully created {package_name}.')
    except Exception as e:
        logger.error(e)
        if package_path and os.path.exists(package_path):
            shutil.rmtree(package_path)