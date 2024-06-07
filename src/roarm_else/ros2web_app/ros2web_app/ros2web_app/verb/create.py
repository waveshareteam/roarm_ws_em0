import os
import os.path
import pkg_resources
from importlib.metadata import metadata as setup_metadata

from argparse import ArgumentParser
from ros2web.verb import VerbExtension

from ..boilerplate import create_package


class CreateVerb(VerbExtension):
    """Create a new package for ros2web_app."""

    def add_arguments(self, parser: ArgumentParser, cli_name):
        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the package directory')

        parser.add_argument(
            'package_name',
            help='The package name')

    def main(self, *, args):
        create_package(args.package_name, args.destination_directory)
