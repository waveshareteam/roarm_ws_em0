from typing import Dict, Optional
import os.path
import importlib.resources
import yaml

import rclpy.logging

logger = rclpy.logging.get_logger(__file__)


def open_yaml_file(config_file_path) -> Dict:
    config = {}

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as yml:
                config = yaml.safe_load(yml)
        except yaml.YAMLError as e:
            logger.error(f'(YAML) :{e}')
    else:
        logger.error(f"File does not exist. ({config_file_path})")

    return config


def open_yaml(package_name, filename) -> Dict:
    with importlib.resources.path(package_name, "data") as path:
        config_file_path = path.joinpath(filename)

    return open_yaml_file(config_file_path)


