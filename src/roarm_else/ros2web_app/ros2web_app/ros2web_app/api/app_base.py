from types import MappingProxyType
from typing import Dict

import pathlib
import asyncio
import os.path
import re
import uuid
import ujson
import importlib.resources

import rclpy.node
import rclpy.logging

from ros2web_interfaces.msg import HTTPStatusCode
from ros2web_interfaces.srv import HTTP

from .state import AppState, AppEvent
from ..utilities import open_yaml


class AppBase(rclpy.node.Node):
    def __init__(self, *,
                 app_name: str,
                 init_state: Dict = None,
                 config: str | Dict = None,
                 loop: asyncio.AbstractEventLoop = None) -> None:
        super().__init__(app_name)

        self.__logger = rclpy.logging.get_logger(f'AppBase[{app_name}]')

        if loop is None:
            self.__loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.__loop)
        else:
            self.__loop = loop

        module_name = self.__class__.__module__
        self.__package_name = module_name.split('.')[0]

        if isinstance(config, str):
            self.__config = open_yaml(self.__package_name, config)
        elif isinstance(config, dict):
            self.__config = config
        else:
            self.__config = {}

        self.__app_state = AppState(ros_node=self, init_state=init_state, config=self.__config)
        self.__app_name = self.__app_state.app_name
        self.__app_state.on_open = self.on_open
        self.__app_state.on_close = self.on_close

        with importlib.resources.path('ros2web_app', 'data') as file_path:
            self.__ros2web_app_directory = file_path.joinpath('public')

    def start(self):
        self.__app_state.create_services()
        self.__create_services()

    def shutdown(self):
        self.__destroy_index_services()
        self.__app_state.destroy_services()
        self.destroy_node()

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        return self.__loop

    def get_state(self) -> MappingProxyType:
        return self.__app_state.state

    def set_state(self, state: Dict):
        self.__app_state.state = state

    def on_open(self, event: AppEvent):
        pass

    def on_close(self, event: AppEvent):
        pass

    def __create_services(self):
        create_new_app_srv_name = f'/http/get/{self.__app_name}/app/new'
        self.__new_app_srv = self.create_service(HTTP, create_new_app_srv_name, self.__create_app_handler)

        index_srv_name = '/http/get/' + self.__app_name
        self.__get_index_srv = self.create_service(HTTP, index_srv_name, self.__get_index)

    def __destroy_index_services(self):
        self.destroy_service(self.__new_app_srv)
        self.destroy_service(self.__get_index_srv)

    def __create_app_handler(self, request: HTTP.Request, response: HTTP.Response):
        self.__logger.debug('REQ: {}'.format(request.path))
        # query = dict(urllib.parse.parse_qsl(request.query))
        client_id = str(uuid.uuid4())
        self.__app_state.add_client(client_id)
        try:
            response.text = ujson.dumps({
                'name': self.__app_name,
                'ui': self.__config.get('ui', {}),
                'clientId': client_id,
                'state': dict(self.__app_state.state),
            })
        except ValueError:
            response.status = HTTPStatusCode.HTTP_INTERNAL_SERVER_ERROR
        return response

    def __get_index(self, request: HTTP.Request, response: HTTP.Response):
        self.__logger.info(f'get_index: {request.path}')

        if self.__ros2web_app_directory is None:
            self.__logger.warning(f'The ros2web_app directory does not exist.')
            response.status = HTTPStatusCode.HTTP_NOT_FOUND
            return response

        index_file_path = self.__ros2web_app_directory.joinpath('index.html')
        if not os.path.exists(index_file_path):
            self.__logger.warning(f"File does not exist. ({index_file_path})")
            response.status = HTTPStatusCode.HTTP_NOT_FOUND
            return response

        response.file_path = str(index_file_path)
        return response


