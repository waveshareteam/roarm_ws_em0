import os
import pathlib
from importlib import resources
import re
from aiohttp import web
import rclpy.logging
from ros2web.handler import HandlerExtension


class AppHandler(HandlerExtension):
    def __init__(self) -> None:
        super().__init__()
        self.__logger = rclpy.logging.get_logger('AppHandler')
        with resources.path('ros2web_app', 'data') as file_path:
            self.__ros2web_app_directory = file_path.joinpath('public')

    async def startup(self, ros_node: rclpy.node.Node):
        pass

    async def shutdown(self):
        pass

    async def handle(self, request: web.Request) -> web.StreamResponse:
        self.__logger.info('REQ: {}'.format(request.path))
        if request.method != 'GET':
            raise web.HTTPNotFound()

        file_match = re.match(r'/(?P<file_name>.+\.\w+)$', request.path)
        if file_match is not None:
            file_name = file_match.group('file_name')
        else:
            raise web.HTTPNotFound()

        index_file_path = self.__ros2web_app_directory.joinpath(file_name)
        base_path = pathlib.Path(self.__ros2web_app_directory).resolve()
        target_path = index_file_path.resolve()

        if str(target_path).startswith(str(base_path)) is False:
            self.__logger.warning(f'Invalid path. ({request.path})')
            raise web.HTTPNotFound()

        if not os.path.exists(target_path):
            self.__logger.warning(f"File does not exist. ({target_path})")
            raise web.HTTPNotFound()

        return web.FileResponse(target_path)
