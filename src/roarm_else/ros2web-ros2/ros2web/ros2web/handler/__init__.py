from aiohttp import web

import rclpy.node
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version

HANDLER_EXTENSION_POINT_NAME = 'ros2web.handler'


class HandlerExtension:
    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self) -> None:
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    async def handle(self, request: web.Request) -> web.StreamResponse:
        raise NotImplementedError()

    async def startup(self, ros_node: rclpy.node.Node):
        raise NotImplementedError()

    async def shutdown(self):
        raise NotImplementedError()
