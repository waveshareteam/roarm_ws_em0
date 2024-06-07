from typing import NoReturn

import asyncio
import functools
import os
import os.path
import pathlib
import secrets
import socket
import ssl
import re
from urllib.parse import urlencode

import rclpy
import rclpy.logging
from ros2cli.entry_points import get_entry_points

from aiohttp import web

from ..api.handler import Handler
from ..api.ros_executor import ROSExecutor
from ..utilities import get_ip_address
from ..handler import HANDLER_EXTENSION_POINT_NAME, HandlerExtension
from . import VerbExtension


@web.middleware
async def token_auth(request: web.Request, handler):
    _token = request.app["token"]
    search_params = request.rel_url.query

    token_param = search_params.get("token")
    if token_param:
        if _token != token_param:
            raise web.HTTPUnauthorized()
        else:
            param = {}
            for param_key, param_value in search_params.items():
                if param_key != "token":
                    param[param_key] = param_value
            query_string = urlencode(param)
            location = request.path + '?' + query_string
            exc = web.HTTPFound(location=location)
            exc.set_cookie("AUTH", _token)
            raise exc

    token_header = request.headers.get("token")
    if token_header:
        token = token_header
    else:
        token = request.cookies.get("AUTH")

    if _token != token:
        raise web.HTTPUnauthorized()

    response = await handler(request)
    return response


class ServerVerb(VerbExtension):

    def __init__(self):
        super().__init__()

        self.__handler = None
        self.__token = None
        self.__ros_executor = None
        self.__logger = rclpy.logging.get_logger('ServerVerb')
        self.__plugin_handlers = {}

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--bind',
            help='Specify alternate bind address [default: all interfaces]')
        parser.add_argument(
            '--port',
            default='8080',
            help='Specify alternate port [default: 8080]')

        parser.add_argument(
            '--destination-directory',
            default=os.path.join(os.path.expanduser('~'), '.ros2web'),
            help='Directory where to create the config directory')

        parser.add_argument(
            '--node-name',
            default='ros2web',
            help='Node name [default: ros2web]')

        parser.add_argument(
            '--namespace',
            default=None,
            help='namespace [default: None]')

        parser.add_argument(
            '--log-level',
            default='ERROR',
            choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
            help='Log Level [default: ERROR]')

        parser.add_argument(
            '--no-auth',
            action='store_true',
            help='Token is not used.')

    def run(self, *, node_name: str, namespace: str,
            ip_address: str, host: str, port: int, token: str,
            directory: str, log_level: str, loop: asyncio.AbstractEventLoop):

        app = self.__init_web_app(node_name=node_name, namespace=namespace,
                                  ip_address=ip_address, port=port,
                                  directory=directory,
                                  token=token, log_level=log_level, loop=loop)

        ssl_context = self.__get_ssl_context(directory)
        scheme = "https" if ssl_context else "http"
        print_url = functools.partial(self.__print_url, scheme, ip_address, host, port, token)
        web.run_app(app, host=host, port=port,
                    loop=loop, ssl_context=ssl_context, print=print_url)

    def available_port(self, ip_address, port):
        is_available = False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = sock.connect_ex((ip_address, port))
        if result != 0:
            is_available = True
        sock.close()
        return is_available

    def __print_url(self, scheme:str, ip_address: str, host:str, port: str,
                    token: str, string: str):

        host = host if host is not None else '0.0.0.0'
        if token is not None:
            url1= f'{scheme}://{ip_address}:{port}/?token={token}'
            url2 = f'{scheme}://{host}:{port}/?token={token}'
        else:
            url1 = f'{scheme}://{ip_address}:{port}'
            url2 = f'{scheme}://{host}:{port}'
        print(
            "------------------------------------------------------------------------\n"
            "{}\n"
            "{}\n"
            "------------------------------------------------------------------------\n"
            "(Press CTRL+C to quit)\n".format(url1, url2)
        )

    def __get_ssl_context(self, directory: str):
        here = pathlib.Path(directory)
        ssl_cert = here / "ssl" / "server.crt"
        ssl_key = here / "ssl" / "server.key"
        ssl_context = None
        if ssl_cert.exists() and ssl_key.exists():
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(str(ssl_cert), str(ssl_key))

        return ssl_context

    def __init_web_app(self, *, node_name: str, namespace: str,
                       ip_address: str, port: int, token: str,
                       directory: str, log_level: str,
                       loop: asyncio.AbstractEventLoop) -> web.Application:

        self.__ros_executor = ROSExecutor(node_name=node_name,
                                          namespace=namespace,
                                          args=['--ros-args', '--log-level', log_level],
                                          loop=loop)

        self.__handler = Handler(directory=directory,
                                 ip_address=ip_address, port=port, token=token,
                                 loop=loop)

        extension_points = get_entry_points(HANDLER_EXTENSION_POINT_NAME)
        for name, entry_point in extension_points.items():
            if re.fullmatch(r'[a-zA-Z0-9_]+', name) is None:
                self.__logger.warning('invalid plugin name: {}'.format(name))
                continue

            handler = entry_point.load()
            if issubclass(handler, HandlerExtension):
                self.__logger.info('load plugin: {}'.format(name))
                try:
                    self.__plugin_handlers[name] = handler()
                except Exception as e:
                    self.__logger.error('plugin: {} load error: {}'.format(name, e))
            else:
                self.__logger.warning('invalid plugin: {}'.format(name))

        if token is None:
            app = web.Application()
        else:
            app = web.Application(middlewares=[token_auth])
            app['token'] = token

        app.router.add_get(r'/{path:plugin(/?|/.*)}', self.plugin_handler)
        app.router.add_post(r'/{path:plugin(/?|/.*)}', self.plugin_handler)
        app.router.add_put(r'/{path:plugin(/?|/.*)}', self.plugin_handler)
        app.router.add_delete(r'/{path:plugin(/?|/.*)}', self.plugin_handler)

        app.router.add_get('/{path:.*}', self.__handler.web_handler)
        app.router.add_post('/{path:.*}', self.__handler.web_handler)
        app.router.add_put('/{path:.*}', self.__handler.web_handler)
        app.router.add_delete('/{path:.*}', self.__handler.web_handler)

        app.on_startup.append(self.on_startup)
        app.on_shutdown.append(self.on_shutdown)
        app.on_cleanup.append(self.on_cleanup)

        return app

    async def root_handler(self, request: web.Request) -> NoReturn:
        raise web.HTTPNotFound()

    async def plugin_handler(self, request: web.Request) -> web.StreamResponse:
        self.__logger.info('REQ: {}'.format(request.path))
        m = re.match(r'^/plugin/(?P<name>[a-zA-Z0-9_]+)(/?|/.*)', request.path)
        if m is not None:
            name = m.group('name')
            handler = self.__plugin_handlers.get(name)
            if handler is not None:
                try:
                    return await handler.handle(request)
                except Exception as e:
                    self.__logger.error('plugin: {} handle error: {}'.format(name, e))
                    raise web.HTTPInternalServerError()
        raise web.HTTPNotFound()

    async def on_startup(self, app: web.Application) -> None:
        self.__logger.info('on_startup')
        self.__ros_executor.start()
        await self.__handler.startup(ros_node=self.__ros_executor.node)

        for plugin_name, handler in self.__plugin_handlers.items():
            try:
                await handler.startup(ros_node=self.__ros_executor.node)
            except Exception as e:
                self.__logger.error('plugin: {} startup error: {}'.format(plugin_name, e))

    async def on_shutdown(self, app: web.Application) -> None:
        self.__logger.info('on_shutdown')
        for plugin_name, handler in self.__plugin_handlers.items():
            try:
                await handler.shutdown()
            except Exception as e:
                self.__logger.error('plugin: {} shutdown error: {}'.format(plugin_name, e))
        await self.__handler.shutdown()
        self.__ros_executor.shutdown()

    async def on_cleanup(self, app: web.Application) -> None:
        self.__logger.info('on_cleanup')
        await asyncio.sleep(0.2)
        # await self.__handler.cleanup()

    def main(self, *, args):
        port = int(args.port)
        directory = args.destination_directory
        host = args.bind
        log_level = args.log_level
        node_name = args.node_name
        namespace = args.namespace
        no_auth = args.no_auth
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        token = None if no_auth is True else secrets.token_hex(16)
        ip_address = get_ip_address() if host is None else host

        if self.available_port(ip_address, port):
            self.run(node_name=node_name, namespace=namespace,
                     ip_address=ip_address, host=host, port=port, token=token,
                     directory=directory, log_level=log_level, loop=loop)
        else:
            print("Address('{}', {}) already in use".format(ip_address, port))