import asyncio
import itertools
import threading
from typing import Dict, List, Tuple
from typing import Optional, NoReturn

import uuid
import aiohttp
import rclpy.logging
import rclpy.node
from aiohttp import web
from ros2service.api import get_service_names_and_types
from ros2web_interfaces.msg import HTTPStatusCode, ContentType, BodyPart
from ros2web_interfaces.msg import WSMsg, WSMsgType
from ros2web_interfaces.srv import HTTP

from .ws_client import WSClient

TIMER_PERIOD = 30  # sec.


class Handler:
    def __init__(self, *, directory: str,
                 ip_address: str, port: int, token: str,
                 loop: asyncio.AbstractEventLoop) -> None:

        self.__timer = None
        self.__directory = directory
        self.__ip_address = ip_address
        self.__port = port
        self.__token = token
        self.__loop = loop

        self.__ros_node = None

        self.__srv_clients_lock = threading.Lock()
        self.__ws_clients_lock = threading.Lock()
        self.__srv_clients: Dict[str, rclpy.node.Client] = {}
        self.__ws_clients: Dict[str, WSClient] = {}

        self.__logger = rclpy.logging.get_logger('Handler')

    async def startup(self, ros_node: rclpy.node.Node):
        self.__ros_node = ros_node
        self.__timer = self.__ros_node.create_timer(TIMER_PERIOD, self.__interval_srv_check)

    async def shutdown(self):
        self.__ros_node.destroy_timer(self.__timer)
        for ws_client in self.__ws_clients.values():
            await ws_client.destroy()

    # Get service name and route from path
    def __get_srv_name(self, method: str, path: str) -> Tuple[Optional[str], Optional[str]]:
        method = method.lower()
        method_path = '/ws' if method == 'ws' else f'/http/{method}'

        if method == 'ws':
            path = path.replace('/ws', '')

        http_route = None
        srv_name = None

        service_names_and_types = get_service_names_and_types(node=self.__ros_node)
        http_srv_names = [service_name.replace(method_path, '')
                          for (service_name, service_type) in service_names_and_types
                          if 'ros2web_interfaces/srv/HTTP' in service_type
                          and service_name.startswith(method_path)]

        routes = sorted(http_srv_names, reverse=True)
        for route in routes:
            if path.startswith(route):
                http_route = route
                srv_name = f'{method_path}{route}'
                break

        return srv_name, http_route

    def __interval_srv_check(self):

        service_names_and_types = get_service_names_and_types(
            node=self.__ros_node)

        srv_names = [service_name
                     for (service_name, service_type) in service_names_and_types
                     if 'ros2web_interfaces/srv/HTTP' in service_type]

        ws_srv_client_keys = set()
        for key, client in self.__ws_clients.items():
            if key not in srv_names:
                ws_srv_client_keys.add(key)

        for key in ws_srv_client_keys:
            client = self.__ws_clients[key]

            asyncio.run_coroutine_threadsafe(client.destroy(), self.__loop)

            with self.__ws_clients_lock:
                del self.__ws_clients[key]

        srv_client_keys = set()
        for key, client in self.__srv_clients.items():
            if key not in srv_names:
                srv_client_keys.add(key)

        for key in srv_client_keys:
            with self.__srv_clients_lock:
                del self.__srv_clients[key]

    def __get_service_client(self, srv_name: str, timeout_sec: int = 1.0) -> Optional[rclpy.node.Client]:

        srv_client = self.__srv_clients.get(srv_name)
        if srv_client is None:
            srv_client = self.__ros_node.create_client(HTTP, srv_name)
            self.__srv_clients[srv_name] = srv_client

        ready = srv_client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            del self.__srv_clients[srv_name]
            self.__ros_node.destroy_client(srv_client)
            self.__logger.warning(
                'Wait for service timed out. ({})'.format(srv_name))
            return None

        return srv_client

    def __get_ws_client(self, srv_name: str, route: str, timeout_sec: int = 1.0) -> Optional[WSClient]:

        ws_client = self.__ws_clients.get(srv_name)
        if ws_client is None:
            ws_client = WSClient(srv_name=srv_name, route=route,
                                 ros_node=self.__ros_node, loop=self.__loop)

            with self.__ws_clients_lock:
                self.__ws_clients[srv_name] = ws_client

        ready = ws_client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            # ws_client.close()
            with self.__ws_clients_lock:
                del self.__ws_clients[srv_name]
            self.__logger.warning(
                'Wait for ws service timed out. ({})'.format(srv_name))
            return None

        return ws_client

    async def __multipart_form_data(self, request: web.Request) -> List[BodyPart]:

        reader = await request.multipart()
        body_parts = []
        while True:
            part = await reader.next()
            if part is None:
                break

            body_part = BodyPart()
            body_part.name = part.name

            headers = list(itertools.chain.from_iterable(list(part.headers.items())))
            body_part.headers = headers

            content_type = part.headers.get(aiohttp.hdrs.CONTENT_TYPE)
            if content_type is None and part.filename is None:
                content_type = ContentType.TEXT_PLAIN
            if content_type is None:
                content_type = ''

            body_part.content_type = content_type
            body_part.filename = part.filename if part.filename is not None else ''

            data = await part.read()
            body_part.data = list(data)
            body_parts.append(body_part)

        return body_parts

    async def __x_www_form_urlencoded(self, request: web.Request) -> List[BodyPart]:
        data = await request.post()

        body_parts = []
        for key, value in data.items():
            body_part = BodyPart()
            body_part.name = key
            body_parts.append(body_part)

            if isinstance(value, str):
                body_part.content_type = ContentType.TEXT_PLAIN
                content = value.encode('utf-8')
                body_part.data = list(content)
            # elif isinstance(value, bytes):
            #     body_part.data = list(value)
            # elif isinstance(value, web.FileField):
            #     body_part.content_type = value.content_type
            #     body_part.filename = value.filename
            #     headers = list(itertools.chain.from_iterable(list(value.headers.items())))
            #     body_part.headers = headers
            #     content = value.file.read()
            #     body_part.data = list(content)
        return body_parts

    async def __convert_http_request(self, request: web.Request) -> HTTP.Request:

        http_request = HTTP.Request()

        http_request.method = request.method
        http_request.path = request.path
        http_request.query = request.query_string
        http_request.content_type = request.content_type

        if request.content_type == ContentType.MULTIPART_FORM_DATA:
            http_request.multipart = await self.__multipart_form_data(request)
        elif request.content_type == ContentType.APPLICATION_X_WWW_FORM_URLENCODED:
            http_request.multipart = await self.__x_www_form_urlencoded(request)
        else:
            if request.body_exists:
                if request.content_type.startswith("text/"):
                    text = await request.text()
                    http_request.text = text
                if request.content_type.startswith("application/json"):
                    text = await request.text()
                    http_request.text = text
                else:
                    data = await request.read()
                    http_request.body = list(data)

        # Conversion from dict to list
        # ex. {key: value, key2: value2} -> [key, value, key2, value2]
        headers = list(itertools.chain.from_iterable(list(request.headers.items())))
        http_request.headers = headers

        # Conversion from list to dict
        # ex. [key, value, key2, value2] -> {key: value, key2: value2}
        # headers = dict(zip(headers[0::2], headers[1::2]))

        return http_request

    def __convert_file_response(self, response: HTTP.Response) -> web.FileResponse:
        status = response.status if response.status > 0 else 200
        reason = response.reason if response.reason != "" else None

        return web.FileResponse(response.file_path, status=status, reason=reason)

    def __convert_web_response(self, response: HTTP.Response) -> web.Response:

        body = bytearray(list(response.body)) if len(response.body) > 0 else None
        status = response.status if response.status > 0 else 200
        reason = response.reason if response.reason != "" else None
        text = response.text if body is None else None
        # headers: LooseHeaders | None = None
        content_type = response.content_type if response.content_type != "" else None
        charset = response.charset if response.charset != "" else None

        return web.Response(body=body, text=text, status=status, reason=reason,
                            content_type=content_type, charset=charset)

    async def ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        srv_name, route = self.__get_srv_name('ws', request.path)
        if route is None:
            self.__logger.error('WS: There is no route. ({})'.format(request.path))
            raise web.HTTPNotFound()

        http_request = await self.__convert_http_request(request)
        # self.__logger.info('request: {}'.format(http_request))

        ws_client = self.__get_ws_client(srv_name, route)
        if ws_client is None:
            raise web.HTTPInternalServerError()

        http_response: HTTP.Response = await ws_client.open(http_request)

        # self.__logger.info('response: {}'.format(http_response))
        if http_response is None:
            raise web.HTTPInternalServerError()

        if http_response.status == HTTPStatusCode.HTTP_NOT_FOUND:
            raise web.HTTPNotFound()
        elif http_response.status == HTTPStatusCode.HTTP_INTERNAL_SERVER_ERROR:
            raise web.HTTPInternalServerError()
        elif http_response.status == HTTPStatusCode.HTTP_UNAUTHORIZED:
            raise web.HTTPUnauthorized()

        ws = web.WebSocketResponse()
        await ws.prepare(request)
        ws_id = str(uuid.uuid4())
        try:
            await ws_client.register(ws_id, ws)
            async for ws_msg in ws:
                msg = WSMsg()
                if ws_msg.type == aiohttp.WSMsgType.TEXT:
                    msg.route = route
                    msg.ws_id = ws_id
                    msg.type = WSMsgType.TEXT
                    msg.data.str = ws_msg.data
                    msg.extra = ws_msg.extra
                    ws_client.publish(msg)
                # elif ws_msg.type == aiohttp.WSMsgType.BINARY:
                #     msg.type = MsgType.BINARY
                #     msg.data.bytes = ws_msg.data
        except RuntimeError as e:
            self.__logger.error(f"RuntimeError: {e}")
        except ValueError as e:
            self.__logger.error(f"ValueError: {e}")
        except TypeError as e:
            self.__logger.error(f"TypeError: {e}")

        try:
            await ws_client.unregister(ws_id)
        except Exception as e:
            self.__logger.error(f"Exception: {e}")

        return ws

    async def web_handler(self, request: web.Request) -> web.StreamResponse:
        self.__logger.info('REQ: {}'.format(request.path))

        ws_key = request.headers.get('Sec-WebSocket-Key')
        if ws_key is not None:
            return await self.ws_handler(request)

        # time_start = time.perf_counter()
        # elapsed_time = time.perf_counter() - time_start
        # print('elapsed time: ', elapsed_time)

        srv_name, route = self.__get_srv_name(request.method, request.path)
        if route is None:
            self.__logger.warning(
                'There is no route. ({})'.format(request.path))
            raise web.HTTPNotFound()

        http_request = await self.__convert_http_request(request)
        # self.__logger.info('request: {}'.format(http_request))
        http_request.srv_name = srv_name
        http_request.route = route

        srv_client = self.__get_service_client(srv_name)
        if srv_client is None:
            raise web.HTTPInternalServerError()

        try:
            http_response: HTTP.Response = await asyncio.wait_for(
                srv_client.call_async(http_request), timeout=5)
        except asyncio.TimeoutError:
            self.__logger.warning('TimeoutError: {}'.format(request.path))
            raise web.HTTPRequestTimeout()

        self.__logger.info('response: {}'.format(http_response))
        if http_response is None:
            raise web.HTTPInternalServerError()
        if http_response.status == HTTPStatusCode.HTTP_NOT_FOUND:
            raise web.HTTPNotFound()
        elif http_response.status == HTTPStatusCode.HTTP_INTERNAL_SERVER_ERROR:
            raise web.HTTPInternalServerError()

        if http_response.file_path != "":
            return self.__convert_file_response(http_response)
        else:
            return self.__convert_web_response(http_response)
