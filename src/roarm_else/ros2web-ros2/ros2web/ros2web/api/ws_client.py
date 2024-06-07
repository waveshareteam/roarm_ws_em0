import asyncio
import weakref
from typing import Dict
from typing import Optional

import rclpy.logging
import rclpy.node
from aiohttp import WSCloseCode
from aiohttp import web
from ros2web_interfaces.msg import WSMsg, WSMsgType
from ros2web_interfaces.srv import HTTP


class WSClient:

    def __init__(self, *, srv_name: str, route: str,
                 ros_node: rclpy.node.Node, loop: asyncio.AbstractEventLoop) -> None:

        self.__logger = rclpy.logging.get_logger('WSClient')

        self.__loop = loop
        self.__ros_node = ros_node
        self.__srv_name = srv_name
        self.__route = route

        pub_topic = f'ws{route}/pub'
        sub_topic = f'ws{route}/sub'

        self.__srv_client = self.__ros_node.create_client(HTTP, srv_name)
        self.__subscription = self.__ros_node.create_subscription(WSMsg, pub_topic, self.__on_subscribe, 10)
        self.__publisher = self.__ros_node.create_publisher(WSMsg, sub_topic, 10)

        self.__ws_lock = asyncio.Lock()
        self.__websocket_dict = weakref.WeakValueDictionary()

    def wait_for_service(self, timeout_sec: float = None) -> bool:
        return self.__srv_client.wait_for_service(timeout_sec=timeout_sec)

    async def open(self, http_request: HTTP.Request):
        http_request.srv_name = self.__srv_name
        http_request.route = self.__route
        ready = self.__srv_client.wait_for_service(timeout_sec=1.0)
        if not ready:
            raise RuntimeError('Wait for service timed out. ({})'.format(self.__srv_name))

        http_response: HTTP.Response = await self.__srv_client.call_async(http_request)
        if http_response is None:
            raise RuntimeError('response error')

        return http_response

    async def register(self, ws_id: str, ws: web.WebSocketResponse):
        async with self.__ws_lock:
            self.__websocket_dict[ws_id] = ws

    async def unregister(self, ws_id: str):
        async with self.__ws_lock:
            del self.__websocket_dict[ws_id]

        msg = WSMsg()
        msg.type = WSMsgType.CLOSE
        msg.ws_id = ws_id

        self.__publisher.publish(msg)

    def publish(self, msg: WSMsg):
        self.__publisher.publish(msg)

    def __on_subscribe(self, msg: WSMsg):
        if msg.ws_id == '' and msg.route == '':
            raise RuntimeError('Invalid message')

        ws_id = None if msg.ws_id == '' else msg.ws_id
        if msg.type == WSMsgType.TEXT:
            data = msg.data.str
            asyncio.run_coroutine_threadsafe(self.__send_str(data, ws_id=ws_id), self.__loop)
        elif msg.type == WSMsgType.BINARY:
            pass
        elif msg.type == WSMsgType.CLOSE:
            if ws_id is not None:
                future = asyncio.run_coroutine_threadsafe(self.__close_ws(ws_id), self.__loop)
                try:
                    future.result(timeout=1)
                except TimeoutError:
                    self.__logger.error('TimeoutError')
                    future.cancel()
                except Exception as e:
                    self.__logger.error(e)

    async def __send_str(self, data: str, *, ws_id: Optional[str]):
        async with self.__ws_lock:
            if ws_id is None:
                await asyncio.wait([ws.send_str(data)
                                    for ws in self.__websocket_dict.values() if ws.closed is False])
            else:
                ws = self.__websocket_dict.get(ws_id)
                await ws.send_str(data)

    async def __close_ws(self, ws_id: str):
        async with self.__ws_lock:
            ws = self.__websocket_dict.get(ws_id)
            if ws.closed is False:
                await ws.close(code=WSCloseCode.GOING_AWAY, message='ws client destroy')

    async def destroy(self):
        async with self.__ws_lock:
            await asyncio.wait([client.close(code=WSCloseCode.GOING_AWAY, message='ws client destroy')
                                for client in self.__websocket_dict.values() if client.closed is False])

        self.__ros_node.destroy_client(self.__srv_client)
        self.__ros_node.destroy_publisher(self.__publisher)
        self.__ros_node.destroy_subscription(self.__subscription)
