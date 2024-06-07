from typing import Callable, NamedTuple, Set, Optional

import functools

import rclpy
import rclpy.logging
from rclpy.client import Client
from rclpy.task import Future
from rclpy.node import Node


class FutureObject(NamedTuple):
    future: Future
    callback: Optional[Callable]


class Service:
    def __init__(self, ros_node: Node) -> None:
        self.__ros_node = ros_node
        self.__logger = rclpy.logging.get_logger('Service')
        self.__futures: Set[FutureObject] = set()

    def futures(self):
        for future in self.__futures.copy():
            if future.future.done():
                future.callback(future.future.result())
                self.__futures.remove(future)

    def call(self, client: Client, request, callback: Optional[Callable] = None):
        future = self.__call_future(request=request, client=client)
        func = functools.partial(self.__call_result, client=client, callback=callback)
        future_obj = FutureObject(future=future, callback=func)
        self.__futures.add(future_obj)

    def __call_future(self, *, request, client: rclpy.client.Client) -> Future:
        ready = client.wait_for_service(timeout_sec=1.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')
        return client.call_async(request)

    def __call_result(self, response, *, client: rclpy.client.Client, callback: Optional[Callable] = None):
        self.__ros_node.destroy_client(client)
        if callback is not None:
            callback(response)
