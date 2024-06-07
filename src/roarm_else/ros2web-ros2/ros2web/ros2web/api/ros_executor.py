import concurrent.futures
from asyncio import AbstractEventLoop
from typing import List
from typing import Optional

import rclpy
import rclpy.parameter
# from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class ROSExecutor:
    def __init__(self, *,
                 node_name: str,
                 namespace: Optional[str] = None,
                 args: Optional[List[str]] = None,
                 loop: AbstractEventLoop) -> None:

        self.__node = None
        self.__ros_executor = None
        self.__loop = loop
        self.__node_name = node_name
        self.__namespace = namespace
        self.__running = False

        self.__ros_context = rclpy.Context()
        rclpy.init(args=args, context=self.__ros_context)

    def start(self):
        self.__running = True
        self.__ros_executor = SingleThreadedExecutor(
            context=self.__ros_context)
        self.__node = rclpy.create_node(self.__node_name,
                                        namespace=self.__namespace,
                                        context=self.__ros_context)
        self.__ros_executor.add_node(self.__node)
        executor = concurrent.futures.ThreadPoolExecutor()
        self.__loop.run_in_executor(executor, self.__ros_loop)

    def shutdown(self):
        self.__running = False
        self.__node.destroy_node()
        self.__ros_executor.remove_node(self.__node)
        rclpy.shutdown(context=self.__ros_context)

    @property
    def node(self) -> Node:
        return self.__node

    def __ros_loop(self):
        try:
            while self.__running:
                self.__ros_executor.spin_once(timeout_sec=1.0)
        except ExternalShutdownException:
            pass
