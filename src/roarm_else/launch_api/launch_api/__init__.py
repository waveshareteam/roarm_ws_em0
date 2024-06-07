from typing import Optional, NamedTuple

import launch.logging
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from .service import DynamicLaunchService, ProcessProxy
from .actions.process import ProcessEvent, ProcessStatus


class LaunchAPI:
    def __init__(self):
        self.__launch_service: Optional[DynamicLaunchService] = None
        self.__logger = launch.logging.get_logger('RosRun')

    def start(self):
        self.__launch_service = DynamicLaunchService()
        self.__launch_service.start()

    def shutdown(self):
        if self.__launch_service is None:
            return
        self.__launch_service.shutdown()
        self.__launch_service.join()
        # self.__launch_service.terminate()
        # self.__launch_service.close()

    def node(self, action: Node) -> ProcessProxy:
        if self.__launch_service is None:
            raise RuntimeError('RosRun has not started.')

        return self.__launch_service.execute_process(action=action)

    def process(self, *, action: ExecuteProcess) -> ProcessProxy:
        if self.__launch_service is None:
            raise RuntimeError('RosRun has not started.')

        return self.__launch_service.execute_process(action=action)
