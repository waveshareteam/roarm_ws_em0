import sys
import asyncio

import launch.logging
from launch_ros.actions import Node
from launch_api import LaunchAPI, ProcessEvent


class Turtlesim:

    def __init__(self):
        self.__process = None
        self.__logger = launch.logging.get_logger('examples.turtlesim')

        self.__launch_api = LaunchAPI()

    def start(self):
        try:
            self.__launch_api.start()
            node_action = Node(package='turtlesim', executable='turtlesim_node')
            self.__process = self.__launch_api.node(node_action)
            self.__process.on_start = self.on_start
            self.__process.on_exit = self.on_exit
            self.__process.on_stdout = self.on_stdout
            self.__process.on_stderr = self.on_stderr
        except Exception as e:
            self.__logger.error(e)

    def shutdown(self):
        self.__launch_api.shutdown()

    def on_start(self, event: ProcessEvent):
        self.__logger.info('on_start: {}'.format(event.pid))

    def on_exit(self, event: ProcessEvent):
        self.__logger.info('on_exit: {}'.format(event.pid))

    def on_stdout(self, event: ProcessEvent):
        text = event.text.decode()
        print(text.strip())

    def on_stderr(self, event: ProcessEvent):
        text = event.text.decode()
        print(text.strip())


def main(argv=None):
    turtlesim = Turtlesim()
    try:
        turtlesim.start()
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        turtlesim.shutdown()


if __name__ == '__main__':
    sys.exit(main())
