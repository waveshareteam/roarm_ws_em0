from typing import Optional, Union, Iterable, Callable, NamedTuple, Awaitable, TYPE_CHECKING
from typing import List, Dict, Set

import asyncio

from multiprocessing.connection import Connection

import launch.logging

from .actions.process import ProcessEvent, ProcessStatus


class ProcessProxy:
    def __init__(self, connection: Connection) -> None:
        self.__connection = connection
        self.__event_handler_dict: Dict[str, Callable[[
            ProcessEvent], None]] = {}
        self.__loop = asyncio.get_event_loop()

        self.__pid = None

        self.__status = ProcessStatus.STARTING

        self.__is_running = True

        self.__loop.run_in_executor(None, self.__update_status)

        self.__logger = launch.logging.get_logger('ProcessProxy')

    def __event_handler(self, event: ProcessEvent):
        if event.event_type == "start":
            self.__status = ProcessStatus.RUNNING
        elif event.event_type == "exit":
            self.__status = ProcessStatus.TERMINATED
            self.__is_running = False
            self.__connection.close()

        handler = self.__event_handler_dict.get(event.event_type)
        if handler is not None:
            try:
                if self.__loop.is_closed() is False:
                    handler(event)
            except Exception as e:
                self.__logger.error(e)

    def __update_status(self):
        while self.__is_running:
            try:
                data = self.__connection.recv()

                if isinstance(data, ProcessEvent):
                    self.__event_handler(data)
                else:
                    pass
            except ValueError as e:
                pass
            except EOFError:
                pass

    def shutdown(self):
        if self.__status == ProcessStatus.RUNNING:
            self.__status = ProcessStatus.SHUTTING_DOWN
            self.__connection.send({
                'method_name': 'shutdown',
                'args': [],
                'kwargs': {},
            })

    # def terminate(self):
    #     self.__is_running = False
    #     self.__connection.close()

    @property
    def pid(self) -> Optional[int]:
        return self.__pid

    @property
    def status(self) -> ProcessStatus:
        return self.__status

    @property
    def on_start(self):
        return self.__event_handler_dict.get('start')

    @on_start.setter
    def on_start(self, func: Callable[[ProcessEvent], None]):
        if callable(func):
            self.__event_handler_dict['start'] = func

    @property
    def on_exit(self):
        return self.__event_handler_dict.get('exit')

    @on_exit.setter
    def on_exit(self, func: Callable[[ProcessEvent], None]):
        if callable(func):
            self.__event_handler_dict['exit'] = func

    @property
    def on_stdout(self):
        return self.__event_handler_dict.get('stdout')

    @on_stdout.setter
    def on_stdout(self, func: Callable[[ProcessEvent], None]):
        if callable(func):
            self.__event_handler_dict['stdout'] = func

    @property
    def on_stderr(self):
        return self.__event_handler_dict.get('stderr')

    @on_stderr.setter
    def on_stderr(self, func: Callable[[ProcessEvent], None]):
        if callable(func):
            self.__event_handler_dict['stderr'] = func
