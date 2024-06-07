from typing import Callable, Union, Optional, NamedTuple

from enum import Enum
import asyncio
import multiprocessing.connection

import launch.logging
from launch.action import Action
from launch.events import matches_action
from launch.events.process import ProcessStarted, ProcessExited
from launch.events.process import ProcessIO
from launch.events.process import ShutdownProcess
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch.launch_context import LaunchContext

from launch_ros.actions import Node
from launch.actions import ExecuteProcess


class ProcessStatus(Enum):
    STARTING = 'PROCESS_STATUS_STARTING'
    RUNNING = 'PROCESS_STATUS_RUNNING'
    SHUTTING_DOWN = 'PROCESS_STATUS_SHUTTING_DOWN'
    TERMINATED = 'PROCESS_STATUS_TERMINATED'


class ProcessEvent(NamedTuple):
    event_type: str
    pid: int
    text: Optional[bytes]


class Process(Action):

    def __init__(self, *, action: Union[ExecuteProcess, Node],
                 connection: multiprocessing.connection.Connection,
                 loop: asyncio.AbstractEventLoop) -> None:

        if isinstance(action, ExecuteProcess) is False:
            raise RuntimeError("The action differs from Execute Process.")

        self.__action = action
        self.__connection = connection
        self.__loop = loop
        self.__is_running = True
        self.__launch_context = None
        self.__on_exit_ext = None
        self.__pid = None

        event_handlers = [
            OnProcessStart(target_action=action, on_start=self.__on_start),
            OnProcessExit(target_action=action, on_exit=self.__on_exit),
            OnProcessIO(target_action=action,
                        on_stdout=self.__on_stdout, on_stderr=self.__on_stderr, on_stdin=self.__on_stdin)
        ]

        self.__event_handlers = event_handlers
        self._status = ProcessStatus.STARTING
        self.__logger = launch.logging.get_logger('launch.actions.process')

        super().__init__()

    def __on_start(self, event: ProcessStarted, context: LaunchContext):

        self.__logger.info('Process started: {}'.format(event.pid))

        self.__pid = event.pid
        self.__launch_context = context

        self._status = ProcessStatus.RUNNING
        process_event = ProcessEvent(
            event_type="start", pid=event.pid, text=None)

        self.__connection.send(process_event)

        self.__loop.run_in_executor(None, self.__recv_loop)

    def __on_exit(self, event: ProcessExited, context: LaunchContext):
        for event_handler in self.__event_handlers:
            self.__launch_context.unregister_event_handler(event_handler)

        self._status = ProcessStatus.TERMINATED
        
        process_event = ProcessEvent(
            event_type="exit", pid=event.pid, text=None)
        self.__connection.send(process_event)

        if self.__on_exit_ext is not None:
            self.__on_exit_ext(self, event)

    def __on_stdout(self, event: ProcessIO):
        process_event = ProcessEvent(
            event_type="stdout", pid=event.pid, text=event.text)
        self.__connection.send(process_event)

    def __on_stderr(self, event: ProcessIO):
        process_event = ProcessEvent(
            event_type="stderr", pid=event.pid, text=event.text)
        self.__connection.send(process_event)

    def __on_stdin(self, event: ProcessIO):
        pass

    def __recv_loop(self):
        while self.__is_running:
            try:
                data = self.__connection.recv()
                method_name: str = data.get('method_name')
                args = data.get('args', [])
                kwargs = data.get('kwargs', {})
                if method_name == 'shutdown':
                    self.shutdown()
            except ValueError as e:
                self.__logger.error('ValueError: '.format(e))
            except EOFError:
                pass

    def shutdown(self):
        self.__is_running = False
        self._status = ProcessStatus.SHUTTING_DOWN
        event = ShutdownProcess(process_matcher=matches_action(self.__action))
        self.__launch_context.emit_event_sync(event)

    def terminate(self):
        self.__is_running = False
        self._status = ProcessStatus.TERMINATED
        for event_handler in self.__event_handlers:
            self.__launch_context.unregister_event_handler(event_handler)

        process_event = ProcessEvent(
            event_type="exit", pid=self.__pid, text=None)
        self.__connection.send(process_event)
        self.__connection.close()

    @property
    def action(self) -> Action:
        return self.__action

    @property
    def status(self) -> ProcessStatus:
        return self._status

    @property
    def pid(self) -> int:
        return self.__pid

    @property
    def on_exit(self):
        return self.__on_exit_ext

    @on_exit.setter
    def on_exit(self, func: Callable[['Process', ProcessExited], None]):
        if callable(func):
            self.__on_exit_ext = func

    @property
    def event_handlers(self):
        return self.__event_handlers
