from typing import Optional, Union, Set

import asyncio
import multiprocessing as mp
import multiprocessing.connection as mp_conn

import concurrent.futures
import concurrent.futures.process
import threading

import launch.logging
from launch import LaunchService, LaunchDescription

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import osrf_pycommon.process_utils

from .actions.process_manager import ProcessManager
from .process_proxy import ProcessProxy


class DynamicLaunchService(mp.Process):
    def __init__(self) -> None:
        super().__init__()

        self.__launch_service: Optional[LaunchService] = None
        self.__process_manager = None
        self.__loop = None
        self.__method_loop_future = None

        self.__processes: Set[ProcessProxy] = set()
        self.__call_lock = threading.Lock()
        parent_conn, child_conn = mp.Pipe()
        self.__parent_conn = parent_conn
        self.__child_conn = child_conn
        self.__is_running = mp.Value('i', False)

        self.__logger = launch.logging.get_logger('DynamicLaunchService')

    @property
    def is_running(self) -> bool:
        return bool(self.__is_running.value)

    def execute_process(self, *, action: Union[Node, ExecuteProcess]) -> ProcessProxy:
        if isinstance(action, ExecuteProcess) is False:
            raise RuntimeError("The action differs from Execute Process.")

        kwargs = {
            'action': action
        }
        response = self.__call_method("_execute_process", [], kwargs)
        if isinstance(response, mp_conn.Connection) is False:
            raise RuntimeError("The response differs from Connection.")

        process = ProcessProxy(response)
        self.__processes.add(process)
        return process

    def shutdown(self):
        self.__call_method("_shutdown", [], {})

    async def _execute_process(self, *, action: Union[Node, ExecuteProcess]):
        parent_conn, child_conn = mp.Pipe()
        self.__process_manager.execute_process(action=action, connection=parent_conn)
        return child_conn

    async def _shutdown(self):
        self.__is_running.value = False
        future = self.__launch_service.shutdown(force_sync=True)
        if future is not None:
            await future
        return

    def run(self):
        try:
            self.__is_running.value = True
            self.__process_manager = ProcessManager()
            self.__loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.__loop)

            self.__launch_service = LaunchService(argv=[], debug=False)
            self.__loop.run_in_executor(None, self.__method_loop)
            self.__launch_service.context.extend_globals({})

            launch_description = LaunchDescription([
                self.__process_manager
            ])
            self.__launch_service.include_launch_description(launch_description)

            loop = osrf_pycommon.process_utils.get_loop()
            run_async_task = loop.create_task(self.__launch_service.run_async(
                shutdown_when_idle=True
            ))

            while self.is_running:
                try:
                    loop.run_until_complete(run_async_task)
                    break
                except KeyboardInterrupt:
                    continue
        finally:
            self.__loop.run_until_complete(asyncio.sleep(1))
            self.__loop.close()

    def __call_method(self, method_name, args, kwargs):
        request = {
            'method_name': method_name,
            'args': args,
            'kwargs': kwargs
        }
        response = None
        try:
            with self.__call_lock:
                self.__parent_conn.send(request)
                if self.__parent_conn.poll(timeout=1.5):
                    response = self.__parent_conn.recv()
        except EOFError as e:
            self.__logger.error("[{}]: EOFError: {}".format(method_name, e))
        except ValueError as e:
            self.__logger.error("[{}]: ValueError: {}".format(method_name, e))
        except BrokenPipeError as e:
            self.__logger.error("[{}]: BrokenPipeError: {}".format(method_name, e))

        return response

    def __method_loop(self):
        while self.is_running:
            try:
                self.__child_conn.send(
                    self.__call_coroutine(self.__child_conn.recv())
                )
            except ValueError as e:
                self.__logger.error('service_loop: '.format(e))
            except EOFError:
                pass
            # except Exception as e:
            #     self.__logger.error('service_loop: '.format(e))
        self.__child_conn.close()

    def __call_coroutine(self, recv_request, timeout: float = 1.0):
        response = None
        future = None
        method_name: str = recv_request.get('method_name')
        args = recv_request.get('args', [])
        kwargs = recv_request.get('kwargs', {})
        try:
            if not self.__loop.is_closed():
                future = asyncio.run_coroutine_threadsafe(
                    getattr(self, method_name)(*args, **kwargs), self.__loop)
                response = future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            self.__logger.error(
                'The coroutine took too long, cancelling the task...')

            if future is not None:
                future.cancel()
        except Exception as e:
            self.__logger.error('call_service[{}]: {}'.format(method_name, e))
        return response


