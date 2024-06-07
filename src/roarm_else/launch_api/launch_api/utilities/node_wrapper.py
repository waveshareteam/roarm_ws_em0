from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from . import logging_logfile_handler_remover, logging_screen_handler_remover


class NodeWrapper(Node):
    def execute(self, context: LaunchContext):
        ret = super().execute(context)

        logging_logfile_handler_remover(self._ExecuteLocal__logger)
        logging_screen_handler_remover(self._ExecuteLocal__logger)

        logging_logfile_handler_remover(self._ExecuteLocal__stdout_logger)
        logging_screen_handler_remover(self._ExecuteLocal__stdout_logger)

        logging_logfile_handler_remover(self._ExecuteLocal__stderr_logger)
        logging_screen_handler_remover(self._ExecuteLocal__stderr_logger)
        
        return ret

    class _ExecuteLocal__ProcessProtocol(ExecuteProcess._ExecuteLocal__ProcessProtocol):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)
            logging_logfile_handler_remover(self._ProcessProtocol__logger)
            logging_screen_handler_remover(self._ProcessProtocol__logger)


class ExecuteProcessWrapper(ExecuteProcess):

    def execute(self, context):
        ret = super().execute(context)

        # logging_logfile_handler_remover(self.__logger)
        # logging_screen_handler_remover(self.__logger)
        #
        # # logging_logfile_handler_remover(self.__stdout_logger)
        # logging_screen_handler_remover(self.__stdout_logger)
        #
        # # logging_logfile_handler_remover(self.__stderr_logger)
        # logging_screen_handler_remover(self.__stderr_logger)

        return ret

    # class __ProcessProtocol(ExecuteProcess._ExecuteProcess__ProcessProtocol):
    #     def __init__(self, *args, **kwargs) -> None:
    #         super().__init__(*args, **kwargs)
    #         # logging_logfile_handler_remover(self.__logger)
    #         logging_screen_handler_remover(self.__logger)
