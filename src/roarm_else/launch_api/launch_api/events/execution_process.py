from typing import TYPE_CHECKING

from launch.event import Event

if TYPE_CHECKING:
    from ..actions import Process # noqa: F401


class ExecutionProcess(Event):
    name = 'ros_run.launch.events.ExecutionProcess'
    
    def __init__(self, *, process: 'Process') -> None:
        self.__process = process
    
    @property
    def process(self):
        return self.__process
