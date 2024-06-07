from typing import Optional, cast, Union, NamedTuple, Any, Callable, Set, Tuple
from typing import List, Dict

from enum import Enum
import functools

import rclpy
import rclpy.client
import rclpy.logging
import rclpy.parameter
from rclpy.task import Future
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ListParametersResult
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import ListParameters


class ParamType(Enum):
    NOT_SET = 'PARAMETER_NOT_SET'
    BOOL = 'PARAMETER_BOOL'
    INTEGER = 'PARAMETER_INTEGER'
    DOUBLE = 'PARAMETER_DOUBLE'
    STRING = 'PARAMETER_STRING'
    BYTE_ARRAY = 'PARAMETER_BYTE_ARRAY'
    BOOL_ARRAY = 'PARAMETER_BOOL_ARRAY'
    INTEGER_ARRAY = 'PARAMETER_INTEGER_ARRAY'
    DOUBLE_ARRAY = 'PARAMETER_DOUBLE_ARRAY'
    STRING_ARRAY = 'PARAMETER_STRING_ARRAY'


class Param(NamedTuple):
    node_name: str
    name: str
    value: Any


PARAM_TYPE = {
    ParameterType.PARAMETER_BOOL: ParamType.BOOL,
    ParameterType.PARAMETER_INTEGER: ParamType.INTEGER,
    ParameterType.PARAMETER_DOUBLE: ParamType.DOUBLE,
    ParameterType.PARAMETER_STRING: ParamType.STRING,
    ParameterType.PARAMETER_BYTE_ARRAY: ParamType.BYTE_ARRAY,
    ParameterType.PARAMETER_BOOL_ARRAY: ParamType.BOOL_ARRAY,
    ParameterType.PARAMETER_INTEGER_ARRAY: ParamType.INTEGER_ARRAY,
    ParameterType.PARAMETER_DOUBLE_ARRAY: ParamType.DOUBLE_ARRAY,
    ParameterType.PARAMETER_STRING_ARRAY: ParamType.STRING_ARRAY,
    ParameterType.PARAMETER_NOT_SET: ParamType.NOT_SET,
}

PARAM_PROP = {
    ParameterType.PARAMETER_BOOL: 'bool_value',
    ParameterType.PARAMETER_INTEGER: 'integer_value',
    ParameterType.PARAMETER_DOUBLE: 'double_value',
    ParameterType.PARAMETER_STRING: 'string_value',
    ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array_value',
    ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
    ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
    ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
    ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value',
    ParameterType.PARAMETER_NOT_SET: 'not_set',
}


class ParamError(RuntimeError):
    pass


def get_param_type(parameter_type: int) -> ParamType:
    param_type = PARAM_TYPE.get(parameter_type, None)
    if param_type is None:
        raise ParamError(f"Unknown parameter type ({parameter_type})")
    return param_type


def get_param_value(param: ParameterValue):
    prop = PARAM_PROP.get(param.type, None)
    if prop is not None:
        value = getattr(param, prop, None)
    else:
        raise ParamError(f"Unknown parameter type '{param.type}'")
    return value


def _get_param_descriptor(*, node_name: str, parameter_names: List[str], ros_node: Node) -> List[ParameterDescriptor]:
    client = ros_node.create_client(
        DescribeParameters, f'{node_name}/describe_parameters')

    ready = client.wait_for_service(timeout_sec=1.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = DescribeParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(ros_node, future)
    response = future.result()
    ros_node.destroy_client(client)

    if response is None or type(response.descriptors) is not list:
        raise RuntimeError('Response value does not exist.')
    if len(response.descriptors) != len(parameter_names):
        raise RuntimeError('Failed to get parameters.')

    return response.descriptors


def _get_param_names(*, node_name, prefixes: List = None, depth: Optional[int] = None, ros_node: Node) -> List[str]:
    prefixes = prefixes or []
    service_name = f'{node_name}/list_parameters'
    client = ros_node.create_client(ListParameters, service_name)

    ready = client.wait_for_service(timeout_sec=1.0)
    if not ready:
        ros_node.destroy_client(client)
        raise RuntimeError('Wait for service timed out')

    request = ListParameters.Request()
    if depth is not None:
        request.depth = depth
    request.prefixes = prefixes
    future = client.call_async(request)
    rclpy.spin_until_future_complete(ros_node, future)
    response = future.result()
    ros_node.destroy_client(client)

    return sorted(response.result.names) if response.result is not None else []


class FutureObject(NamedTuple):
    future: Future
    callback: Callable


class ParamService:
    def __init__(self, ros_node: Node) -> None:
        self.__ros_node = ros_node
        self.__logger = rclpy.logging.get_logger('ParamService')
        self.__futures: Set[FutureObject] = set()

    def set(self, node_name: str, parameters: List[RclpyParameter],
            callback: Callable[[List[SetParametersResult]], None]):

        client = self.__ros_node.create_client(SetParameters, f'{node_name}/set_parameters')

        future = self.__set_param_future(parameters=parameters, client=client)
        func = functools.partial(self.__set_param_result, client=client, callback=callback)
        future_obj = FutureObject(future=future, callback=func)
        self.__futures.add(future_obj)

    def __set_param_future(self, *, parameters: List[RclpyParameter], client: rclpy.client.Client) -> Future:
        parameter_msgs: List[Parameter] = []
        for parameter in parameters:
            parameter_msgs.append(parameter.to_parameter_msg())
        request = SetParameters.Request()
        request.parameters = parameter_msgs

        ready = client.wait_for_service(timeout_sec=1.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')

        return client.call_async(request)

    def __set_param_result(self, response, *, client: rclpy.client.Client, callback: Callable):
        self.__ros_node.destroy_client(client)
        callback(response)

    def get(self, node_name: str, parameter_names: List[str], callback: Callable[[List[Param]], None]):
        client = self.__ros_node.create_client(GetParameters, f'{node_name}/get_parameters')
        future = self.__get_param_future(parameter_names=parameter_names, client=client)
        func = functools.partial(self.__get_param_result, node_name=node_name,
                                 parameter_names=parameter_names, client=client, callback=callback)
        future_obj = FutureObject(future=future, callback=func)
        self.__futures.add(future_obj)

    def __get_param_future(self, *, parameter_names: List[str], client: rclpy.client.Client) -> Future:
        ready = client.wait_for_service(timeout_sec=1.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')
        request = GetParameters.Request()
        request.names = parameter_names
        return client.call_async(request)

    def __get_param_result(self, response, *, node_name: str, parameter_names: List[str],
                           client: rclpy.client.Client, callback: Callable):

        self.__ros_node.destroy_client(client)
        if response is None or type(response.values) is not list:
            raise RuntimeError('Response value does not exist.')
        if len(response.values) != len(parameter_names):
            raise RuntimeError('Failed to get parameters.')

        parameters = []
        for param_name, value in zip(parameter_names, response.values):
            param_value = get_param_value(value)
            param = Param(node_name=node_name, name=param_name, value=param_value)
            parameters.append(param)
        callback(parameters)

    # def describe(self, node_name: str, parameter_names: List[str]) -> List[ParameterDescriptor]:
    #     future = _get_param_descriptor(
    #         node_name=node_name,
    #         parameter_names=parameter_names,
    #         ros_node=self.__ros_node
    #     )
    #
    # def names(self, node_name: str) -> List[str]:
    #     return _get_param_names(
    #         node_name=node_name,
    #         ros_node=self.__ros_node
    #     )

    def futures(self):
        for future_obj in self.__futures.copy():
            if future_obj.future.done():
                result = future_obj.future.result()
                future_obj.callback(result)
                self.__futures.remove(future_obj)




