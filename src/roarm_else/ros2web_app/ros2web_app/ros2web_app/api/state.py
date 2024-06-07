from typing import Dict, List, Any
from typing import Optional, NamedTuple, Callable
from types import MappingProxyType

import threading
import ujson
import re
import urllib.parse

import rclpy.node
import rclpy.logging

from ros2web_interfaces.msg import WSMsg, WSMsgData, WSMsgType, HTTPStatusCode
from ros2web_interfaces.srv import HTTP

from ..utilities.ros_node import check_duplicates


class AppEvent(NamedTuple):
    type: str
    client_id: str
    state: MappingProxyType
    value: Optional[Any] = None


class AppState:

    def __init__(self, *, ros_node: rclpy.node.Node,
                 init_state: Dict, config: Dict = None) -> None:

        self.__app_name = ros_node.get_name()
        if check_duplicates(self.__app_name, ros_node=ros_node):
            raise RuntimeError("Duplicate name ({}).".format(self.__app_name))

        self.__ws_srv = None
        self.__publisher = None
        self.__subscription = None
        self.__route = None
        self.__config = config or {}
        self.__ros_node = ros_node

        self.__client_ids: Dict[str, Optional[str]] = {}
        self.__event_handler_dict: Dict[str, Callable] = {}
        self.__handlers: Dict[str, Callable] = {}

        init_state = init_state or {}
        for key, value in init_state.items():
            if callable(value):
                event_id = f"$${{{key}}}"
                init_state[key] = event_id
                self.__handlers[key] = value

        self.__state_keys = set(init_state.keys())
        self.__state = ujson.loads(ujson.dumps(init_state))

        self.__state_lock = threading.Lock()
        self.__client_lock = threading.Lock()

        self.__logger = rclpy.logging.get_logger('AppState')

    def add_client(self, client_id):
        with self.__client_lock:
            self.__client_ids[client_id] = None

    def create_services(self):
        self.__route = f'/{self.__app_name}'
        ws_srv_name = f'/ws{self.__route}'
        pub_name = f'{ws_srv_name}/pub'
        sub_name = f'{ws_srv_name}/sub'

        self.__ws_srv = self.__ros_node.create_service(HTTP, ws_srv_name, self.__request_handler)
        self.__publisher = self.__ros_node.create_publisher(WSMsg, pub_name, 10)
        self.__subscription = self.__ros_node.create_subscription(WSMsg, sub_name, self.__message_handler, 10)

    def destroy_services(self):
        self.__ros_node.destroy_service(self.__ws_srv)
        self.__ros_node.destroy_publisher(self.__publisher)
        self.__ros_node.destroy_subscription(self.__subscription)

    def __request_handler(self, request: HTTP.Request, response: HTTP.Response):
        self.__logger.debug('[{}]:WS REQ: {}'.format(self.__app_name, request.path))
        query = dict(urllib.parse.parse_qsl(request.query))
        client_id = query.get('clientId')
        if client_id in self.__client_ids:
            return response
        else:
            response.status = HTTPStatusCode.HTTP_UNAUTHORIZED
        return response

    def __message_handler(self, msg: WSMsg):
        self.__logger.debug('[{}]:MSG: {}'.format(self.__app_name, msg))

        if msg.type == WSMsgType.TEXT:
            json_data = msg.data.str
            ws_id = msg.ws_id

            try:
                data = ujson.loads(json_data)
                operation = data.get('operation')
                client_id = data.get('clientId')
                # self.__logger.info("client_id: {}".format(client_id))

                # Check the client id.
                if client_id is None or client_id not in self.__client_ids.keys():
                    self.__close_connection(ws_id)
                    return

                if operation == 'open':
                    with self.__client_lock:
                        old_ws_id = self.__client_ids.get(client_id)
                        if old_ws_id is not None:
                            self.__close_connection(old_ws_id)
                        self.__client_ids[client_id] = ws_id

                    event = AppEvent(type='open', client_id=client_id, state=self.state)
                    self.__call_event_handler('on_open', args=[event])

                elif operation == 'emit':
                    event = data.get('event')
                    self.__emit(client_id, event)

            except ValueError as e:
                self.__logger.error(e)

        elif msg.type == WSMsgType.CLOSE:
            client_id = None
            for key, value in self.__client_ids.items():
                if value == msg.ws_id:
                    client_id = key
                    break

            if client_id is None:
                self.__logger.debug('The client id does not exist.')
                return

            with self.__client_lock:
                del self.__client_ids[client_id]

            event = AppEvent(type='close', client_id=client_id, state=self.state)
            self.__call_event_handler('on_close', args=[event])

    def __emit(self, client_id: str, event: Dict):
        event_id = event.get('id')
        event_value = event.get("value")
        event_type = event.get("type")
        event = AppEvent(type=event_type, value=event_value, state=self.state, client_id=client_id)
        try:
            m = re.match(r'^\$\$\{(.+)}$', event_id)
            event_id = m.group(1) if m is not None else '__invalid_event_id__'
            handler = self.__handlers.get(event_id)
            if handler is not None:
                handler(event)
        except Exception as e:
            self.__logger.error('event[{}]: {}'.format(event_id, e))

    def __close_connection(self, ws_id: str):
        msg = WSMsg()
        msg.type = WSMsgType.CLOSE
        msg.ws_id = ws_id
        msg.route = self.__route
        self.__publisher.publish(msg)

    def __call_event_handler(self, event_type: str, args: List = None, kwargs: Dict = None):
        args = args or []
        kwargs = kwargs or {}
        handler = self.__event_handler_dict.get(event_type)
        res = None
        if handler is not None:
            try:
                res = handler(*args, **kwargs)
            except Exception as e:
                self.__logger.error(e)
        return res

    def __update_state(self, state: Dict):
        event = {
            'operation': 'update',
            'updateState': state,
        }
        json_str = ujson.dumps(event)
        msg = WSMsg()
        msg.data.str = json_str
        msg.type = WSMsgType.TEXT
        msg.route = self.__route
        self.__publisher.publish(msg)

    @property
    def app_name(self):
        return self.__app_name

    @property
    def state(self):
        return MappingProxyType(self.__state)

    @state.setter
    def state(self, state: Dict):
        update_state = {}
        update_handler = {}

        # Remove the handlers from the update state.
        for k, v in state.items():
            # Only the keys that were set in the constructor can be updated.
            if k in self.__state_keys:
                if callable(v):
                    update_handler[k] = v
                else:
                    update_state[k] = v

        _state = ujson.loads(ujson.dumps(update_state))
        with self.__state_lock:
            for k, v in update_handler.items():
                self.__handlers[k] = v
            for k, v in _state.items():
                self.__state[k] = v

        self.__update_state(update_state)

    @property
    def on_open(self):
        return self.__event_handler_dict.get('on_open')

    @on_open.setter
    def on_open(self, func: Callable):
        if callable(func):
            self.__event_handler_dict['on_open'] = func

    @property
    def on_close(self):
        return self.__event_handler_dict.get('on_close')

    @on_close.setter
    def on_close(self, func: Callable):
        if callable(func):
            self.__event_handler_dict['on_close'] = func
