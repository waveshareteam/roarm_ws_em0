from typing import List
import math

import rclpy
import rclpy.logging
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from launch_ros.actions import Node as NodeAction
from ros2web_app.api import AppBase, AppEvent
from ros2web_app.utilities.ros import ParamService, Param, Service

from launch_api import LaunchAPI, ProcessEvent

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from roarm_moveit.srv import MovePointCmd, GetPoseCmd
NODE_NAME = 'roarm_web_app'

class App(AppBase):
    def __init__(self, app_name) -> None:
        init_state = {
            'move_point_cmd_started': False,
            'move_point_cmd_start_switch_disabled': False,
            'on_change_move_point_cmd_switch': self.on_change_move_point_cmd_on_off_switch,
            'get_pose_cmd_started': False,
            'get_pose_cmd_start_switch_disabled': False,
            'on_change_get_pose_cmd_switch': self.on_change_get_pose_cmd_on_off_switch,
            'on_change_joystick': self.on_change_joystick,
            'get_pose_name': '',
            'pose_list': ['pose'],
            'on_click_get_pose_button': self.on_get_pose_cmd,
            'on_click_move_point_button': self.on_move_point_cmd,
        }
        super().__init__(app_name=app_name, init_state=init_state, config='config.yml')
        self.__logger = rclpy.logging.get_logger(NODE_NAME)
        self.__launch_api = LaunchAPI()
        self.__move_point_process = None
        self.__get_pose_process = None

        self.__twist_publisher = None
        self.__param_srv = ParamService(self)
        self.__srv = Service(self)

    def start(self):
        super().start()
        self.__launch_api.start()
        self.__twist_publisher = self.create_publisher(Twist, '/webappcontrol', 10)

    def shutdown(self):
        self.destroy_publisher(self.__twist_publisher)
        self.__launch_api.shutdown()
        super().shutdown()

    def futures(self):
        self.__param_srv.futures()
        self.__srv.futures()

    def on_change_move_point_cmd_on_off_switch(self, event: AppEvent):
        # self.__logger.info(f"on_change_move_point_cmd_on_off_switch: {event.value}")

        if event.value:
            if self.__move_point_process is None:
                node_action = NodeAction(package='roarm_moveit_cmd', executable='movepointcmd')
                try:
                    self.__move_point_process = self.__launch_api.node(node_action)
                    self.__move_point_process.on_start = self.__on_start_move_point_node
                    self.__move_point_process.on_exit = self.__on_exit_move_point_node

                    self.set_state({'move_point_cmd_started': True, 'move_point_cmd_start_switch_disabled': False})
                except Exception as e:
                    self.__logger.error(f"Failed to start movepointcmd: {e}")
                    self.__move_point_process = None
                    self.set_state({'move_point_cmd_started': False, 'move_point_cmd_start_switch_disabled': False})
        else:
            if self.__move_point_process is not None:
                self.__move_point_process.shutdown()

    def __on_start_move_point_node(self, event: ProcessEvent):
        self.__logger.info(f"move_point_cmd_node started: {event.pid}")
                       
    def __on_exit_move_point_node(self, event: ProcessEvent):
        self.__logger.info(f"move_point_node exited: {event.pid}")
        self.__move_point_process = None
        self.set_state({'move_point_cmd_started': False, 'move_point_cmd_start_switch_disabled': False})

                
    def on_change_get_pose_cmd_on_off_switch(self, event: AppEvent):
        # self.__logger.info(f"on_change_get_pose_cmd_on_off_switch: {event.value}")

        if event.value:
            if self.__get_pose_process is None:
                node_action = NodeAction(package='roarm_moveit_cmd', executable='getposecmd')
                try:
                    self.__get_pose_process = self.__launch_api.node(node_action)
                    self.__get_pose_process.on_start = self.__on_start_get_pose_node
                    self.__get_pose_process.on_exit = self.__on_exit_get_pose_node

                    self.set_state({'get_pose_cmd_started': True, 'get_pose_cmd_start_switch_disabled': False})
                except Exception as e:
                    self.__logger.error(f"Failed to start movepointcmd: {e}")
                    self.__get_pose_process = None
                    self.set_state({'get_pose_cmd_started': False, 'get_pose_cmd_start_switch_disabled': False})
        else:
            if self.__get_pose_process is not None:
                self.__get_pose_process.shutdown()
                
    def __on_start_get_pose_node(self, event: ProcessEvent):
        self.__logger.info(f"get_pose_cmd_node started: {event.pid}")
                       
    def __on_exit_get_pose_node(self, event: ProcessEvent):
        self.__logger.info(f"get_pose_node exited: {event.pid}")
        self.__get_pose_process = None
        self.set_state({'get_pose_cmd_started': False, 'get_pose_cmd_start_switch_disabled': False})

    def on_change_joystick(self, event: AppEvent):
        if event.type == 'stop':
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.__twist_publisher.publish(twist)
        elif event.type == 'move':
            value = event.value
            if value is not None:
                x = value.get('x')
                y = value.get('y')
                twist = Twist()
                twist.linear.x = float(y) 
                twist.linear.y = float(x) 
                self.__twist_publisher.publish(twist)

    def on_get_pose_cmd(self, event: AppEvent):
        # self.__logger.info(f"on_get_pose_cmd: {event.value}")
        self.set_state({'get_pose_name': event.value})
        if event.value == '':
            return
        request = GetPoseCmd.Request()

        def callback(response: GetPoseCmd.Response):
            pose_x =  "x: " + str(response.x)
            pose_y =  "y: " + str(response.y)
            pose_z =  "z: " + str(response.z)
            if pose_x == "":
                self.set_state({'get_pose_name': ''})
            else:
                pose_list = self.get_state().get('pose_list') + [pose_x]+ [pose_y]+ [pose_z]
                self.set_state({'pose_list': pose_list, 'get_pose_name': ''})

        client = self.create_client(GetPoseCmd, '/get_pose_cmd')
        self.__srv.call(client, request, callback=callback)

    def on_move_point_cmd(self, event: AppEvent):
        # self.__logger.info(f"on_move_point_cmd: {event.value}")
        attr = {}
        for item in event.value:
            if item['value'] is None or item['value'] == '':
                continue
            attr[item['label']] = item['value']

        request = MovePointCmd.Request()
        request.x = float(attr.get('x', 0))
        request.y = float(attr.get('y', 0))
        request.z = float(attr.get('z', 0))

        client = self.create_client(MovePointCmd, '/move_point_cmd')
        self.__srv.call(client, request)


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    app.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(app)
            app.futures()
    except KeyboardInterrupt:
        pass
    app.shutdown()

