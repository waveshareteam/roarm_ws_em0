from typing import List

from ros2web_interfaces.srv import HTTP
from ros2web_interfaces.msg import ContentType, BodyPart

import rclpy
from rclpy.node import Node

import urllib.parse
import json


class WebHandler(Node):

    def __init__(self):
        super().__init__('ros2web_example_py')
        self.get_srv = self.create_service(HTTP, 'http/get/add_three_ints', self.__get_callback)
        self.post_srv = self.create_service(HTTP, 'http/post/add_three_ints', self.__post_callback)

    def __get_callback(self, request: HTTP.Request, response: HTTP.Response):
        query = dict(urllib.parse.parse_qsl(request.query))

        a = int(query.get('a', 0))
        b = int(query.get('b', 0))
        c = int(query.get('c', 0))
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (a, b, c))
        sum_num = a + b + c
        response.text = json.dumps({'sum': sum_num})

        return response

    def __post_callback(self, request: HTTP.Request, response: HTTP.Response):
        query = {}
        if request.content_type == ContentType.APPLICATION_X_WWW_FORM_URLENCODED:
            body_parts: List[BodyPart] = request.multipart
            for part in body_parts:
                query[part.name] = int(bytearray(list(part.data)))

        a = query.get('a', 0)
        b = query.get('b', 0)
        self.get_logger().info('Incoming request\na: %d b: %d' % (a, b))
        sum_num = a + b
        response.text = json.dumps({'sum': sum_num})

        return response


def main(args=None):
    rclpy.init(args=args)
    web_handler = WebHandler()
    try:
        rclpy.spin(web_handler)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
