from typing import List

import re
import importlib.resources
import os.path

from ros2web_interfaces.srv import HTTP
from ros2web_interfaces.msg import ContentType, BodyPart, HTTPStatusCode

import rclpy
from rclpy.node import Node


class WebHandler(Node):
    
    def __init__(self):
        super().__init__('ros2web_example_py')
        self.get_srv = self.create_service(
            HTTP, 'http/get', self.__index_callback)
        self.post_srv = self.create_service(
            HTTP, 'http/post/upload', self.__post_callback)
        
        self.__logger = self.get_logger()

    def __index_callback(self, request: HTTP.Request, response: HTTP.Response):
        headers = dict(zip(request.headers[0::2], request.headers[1::2]))
        # self.__logger.info(f"{request.path}, {headers}")

        path = request.path.replace(request.route, '')
        path_match = re.match(r'/(?P<file_name>.+\.\w+)$', path)

        file_name = ''
        if path == '' or path == '/':
            file_name = 'index.html'
        elif path_match is not None:
            file_name = path_match.group('file_name')
        else:
            response.status = HTTPStatusCode.HTTP_NOT_FOUND
            return response

        with importlib.resources.path('ros2web_example_py', 'data') as path:
            index_file_path = path.joinpath(file_name)

        if not os.path.exists(index_file_path):
            self.__logger.error(f"File does not exist. ({index_file_path})")
            response.status = HTTPStatusCode.HTTP_NOT_FOUND

        response.file_path = str(index_file_path)
        return response

    def __post_callback(self, request: HTTP.Request, response: HTTP.Response):
        label = ''
        filename = ''
        tmpl = """
            <html>
                <body>
                    <p>{}</p>
                    <img src="/{}">
                </body>
            </html>
            """
            
        if len(request.multipart) > 0:
            body_parts: List[BodyPart] = request.multipart
            for part in body_parts:
                if part.name == 'label':
                    if part.content_type == ContentType.TEXT_PLAIN:
                        label = bytearray(list(part.data)).decode('utf-8')
                elif part.name == 'image':
                    filename = part.filename
                    if part.content_type == ContentType.IMAGE_JPEG:
                        content = bytearray(list(part.data))
                        with importlib.resources.path('ros2web_example_py', 'data') as path:
                            index_file_path = path.joinpath(filename)
                            with open(index_file_path, 'wb') as f:
                                f.write(content)

        response.content_type = ContentType.TEXT_HTML
        response.text = tmpl.format(label, filename)

        return response


def main(args=None):
    rclpy.init(args=args)
    web_handler = WebHandler()

    try:
        rclpy.spin(web_handler)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
