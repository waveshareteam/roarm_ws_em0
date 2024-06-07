import rclpy
import rclpy.logging

from ros2web_app.api import AppBase, AppEvent
from std_msgs.msg import String

NODE_NAME = '@package_name'


class App(AppBase):

    def __init__(self, app_name) -> None:
        init_state = {
            'on_click': self.on_click,
            'label': 'Hello World: 0',
        }
        super().__init__(app_name=app_name, init_state=init_state, config='config.yml')
        self.__logger = rclpy.logging.get_logger(NODE_NAME)
        self.i = 0
        self.publisher = self.create_publisher(String, 'topic', 10)

    def on_click(self, event: AppEvent):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.i += 1
        self.publisher.publish(msg)
        self.set_state({'label': msg.data})

    def shutdown(self):
        self.destroy_publisher(self.publisher)
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    app.start()
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass
    app.shutdown()

if __name__ == '__main__':
    main()
