import rclpy
import rclpy.logging

from ros2web_app.api import AppBase, AppEvent
NODE_NAME = 'app'


class App(AppBase):

    def __init__(self, app_name) -> None:
        init_state = {
            'on_click': self.on_click,
            'on_change': self.on_change,
            'on_select': self.on_select,
            'labels': ['label1', 'label2', 'label3'],
        }
        super().__init__(app_name=app_name, init_state=init_state, config='config.yml')
        self.__logger = rclpy.logging.get_logger("app")

    # def get_data_before_update(self, update_state: Dict, prev_state: Dict, state_id: str) -> Dict:
    #     return update_state

    # def get_state_before_create(self, *, init_state: Dict, query: Dict) -> Optional[Dict]:
    #     return None
    def on_click(self, event: AppEvent):
        self.__logger.info("on_click")

    def on_change(self, event: AppEvent):
        self.__logger.info("on_change {}".format(event.value))

    def on_select(self, event: AppEvent):
        self.__logger.info("on_select {}".format(event.value))

    def on_open(self, event: AppEvent):
        self.__logger.info("on_open")

    def on_close(self, event: AppEvent):
        self.__logger.info("on_close")


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    app.start()

    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass

    app.shutdown()
