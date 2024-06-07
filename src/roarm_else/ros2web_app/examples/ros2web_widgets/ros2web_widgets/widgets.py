import rclpy
import rclpy.logging
from std_msgs.msg import String

from ros2web_app.api import AppBase, AppEvent

NODE_NAME = 'widgets'


class App(AppBase):
    def __init__(self, app_name) -> None:
        init_state = {
            'on_click_button': self.on_click_button,
            'on_change_joystick': self.on_change_joystick,
            'on_select_list': self.on_select_list,
            'on_change_slider': self.on_change_slider,
            'on_change_switch': self.on_change_switch,
            'on_change_input': self.on_change_input,
            'on_change_input_group': self.on_change_input_group,
            'list_labels': ["item1", "item2", "item3", "item4", "item5"],
            'list_selected': 2,
            'slider_value': 10,
            'switch_value': False,
            'input_value': 'Hello, World!',
            'on_click_input': self.on_click_input,
            'input_group_items': [
                {'label': 'label1', 'value': 'Hello', 'type': 'text'},
                {'label': 'label2', 'value': 10, 'type': 'number'},
                {'label': 'label3', 'value': '', 'type': 'text'},
                {'label': 'label4', 'value': '', 'type': 'text'},
                {'label': 'label5', 'value': '', 'type': 'text'},
                {'label': 'label6', 'value': '', 'type': 'text'},
            ],
            'on_click_input_group': self.on_click_input_group,
        }
        super().__init__(app_name=app_name, init_state=init_state, config='config.yml')
        self.__logger = rclpy.logging.get_logger(NODE_NAME)

    def start(self):
        super().start()

    def shutdown(self):
        super().shutdown()

    def on_click_button(self, event: AppEvent):
        self.__logger.info(f"on_click_button")

    def on_change_joystick(self, event: AppEvent):
        self.__logger.info(f"on_change_joystick: {event.value}")

    def on_select_list(self, event: AppEvent):
        self.set_state({'list_selected': event.value})
        self.__logger.info(f"on_select_list: {event.value}")

    def on_change_slider(self, event: AppEvent):
        self.set_state({'slider_value': event.value})
        self.__logger.info(f"on_change_slider[{event.type}]: {event.value}")

    def on_change_switch(self, event: AppEvent):
        self.set_state({'switch_value': event.value})
        self.__logger.info(f"on_change_switch: {event.value}")

    def on_change_input(self, event: AppEvent):
        self.set_state({'input_value': event.value})
        self.__logger.info(f"on_change_input[{event.type}]: {event.value}")

    def on_click_input(self, event: AppEvent):
        self.set_state({'input_value': ""})
        self.__logger.info(f"on_click_input[{event.type}]: {event.value}")

    def on_change_input_group(self, event: AppEvent):
        self.set_state({'input_group_items': event.value})
        self.__logger.info(f"on_change_input_group[{event.type}]: {event.value}")

    def on_click_input_group(self, event: AppEvent):
        self.__logger.info(f"on_click_input_group[{event.type}]: {event.value}")


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    app.start()
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass

    app.shutdown()
