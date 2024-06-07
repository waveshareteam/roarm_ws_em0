import rclpy.node
from ros2node.api import get_publisher_info, get_subscriber_info, get_service_server_info


def pub_topic_exists(*, topic_name: str, remote_node_name: str, ros_node: rclpy.node.Node) -> bool:
    topic_names_and_types = get_publisher_info(
        node=ros_node, remote_node_name=remote_node_name)
    topic_names = [topic_name
                   for (topic_name, topic_types) in topic_names_and_types
                   if 'ros2web_interfaces/msg/WSMsg' in topic_types]

    return topic_name in topic_names


def sub_topic_exists(*, topic_name: str, remote_node_name: str, ros_node: rclpy.node.Node) -> bool:
    topic_names_and_types = get_subscriber_info(
        node=ros_node, remote_node_name=remote_node_name)
    topic_names = [topic_name
                   for (topic_name, topic_types) in topic_names_and_types
                   if 'ros2web_interfaces/msg/WSMsg' in topic_types]

    return topic_name in topic_names


def service_exists(*, srv_name: str, remote_node_name: str, ros_node: rclpy.node.Node) -> bool:
    service_names_and_types = get_service_server_info(
        node=ros_node, remote_node_name=remote_node_name)

    srv_names = [srv_name
                 for (srv_name, srv_types) in service_names_and_types
                 if 'ros2web_interfaces/srv/HTTP' in srv_types]

    return srv_name in srv_names
