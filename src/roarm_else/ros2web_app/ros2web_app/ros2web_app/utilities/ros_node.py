from rclpy.node import Node
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names


def check_duplicates(check_name: str, ros_node: Node = None) -> bool:
    if ros_node is None:
        with NodeStrategy(None) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=False)
    else:
        node_names = get_node_names(node=ros_node, include_hidden_nodes=False)

    names = [node_name.name for node_name in node_names]
    count = names.count(check_name)

    return count > 1

