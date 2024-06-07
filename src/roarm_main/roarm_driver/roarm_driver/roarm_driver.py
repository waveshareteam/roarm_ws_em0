import rclpy
from rclpy.node import Node
import json
import serial
from serial import SerialException
import time
from sensor_msgs.msg import JointState

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('roarm_driver')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(serial_port_name, baud_rate)
            self.get_logger().info(f"{serial_port_name}，{baud_rate}。")
        except SerialException as e:
            self.get_logger().error(f" {serial_port_name}：{e}")
            return

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):

        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }
        
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        effort = msg.effort

        base = position[name.index('base_link_to_link1')]
        shoulder = -position[name.index('link1_to_link2')]
        elbow = position[name.index('link2_to_link3')]
        hand =  3.1415926 - position[name.index('link3_to_link4')]

        data = json.dumps({
            'T': 102, 
            'base': base, 
            'shoulder': shoulder, 
            'elbow': elbow, 
            'hand': hand, 
            'spd': 0,
            'acc': 10
        }) + "\n"
        
        try:
            self.serial_port.write(data.encode())

            time.sleep(0.05)
            self.get_logger().info(f"{data}")
        except SerialException as e:
            self.get_logger().error(f"{e}")

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    if minimal_subscriber.serial_port.is_open:
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        minimal_subscriber.serial_port.close()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
