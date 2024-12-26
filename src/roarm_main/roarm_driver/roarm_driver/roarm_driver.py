import rclpy
from rclpy.node import Node
import json
import serial
from serial import SerialException
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import queue
import threading
import logging
import time
import math

serial_port = "/dev/ttyUSB0"

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    def clear_buffer(self):
        self.s.reset_input_buffer()

class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger('BaseController')
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)
        self.rl = ReadLine(self.ser)
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        self.base_data = {"T": 1051, "x": 0, "y": 0, "z": 0, "b": 0, "s": 0, "e": 0, "t": 0, "torB": 0, "torS": 0, "torE": 0, "torH": 0}
        
    def feedback_data(self):
        try:
            line = self.rl.readline().decode('utf-8')
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            return self.base_data
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()

    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))
        return data_read

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)
        
class RoarmDriver(Node):

    def __init__(self):
        super().__init__('roarm_driver')
        
        self.declare_parameter('serial_port', serial_port)
        self.declare_parameter('baud_rate', 115200)
        
        serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.serial_port = serial.Serial(serial_port_name, baud_rate)
            self.get_logger().info(f"{serial_port_name},{baud_rate}.")
            
            start_data = json.dumps({'T': 605, "cmd": 0}) + "\n"
            self.serial_port.write(start_data.encode())
            time.sleep(0.1)
            
        except SerialException as e:
            self.get_logger().error(f"{serial_port_name}：{e}")
            return

        self.joint_states_sub = self.create_subscription(JointState, 'joint_states', self.joint_states_callback,10)  
        self.pose_sub = self.create_subscription(Pose, 'hand_pose', self.pose_callback,10)
        self.led_ctrl_sub = self.create_subscription(Float32, 'led_ctrl', self.led_ctrl_callback, 10)  
                             
    def joint_states_callback(self, msg):

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

        base = -position[name.index('base_link_to_link1')]
        shoulder = -position[name.index('link1_to_link2')]
        elbow = position[name.index('link2_to_link3')] 
        hand =  3.1415926 - position[name.index('link3_to_gripper_link')]

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
        except SerialException as e:
            self.get_logger().error(f"{e}")

    def pose_callback(self, msg):
        try:
            request_data = json.dumps({'T': 105}) + "\n"
            self.serial_port.write(request_data.encode())
            self.base_controller = BaseController(serial_port, 115200)
            time.sleep(0.1)
            self.base_controller.feedback_data()
            
            if self.base_controller.base_data["T"] == 1051:
               feedback = self.base_controller.base_data
               feedback['x'] /= 1000 
               feedback['y'] /= 1000
               feedback['z'] /= 1000  
               if float(feedback["x"]) != 0.0 or float(feedback["y"]) != 0.0 or float(feedback["z"]) != 0.0:
                  self.get_logger().info(f'Received feedback from serial port: {feedback}')
            time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f'Error communicating with serial port: {str(e)}')

    def led_ctrl_callback(self, msg):
        data = msg.data
        
        led_ctrl_data = json.dumps({
            'T': 114, 
            "led": data,
        }) + "\n"    
        self.serial_port.write(led_ctrl_data.encode())
                                
def main(args=None):
    rclpy.init(args=args)
    
    roarm_driver = RoarmDriver()
    
    if roarm_driver.serial_port.is_open:
        rclpy.spin(roarm_driver)
        roarm_driver.destroy_node()
        roarm_driver.serial_port.close()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()