# This node gives feedback of the servo motors on the hardware and publishes it to a topic called 'feedback' as a Point data structure.

import rclpy
from rclpy.node import Node
import array
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import json
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)  # Added timeout for non-blocking read
global x, y, z, b, s, e, t, t_b, t_s, t_e, t_t, t1, t2, t3, l1, l2
x, y, z, b, s, e1, t, t_b, t_s, t_e, t_t, t1, t2, t3 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
l1 = math.sqrt(0.23682**2 + 0.03**2)
l2 = 0.21599

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('feedback')
        self.position = []
        self.publisher_ = self.create_publisher(Float64MultiArray, "feedback", 10)

        self.joint_data = Float64MultiArray()
        self.joint_data.data = [0.0] * 11

        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        else:
            getPos = int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 0.5)
            return getPos

    def timer_callback(self):
        global x, y, z, b, s, e1, t, t_b, t_s, t_e, t_t, t1, t2, t3, l1, l2

        # a = msg.position
        data = json.dumps({"T":105}) + "\n"
        ser.write(data.encode())

        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)  # Read all available data
            response_str = response.decode('utf-8')  # Decode bytes to string

            # Split the response by newline to handle multiple JSON objects
            responses = response_str.strip().split('\n')
            # print(responses)
            # Assuming the third response (responses[2]) contains the required dictionary
            for res in responses:
                try:
                    req_dictionary = json.loads(res)
                    b = req_dictionary.get("b", b)
                    s = req_dictionary.get("s", s)
                    e1 = req_dictionary.get("e", e1)

                    t1 = b
                    t2 = (math.pi/2) - (s + math.atan2(0.03, 0.23682))
                    t3 = e1 - math.atan2(0.03, 0.23682)

                    x = (l1 * math.cos(t2) + l2 * math.cos(t2 - t3)) * math.cos(t1)
                    y = (l1 * math.cos(t2) + l2 * math.cos(t2 - t3)) * math.sin(t1)
                    z = (l1 * math.sin(t2) + l2 * math.sin(t2 - t3)) + 0.03796

                    t = req_dictionary.get("t", t)
                    t_b = req_dictionary.get("torB", t_b)
                    t_s = req_dictionary.get("torS", t_s)
                    t_e = req_dictionary.get("torE", t_e)
                    t_t = req_dictionary.get("torH", t_t)

                    # print(f"Parsed JSON: {req_dictionary}")  # Debug: print each parsed dictionary
                except json.JSONDecodeError as e:
                    # print(f"Failed to decode JSON response: {e}")
                    pass
                except KeyError as e:
                    # print(f"Missing expected key in response: {e}")
                    pass

        else:
            print("No data available to read")
        self.joint_data.data[0] = float(x)
        self.joint_data.data[1] = float(y)
        self.joint_data.data[2] = float(z)
        self.joint_data.data[3] = float(b)
        self.joint_data.data[4] = float(s)
        self.joint_data.data[5] = float(e1)
        self.joint_data.data[6] = float(t)
        self.joint_data.data[7] = float(t_b)
        self.joint_data.data[8] = float(t_s)
        self.joint_data.data[9] = float(t_e)
        self.joint_data.data[10] = float(t_t)
        self.publisher_.publish(self.joint_data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    print("Starting spin")
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    print("Shut down")


if __name__ == '__main__':
    main()
