from av import VideoFrame
from aiortc import VideoStreamTrack, MediaStreamTrack
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image

import queue
from weakref import WeakSet


class ImageMessageTrack(VideoStreamTrack):

    def __init__(self, q):
        super().__init__()
        self.__queue: queue.Queue = q
        height, width = 480, 640
        self.__frame = VideoFrame(width, height, "rgb24")

    async def recv(self):
        frame = self.__frame
        try:
            frame = self.__queue.get_nowait()
            self.__frame = frame
        except queue.Empty:
            pass
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame


class ImageMessage:
    def __init__(self, ros_node: Node, topic_name: str):
        self.__logger = rclpy.logging.get_logger("ImageMessage")
        self.__bridge = CvBridge()
        self.__queues = WeakSet()

        self.__ros_node = ros_node
        self.__subscription = ros_node.create_subscription(
            Image,
            topic_name,
            self.__listener_callback,
            10)

    def __listener_callback(self, image: Image):
        try:
            cv_image: cv2.Mat = self.__bridge.imgmsg_to_cv2(image)
            frame = VideoFrame.from_ndarray(cv_image)
            for q in self.__queues:
                q.put(frame)
        except CvBridgeError as e:
            self.__logger.error(e)
        except Exception as e:
            self.__logger.error(e)

    def close(self):
        self.__ros_node.destroy_subscription(self.__subscription)

    @property
    def track(self) -> MediaStreamTrack:
        q = queue.Queue()
        self.__queues.add(q)
        track = ImageMessageTrack(q)
        return track
