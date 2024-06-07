from typing import Dict, Set

import rclpy.logging
from aiohttp import web
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay

from ros2web.handler import HandlerExtension

from .track.image import ImageMessage


class WebRtcHandler(HandlerExtension):

    def __init__(self) -> None:
        super().__init__()

        self.__peer_connections: Set[RTCPeerConnection] = set()
        self.__relay = MediaRelay()
        self.__image_msg_dict: Dict[str, ImageMessage] = {}
        self.__logger = rclpy.logging.get_logger('WebRtc')

        self.__ros_node = None

    async def startup(self, ros_node: rclpy.node.Node):
        self.__ros_node = ros_node

    async def shutdown(self):
        for track in self.__image_msg_dict.values():
            track.close()

        tasks = [pc.close() for pc in self.__peer_connections]
        await asyncio.gather(*tasks)
        self.__peer_connections.clear()

    async def handle(self, request: web.Request) -> web.StreamResponse:
        self.__logger.info('REQ: {}'.format(request.path))

        if request.method == 'POST':
            if request.path == '/plugin/web_rtc/offer':
                return await self.__offer_handler(request)
        else:
            raise web.HTTPNotFound()

    async def __offer_handler(self, request: web.Request) -> web.StreamResponse:

        try:
            param = await request.json()
            sdp = param.get('sdp')
            topic = param.get('topic')
            rct_type = param.get('type')

            offer = RTCSessionDescription(sdp=sdp, type=rct_type)
            pc = RTCPeerConnection()
            self.__peer_connections.add(pc)

            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                self.__logger.info("Connection state is %s" % pc.connectionState)
                if pc.connectionState == "new":
                    pass
                elif pc.connectionState == "connecting":
                    pass
                elif pc.connectionState == "connected":
                    pass
                elif pc.connectionState == "closed":
                    pass
                elif pc.connectionState == "failed":
                    await pc.close()
                    self.__peer_connections.discard(pc)

            track = self.__get_image_track(topic)
            pc.addTrack(track)

            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)

        except Exception as e:
            self.__logger.error('request_handler: {}'.format(e))
            raise web.HTTPInternalServerError()

        res = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        return web.json_response(res)

    # async def __on_peer_connection_closed(self, pc: RTCPeerConnection):
    #     if pc in self.__peer_connections:
    #         self.__peer_connections.discard(pc)

    def __get_image_track(self, topic_name):
        if self.__ros_node is None:
            raise RuntimeError('ROS node is None')

        image_msg = self.__image_msg_dict.get(topic_name)
        if image_msg is None:
            image_msg = ImageMessage(self.__ros_node, topic_name)
            self.__image_msg_dict[topic_name] = image_msg

        return self.__relay.subscribe(image_msg.track)
