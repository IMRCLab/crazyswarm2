#!/usr/bin/env python3
import socket,os,struct, time
import numpy as np
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ImageStreamerNode(Node):
    def __init__(self):
        super().__init__("image_node")

        # declare config path parameter
        self.declare_parameter(
            name="config_path",
            value=os.path.join(
                    get_package_share_directory('crazyflie'),
                    'config',
                    'aideck_streamer.yaml'
                )
        )

        config_path = self.get_parameter("config_path").value
        with open(config_path) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        # declare topic names
        self.declare_parameter(
            name="image_topic",
            value=config["image_topic"],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to publish to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value=config["camera_info_topic"],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        # declare aideck ip and port
        self.declare_parameter(
            name='deck_ip',
            value=config['deck_ip'],        
            )
        
        self.declare_parameter(
            name='deck_port',
            value=config['deck_port'],        
        )

        # declare image encoding with fallback to rgba8 (original behavior)
        self.declare_parameter(
            name="image_encoding",
            value=config.get("image_encoding", "rgba8"),
        )
        self.image_encoding = self.get_parameter("image_encoding").value

        # define variables from ros2 parameters
        image_topic = (
            self.get_parameter("image_topic").value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")


        # create messages and publishers
        self.image_msg = Image()
        self.camera_info_msg = self._construct_from_yaml(config)
        self.image_publisher = self.create_publisher(Image, image_topic, 10)
        self.info_publisher = self.create_publisher(CameraInfo, info_topic, 10)

        # set up connection to AI Deck
        deck_ip = self.get_parameter("deck_ip").value
        deck_port = int(self.get_parameter("deck_port").value)
        self.get_logger().info("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        self.get_logger().info("Socket connected")
        self.image = None
        self.rx_buffer = bytearray()

        # set up timers for callbacks
        timer_period = 0.01
        self.rx_timer = self.create_timer(timer_period, self.receive_callback)
        self.tx_timer = self.create_timer(timer_period, self.publish_callback)


    def _construct_from_yaml(self, config):
        camera_info = CameraInfo()

        camera_info.header.frame_id = config['camera_name']
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.width = int(config['image_width'])
        camera_info.height = int(config['image_height'])
        camera_info.distortion_model = config['distortion_model']
        camera_info.d = config['distortion_coefficients']['data']
        camera_info.k = config['camera_matrix']['data']
        camera_info.r = config['rectification_matrix']['data']
        camera_info.p = config['projection_matrix']['data']
        return camera_info

    def _rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data

    def receive_callback(self):
        # first get the info
        packetInfoRaw = self._rx_bytes(4)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

        # receive the header
        imgHeader = self._rx_bytes(length - 2)
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        # if magic is correct, get new image
        if magic == 0xBC:
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = self._rx_bytes(4)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = self._rx_bytes(length - 2)
                imgStream.extend(chunk)

            raw_img = np.frombuffer(imgStream, dtype=np.uint8)
            
            # Fix numpy shape requirement (height, width)
            raw_img = raw_img.reshape((height, width))

            if self.image_encoding == "mono8":
                self.image = raw_img
            else:
                self.image = cv2.cvtColor(raw_img, cv2.COLOR_BayerBG2RGBA)

    def publish_callback(self):
        if self.image is not None:
            self.image_msg.header.frame_id = self.camera_info_msg.header.frame_id
            self.image_msg.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_msg.header.stamp = self.image_msg.header.stamp
            
            # Safe shape unpacking for both 2D (mono) and 3D (color) arrays
            shape = self.image.shape
            height, width = shape[0], shape[1]
            channels = shape[2] if len(shape) > 2 else 1
            self.image_msg.height = height
            self.image_msg.width = width
            self.image_msg.encoding = self.image_encoding
            self.image_msg.step = width * channels   # number of bytes each row in the array will occupy
            self.image_msg.is_bigendian = 0 # TODO: implement automatic check depending on system
            self.image_msg.data = self.image.tobytes()

            self.image_publisher.publish(self.image_msg)
            self.info_publisher.publish(self.camera_info_msg)
            self.image = None
            
            

def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
