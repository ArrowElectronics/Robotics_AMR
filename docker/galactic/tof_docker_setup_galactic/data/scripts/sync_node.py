#!/usr/bin/python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import time


class Image_SyncNode(Node):

    def __init__(self):

        super().__init__('image_sync_node')

        self.sub_caminfo = self.create_subscription(CameraInfo,
                '/cam1/camera_info', self.caminfo_callback, 10)
        self.sub_caminfo  # prevent unused variable warning
        self.pub_caminfo = self.create_publisher(CameraInfo,
                '/cam1/camera_info_sync', 10)

        self.sub_depthimage = self.create_subscription(Image,
                '/cam1/depth_image', self.depth_callback, 10)
        self.sub_depthimage  # prevent unused variable warning
        self.pub_depthimage = self.create_publisher(Image,
                '/cam1/depth_image_sync', 10)

        self.sub_irimage = self.create_subscription(Image,
                '/cam1/ir_image', self.ir_callback, 10)
        self.sub_irimage  # prevent unused variable warning
        self.pub_irimage = self.create_publisher(Image,
                '/cam1/ir_image_sync', 10)

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_pubcallback)
        # self.i = 0

    def caminfo_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id="camera_link_optical"
        self.pub_caminfo.publish(msg)

    def depth_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id="camera_link_optical"
        self.pub_depthimage.publish(msg)

    def ir_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id="camera_link_optical"
        self.pub_irimage.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    image_sync_node = Image_SyncNode()
    rclpy.spin(image_sync_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    image_sync_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()

