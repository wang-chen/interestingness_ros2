'''Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.'''


# ROS2 imports 
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
# sys
import os
import sys
# lib
from interface.msg import InterestInfo, UnInterests
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_prefix

share_path=get_package_prefix('interestingness_ros2')
pack_path=os.path.join(share_path[0:-28],'src','interestingness_ros2')

sys.path.append(os.path.join(pack_path))
sys.path.append(os.path.join(pack_path,'interestingness'))
from interestingness.online import level_height

import cv2
import math


class MarkerNode(Node):

    def __init__(self):
        super().__init__('InterestingnessNode')
        #declare params
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('min-level', None),
                ]
            )
        #config params
        self.config()
        
        # Create a subscriber to the InteresInfo topic
        self.subscription = self.create_subscription(InterestInfo, 'interestingness/info', self.info_callback, 10)
        self.subscription  # prevent unused variable warning

        # Create a topic to publish interestingness level
        self.marker_publisher = self.create_publisher(Marker, 'interestmarker/marker', 10)
      
    def config(self):
        self.min_level = self.get_parameter("min-level").value
        self.id=1

    def info_callback(self, msg):
        level = level_height(msg.level)
        if level < self.min_level:
           self.get_logger().info('Skip interests with level: {}'.format(level))
           return
        marker = Marker()
        marker.id = self.id
        self.id+=1
        marker.header = msg.header
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.color.a = level
        marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        marker.scale.x, marker.scale.y, marker.scale.z = [4*level]*3

        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 3.0
        marker.lifetime = Duration(seconds=9999,nanoseconds=9999).to_msg()
        self.marker_publisher.publish(marker)
        self.get_logger().info('Sent interests with level: {}.'.format(level))

def main(args=None):
    rclpy.init(args=args)
    marker = MarkerNode()
    rclpy.spin(marker)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    marker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        


