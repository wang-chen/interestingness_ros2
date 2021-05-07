'''Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.'''


# ROS2 imports 
import rclpy
from rclpy.node import Node

#path
import os
import sys
sys.path.append('./')

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String
from interface.msg import InterestInfo, UnInterests
from cv_bridge import CvBridge, CvBridgeError
import torchvision.transforms as transforms
from ament_index_python.packages import get_package_prefix

share_path=get_package_prefix('interestingness_ros2')
pack_path=os.path.join(share_path[0:-28],'src','interestingness_ros2')

sys.path.append(os.path.join(pack_path))
sys.path.append(os.path.join(pack_path,'interestingness'))
from interestingness.online import MovAvg, show_batch_box, level_height
from interestingness.interestingness import Interestingness
from interestingness.dataset import ImageData, Dronefilm, DroneFilming, SubT, SubTF, PersonalVideo
from interestingness.torchutil import VerticalFlip, count_parameters, show_batch, show_batch_origin, Timer, MovAvg
from interestingness.torchutil import ConvLoss, CosineLoss, CorrelationLoss, Split2d, Merge2d, PearsonLoss, FiveSplit2d

import cv2
import torch
import numpy as np
import PIL


class InterestingnessNode(Node):

    def __init__(self):
        super().__init__('InterestingnessNode')
        #declare params
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('image-topic', None),
                    ('interaction-topic', None),
                    ('data-root', None),
                    ('model-save', None),
                    ('crop-size', None),
                    ('skip-frames', None),
                    ('window-size', None),
                    ('save-flag', None),
                    ('rr', None),
                    ('wr', None),
                ]
            )
        #config params
        self.config()

        # internal params and settings
        self.movavg = MovAvg(self.window_size)
        self.transform= transforms.Compose([
        # VerticalFlip(), # Front camera of UGV0 in SubTF is mounted vertical flipped. Uncomment this line when needed.
        transforms.CenterCrop(self.get_parameter("crop-size").value),
        transforms.Resize((self.get_parameter("crop-size").value, self.get_parameter("crop-size").value)),
        transforms.ToTensor()])
        self.bridge = CvBridge()
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        
        # Create a subscriber to the Image topic
        for topic in self.image_topic:
            self.subscription = self.create_subscription(Image, topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # Create a topic to publish interestingness level
        self.level_publisher = self.create_publisher(InterestInfo, 'interestingness/info', 10)

        # Create an Image publisher for the results
        self.img_publisher = self.create_publisher(Image,'interestingness/image',10)
        
        # load net
        net = torch.load(self.model_save)
        net.set_train(False)
        net.memory.set_learning_rate(rr=self.rr, wr=self.wr)
        self.net = net.cuda() if torch.cuda.is_available() else net
      
    def config(self):
        self.rr, self.wr = self.get_parameter("rr").value, self.get_parameter("wr").value
        self.model_save = self.get_parameter("model-save").value
        self.image_topic = self.get_parameter("image-topic").value
        self.skip_frames = self.get_parameter("skip-frames").value
        self.window_size = self.get_parameter("window-size").value

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            frame = PIL.Image.fromarray(frame)
            image = self.transform(frame)
            frame = self.normalize(image).unsqueeze(dim=0)
        except CvBridgeError as e:
            print(e)
        else:
            self.get_logger().info("Received image %s"%(msg.header.frame_id))
            frame = frame.cuda() if torch.cuda.is_available() else frame
            loss = self.net(frame)
            loss = self.movavg.append(loss)
            frame = 255*show_batch_box(frame, 1, loss.item(),show_now=False)
            frame_msg = self.bridge.cv2_to_imgmsg(frame.astype(np.uint8))
            info = InterestInfo()
            info.level = loss.item()
            info.image_shape = image.shape
            info.image = image.view(-1).numpy().tolist()
            info.shape = self.net.states.shape
            info.feature = self.net.states.cpu().view(-1).numpy().tolist()
            info.memory = self.net.coding.cpu().view(-1).numpy().tolist()
            info.reading_weights = self.net.memory.rw.cpu().view(-1).numpy().tolist()
            info.header = frame_msg.header = msg.header
            frame_msg.header = msg.header
            self.img_publisher.publish(frame_msg)
            self.level_publisher.publish(info)
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            self.get_logger().info("Interest level: %5f"%(info.level))
        #     cv2.putText(cv_image, str(info.level),
        #                (20, 40),
        #                 cv2.FONT_HERSHEY_SIMPLEX,
        #                 1,  # font scale
        #                (255, 0, 255), 2)  # line type
        # # Displaying the predictions
        #     cv2.imshow('object_detection', cv_image)

        def interaction_callback(self, data):
            ''' To cooperate with interaction package
            '''
            self.get_logger().info('Received uninteresting feature maps %d'%(msg.header.seq))
            coding = msg_to_torch(msg.feature, msg.shape)
            coding = coding.cuda() if torch.cuda.is_available() else coding
            self.net.memory.write(coding)
        


