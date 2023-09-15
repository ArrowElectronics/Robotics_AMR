# Copyright (c) 2022 Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, “Robot Operating System 2: Design, architecture, and uses in the wild,” Science Robotics vol. 7, May 2022.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#===========================================================================================================
# Copyright (c) 2007 Open Robotics
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# ===========================================================================================================
# NOTICE: e-Infochips Private Limited developed code based on the ROS2 package.  

# Copyright (c) 2023 e-Infochips Private Limited
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:#
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation #and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software #without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ”AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE #IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE #LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS #OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT #LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH #DAMAGE.
#If any term or provision set forth herein is deemed to be invalid, illegal, or unenforceable in any jurisdiction, such invalidity, illegality, or unenforceability will not affect any other term or provision or invalidate or render unenforceable such term or provision in any other jurisdiction. Upon a court determination that any term or provision is invalid, illegal, or unenforceable, the court may modify these terms and conditions to affect our original intent as closely as possible in order that the transactions contemplated hereby be consummated to the greatest extent possible as originally contemplated.
#=============================================================================================================

#!/usr/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import time


class image_syncnode(Node):

    def __init__(self):

        super().__init__('image_sync_node')

        self.sub_caminfo = self.create_subscription(CameraInfo,
                '/aditof_roscpp/camera_info', self.caminfo_callback, 10)
        self.sub_caminfo  # prevent unused variable warning
        self.pub_caminfo = self.create_publisher(CameraInfo,
                '/cam1/camera_info_sync', 10)

        self.sub_depthimage = self.create_subscription(Image,
                '/cam1/depth_image_rect', self.depth_callback, 10)
        self.sub_depthimage  # prevent unused variable warning
        self.pub_depthimage = self.create_publisher(Image,
                '/cam1/depth_image_sync', 10)

        self.sub_irimage = self.create_subscription(Image,
                '/cam1/ir_image_rect', self.ir_callback, 10)
        self.sub_irimage  # prevent unused variable warning
        self.pub_irimage = self.create_publisher(Image,
                '/cam1/ir_image_sync', 10)


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
    image_sync_node = image_syncnode()
    rclpy.spin(image_sync_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    image_sync_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()

