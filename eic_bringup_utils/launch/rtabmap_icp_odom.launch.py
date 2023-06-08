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


import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals




def generate_launch_description():
    qos = LaunchConfiguration('qos')
    
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    icp_odometry_node = Node(
            package='rtabmap_ros', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'subscribe_scan': False,
              'subscribe_scan_cloud': True,
              'qos':qos,
            }],
            remappings=[  
              ('scan_cloud', '/cloud')
            ],
            arguments=[
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/VoxelSize', '0.1',
              'Icp/Epsilon', '0.001',
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/MaxTranslation', '2',
              'Icp/MaxCorrespondenceDistance', '1',
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.7',
              'Icp/CorrespondenceRatio', '0.01',
              'Odom/ScanKeyFrameThr', '0.6',
              'OdomF2M/ScanSubtractRadius', '0.1',
              'OdomF2M/ScanMaxSize', '15000',
              'OdomF2M/BundleAdjustment', 'false',
    ])

    rtabmap_node = Node(
        package='rtabmap_ros', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id':'base_link',
            'subscribe_depth':False,
            'subscribe_rgb':False,
            'subscribe_scan':False,
            'subscribe_scan_cloud':True,
            'subscribe_rgbd':False,
            'approx_sync':True,
            'use_action_for_goal':True,

	    'Rtabmap/DetectionRate':'1',
            'RGBD/NeighborLinkRefining':'false',
            'RGBD/AngularUpdate':'5.0',
            'RGBD/LinearUpdate':'0.05',
            'RGBD/ProximityBySpace':'true',
            'RGBD/ProximityMaxGraphDepth':'0',
            'RGBD/ProximityPathMaxNeighbors':'1',
            'RGBD/LocalRadius':'5', #5
            'RGBD/ProximityPathFilteringRadius':'2.0',
            'RGBD/ProximityMaxPaths':'2',
            'RGBD/ProximityOdomGuess':'true',
            'RGBD/ProximityAngle':'45', #180
            'Kp/MaxFeatures':'-1',
            'Mem/NotLinkedNodesKept':'false',
            'Mem/STMSize':'10', #30
            'Reg/Strategy':'1',
            'Optimizer/GravitySigma':'0.3',

            # 'Grid/RangeMin':'0.0', # ignore laser scan points on the robot itself
            'Grid/RangeMax':'5.0', # ignore laser scan points on the robot itself
            'Grid/GroundIsObstacle':'false',
            'Grid/FrcharacteromDepth':'false',
            'Grid/RayTracing':'True',
            'Grid/3D':'false',
            'Grid/PreVoxelFiltering':'true',
            'FootprintLength':'0.81',
            'FootprintWidth':'0.635',
            'FootprintHeight':'0.7',
            'NormalK':'20',
            'MaxGroundAngle':'45',
            'MaxGroundHeight':'0.05',
            'Grid/ClusterRadius':'1',
            'MinClusterSize':'30',
            'FlatObstacleDtected':'true',
            'MaxObstacleHeight':'2.0',

            'Grid/NoiseFilteringRadius':'0.07', #0
            'Grid/NoiseFilteringMinNeighbors':'10',

            'qos_scan':qos,
            'qos_imu':qos,
            # 'filter_nans':True, 
            'queue_size':100,
            # Disable imu constraints (we are already in 2D)
            'Reg/Force3DoF':'true',
            'Icp/Strategy':'1',
            'Icp/VoxelSize':'0.05',
            'Icp/PointToPlaneK':'20',
            'Icp/PointToPlaneRadius':'0',
            'Icp/PointToPlane':'true',
            'Icp/Iterations':'50', #100
            'Icp/Epsilon':'0.001',
            'Icp/MaxTranslation':'2.0',
            'Icp/MaxCorrespondenceDistance':'1.5',
            'Icp/PMOutlierRatio':'0.8', #0.7
            'Icp/CorrespondenceRatio':'0.4',
            'Icp/PointToPlaneGroundNormalsUp':'0.8',
            'Icp/RangeMin':'0.3',
            'Icp/RangeMax':'100',


          }],
        remappings=[
            ('scan_cloud', '/cloud'),
            ('odom', '/odom'),
            ],
        arguments=['-d']
    )

    rtabmap_viz_node = Node(
        package='rtabmap_ros', executable='rtabmapviz', output='screen',
        parameters=[{
            'frame_id':'base_link',
            'subscribe_depth': False,
            'approx_sync': True,
            'map_always_update': False,
            'gen_depth_fill_holes_size': 200,  
          }],
        remappings=[
            ('scan_cloud', '/cloud'),
            ('odom', '/odom'),
            ],
    )

    pointcloud_xyz_node = Node(
        package='rtabmap_ros', executable='point_cloud_xyz', output='screen',

        parameters=[{
                'subscribe_depth': True,
                'approx_sync': True,
                'filter_nans':True,
                'decimation':2,
                'min_depth':0.50,
                'max_depth':5.0,
                'roi_ratios':'0.0 0.0 0.2 0.1',
        }],

        remappings=[
            ('depth/camera_info', '/cam1/camera_info_sync'),
            ('depth/image', '/cam1/depth_image_sync'),
        ]
    )


    base_link_tf_node = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','-1.57','0','-1.57','base_link','camera_link_optical'],

    )


    return LaunchDescription([
        # Nodes to launch
        DeclareLaunchArgument('qos', default_value='2',description='QoS used for input sensor topics'),
        rtabmap_node,
        rtabmap_viz_node,
        pointcloud_xyz_node,
        icp_odometry_node,
	base_link_tf_node,

    ])
