# This launch file will generate map using rgb,depth and camera info and 
# Odometry as visual odometry gererated using rgb and depth images

#Requirements:
# RGB, Depth and Camera info topics from  intel realsense camera 
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    lifecycle_nodes = ['map_server']

    lifecycle_node = Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_map', output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}],
                
    )


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
        #remappings=remappings
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

        # Compute quaternion of the IMU
    imu_madgwick_node = Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu')]
    )

    base_link_tf_node = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','-1.57','0','-1.57','base_link','camera_link_optical'],

    )

    #Navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        #launch_arguments={'params_file': /home/admin/data/ros2_ws/src/eic_bringup_utils/config/nav2_params_RPP.yaml}
        launch_arguments={'params_file': os.path.join(
            get_package_share_directory(
                'eic_bringup_utils'), 'config', 'nav2_params_RPP.yaml')}.items()        
    )

    return LaunchDescription([
        # Nodes to launch
        DeclareLaunchArgument('qos', default_value='2',description='QoS used for input sensor topics'),
        rtabmap_node,
        rtabmap_viz_node,
        #imu_madgwick_node,
        pointcloud_xyz_node,
        #nav2_launch,
        icp_odometry_node,
	base_link_tf_node,

    ])
