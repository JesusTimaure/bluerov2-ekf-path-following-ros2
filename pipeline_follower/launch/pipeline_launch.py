from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bluerov2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bluerov2_description'),
                'launch',
                'world_launch.py'
            )
        ),
        launch_arguments={
            'gui': 'true',
            'spawn': 'true',
            'use_sim_time': 'true'
        }.items()
    )
    
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bluerov2_control'),
                'launch',
                'rviz_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    
    controller_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('bluerov2_control'),
                        'launch',
                        'cascaded_pids_launch.py'
                    )
                ),
                launch_arguments={
                'namespace': 'bluerov2',
                'sliders': 'false',
                'rviz': 'false',
                'use_sim_time': 'true'
            }.items()
            )
        ]
    )
    
    ekf_config = os.path.join(
        get_package_share_directory('pipeline_follower'),  
        'config',
        'ekf.yaml'   
    )
    ekf_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config, {'use_sim_time': True}],
                remappings=[
                ('/odometry/filtered', '/bluerov2/ekf_odom')
                ]
            )
        ]
    )
    
    imu_cov_node = TimerAction(
        period=5.0,
        actions=[        
            Node(
                package='pipeline_follower',  # Assuming the node is in this package
                executable='imu_covariance_wrapper',  # Use your actual executable name here
                name='imu_covariance_wrapper',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    pseudo_dvl_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pipeline_follower',
                executable='pseudo_dvl',
                name='pseudo_dvl_node',
                namespace='/bluerov2',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    pseudo_depth_node= TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pipeline_follower',
                executable='pseudo_depth',
                name='pseudo_depth_node',
                namespace='/bluerov2',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    landmark_sim_node= TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pipeline_follower',
                executable='landmark_simulator',
                name='acoustic_positioning_node',
                namespace='/bluerov2',
                output='screen',
                parameters=[{'use_sim_time': True},
                    {'update_rate_hz': 1.0},
                    {'noise_std_dev': 0.05},
                    {'delay_sec': 0.2}]
            )
        ]
    )
    
    trajectory_gen_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pipeline_follower',
                executable='trajectory_gen',
                name='interpolated_trajectory_publisher',
                namespace='/bluerov2',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )    
    
    

    '''
    trajectory_gen_node = Node(
                package='pipeline_follower',
                executable='trajectory_gen',
                name='interpolated_trajectory_publisher',
                namespace='/bluerov2',
                output='screen',
                parameters=[{'use_sim_time': True}]
    ) 
    ''' 
    
    path_follower_node= TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pipeline_follower',
                executable='path_follower',
                name='path_follower_node',
                namespace='/bluerov2',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )    
    
    return LaunchDescription([
        SetLaunchConfiguration('use_sim_time', 'true'),
        bluerov2_launch,
        rviz_launch,
        controller_launch,
        ekf_node,
        imu_cov_node,
        pseudo_dvl_node,
        pseudo_depth_node,
        landmark_sim_node,
        trajectory_gen_node,
        path_follower_node
    ])
        
