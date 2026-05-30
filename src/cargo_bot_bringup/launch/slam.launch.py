from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_config = PathJoinSubstitution([
        FindPackageShare('cargo_bot_navigation'),
        'config',
        'slam_toolbox.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('cargo_bot_description'),
        'rviz',
        'cargo_bot.rviz'
    ])

    # First, bring up the EKF localization (input to SLAM)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cargo_bot_bringup'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Isaac simulation clock'
        ),

        localization_launch,

        # Republish Isaac's /scan to /scan_fixed with consistent angle metadata so
        # slam_toolbox (Karto) stops rejecting every scan with
        # "LaserRangeScan contains 1066 range readings, expected 1067".
        # See cargo_bot_bringup/scan_angle_fixer.py for the full root cause.
        Node(
            package='cargo_bot_bringup',
            executable='scan_angle_fixer',
            name='scan_angle_fixer',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'input_topic': '/scan'},
                {'output_topic': '/scan_fixed'},
            ],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])