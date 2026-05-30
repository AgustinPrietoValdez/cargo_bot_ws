from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path al xacro instalado por cargo_bot_description
    xacro_path = PathJoinSubstitution([
        FindPackageShare('cargo_bot_description'),
        'urdf',
        'cargo_bot.urdf.xacro'
    ])

    # robot_description = output de `xacro <xacro_path>` (string URDF puro).
    # ParameterValue(..., value_type=str) le dice a ROS 2 "es string, no parsees como YAML".
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # Path al ekf.yaml instalado por cargo_bot_navigation
    ekf_config = PathJoinSubstitution([
        FindPackageShare('cargo_bot_navigation'),
        'config',
        'ekf.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Isaac simulation clock (/clock topic)'
        ),

        # ── 1. robot_state_publisher ──
        # Lee robot_description, publica /tf_static (joints fixed: imu, lidar, caster)
        # y /tf (joints non-fixed: wheels, recalculadas a partir de /joint_states).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # ── 2. joint_state_publisher ──
        # Publica /joint_states con los wheels en pos=0. No giran visualmente
        # en RViz, pero SLAM/Nav2 no leen las wheels: usan base_footprint y
        # lidar_link (joints fixed), que rsp publica como /tf_static.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # ── 3. ekf_filter_node ──
        # name='ekf_filter_node' DEBE coincidir EXACTO con el primer key del
        # ekf.yaml. Si no, los params se ignoran silenciosamente y el filtro
        # arranca con defaults.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])