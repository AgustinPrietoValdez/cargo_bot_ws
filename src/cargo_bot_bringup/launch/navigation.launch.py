import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = get_package_share_directory('cargo_bot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar /clock de Isaac')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
        description='Ruta al nav2_params.yaml')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_share, 'maps', 'cuarto_v1.yaml'),
        description='Ruta al mapa')

    # Nodos que maneja el lifecycle_manager de LOCALIZACIÓN
    localization_nodes = ['map_server', 'amcl']

    # Nodos que maneja el lifecycle_manager de NAVEGACIÓN
    # ⚠️ el ORDEN importa: controller primero, bt_navigator después de sus servers
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    # ---------- LOCALIZACIÓN ----------
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}])

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': localization_nodes}])

    # ---------- NAVEGACIÓN ----------
    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')])   # 👈 salida del controller → twist_mux

    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': navigation_nodes}])

    # ---------- TWIST MUX ----------
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel')])  # 👈 salida del mux → Isaac

    return LaunchDescription([
        declare_use_sim_time, declare_params, declare_map,
        map_server, amcl, lifecycle_localization,
        controller_server, planner_server, behavior_server,
        bt_navigator, waypoint_follower, lifecycle_navigation,
        twist_mux,
    ])