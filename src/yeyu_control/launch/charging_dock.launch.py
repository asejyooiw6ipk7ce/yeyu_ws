from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    battery_topic = LaunchConfiguration('battery_topic')

    target_marker_id = LaunchConfiguration('target_marker_id')
    marker_size_m = LaunchConfiguration('marker_size_m')

    pre_dock_distance_m = LaunchConfiguration('pre_dock_distance_m')
    final_dock_distance_m = LaunchConfiguration('final_dock_distance_m')
    lateral_tolerance_m = LaunchConfiguration('lateral_tolerance_m')
    bearing_tolerance_rad = LaunchConfiguration('bearing_tolerance_rad')
    k_z = LaunchConfiguration('k_z')
    k_lateral = LaunchConfiguration('k_lateral')
    k_bearing = LaunchConfiguration('k_bearing')
    search_angular_speed_rps = LaunchConfiguration('search_angular_speed_rps')
    marker_lost_timeout_sec = LaunchConfiguration('marker_lost_timeout_sec')
    stale_stop_timeout_sec = LaunchConfiguration('stale_stop_timeout_sec')
        
    use_battery_verify = LaunchConfiguration('use_battery_verify')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw/compressed'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('battery_topic', default_value='/battery_state'),

        DeclareLaunchArgument('target_marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.10'),

        DeclareLaunchArgument('pre_dock_distance_m', default_value='0.50'),
        DeclareLaunchArgument('final_dock_distance_m', default_value='0.15'),
        DeclareLaunchArgument('lateral_tolerance_m', default_value='0.060'),
        DeclareLaunchArgument('bearing_tolerance_rad', default_value='0.100'),
        DeclareLaunchArgument('k_z', default_value='0.20'),
        DeclareLaunchArgument('k_lateral', default_value='0.30'),
        DeclareLaunchArgument('k_bearing', default_value='0.80'),
        DeclareLaunchArgument('search_angular_speed_rps', default_value='0.10'),
        DeclareLaunchArgument('marker_lost_timeout_sec', default_value='0.80'),
        DeclareLaunchArgument('stale_stop_timeout_sec', default_value='0.15'),
        DeclareLaunchArgument('use_battery_verify', default_value='false'),

        Node(
            package='yeyu_control',
            executable='charging_dock_node',
            name='charging_dock_node',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'battery_topic': battery_topic,

                'target_marker_id': ParameterValue(target_marker_id, value_type=int),
                'marker_size_m': ParameterValue(marker_size_m, value_type=float),

                'pre_dock_distance_m': ParameterValue(pre_dock_distance_m, value_type=float),
                'final_dock_distance_m': ParameterValue(final_dock_distance_m, value_type=float),
                'lateral_tolerance_m': ParameterValue(lateral_tolerance_m, value_type=float),
                'bearing_tolerance_rad': ParameterValue(bearing_tolerance_rad, value_type=float),
                'k_z': ParameterValue(k_z, value_type=float),
                'k_lateral': ParameterValue(k_lateral, value_type=float),
                'k_bearing': ParameterValue(k_bearing, value_type=float),
                'search_angular_speed_rps': ParameterValue(search_angular_speed_rps, value_type=float),
                'marker_lost_timeout_sec': ParameterValue(marker_lost_timeout_sec, value_type=float),
                'stale_stop_timeout_sec': ParameterValue(stale_stop_timeout_sec, value_type=float),
                'use_battery_verify': ParameterValue(use_battery_verify, value_type=bool),
            }],
        ),
    ])