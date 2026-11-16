import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ===== UKURAN ROBOT (meter). Ganti dengan hasil ukur Anda =====
TANK_LENGTH = 0.223   # panjang tank termasuk track
TANK_WIDTH = 0.223    # lebar tank termasuk track
LASER_X = -0.0025      # posisi LiDAR dari tengah tank (positif ke depan)
LASER_Z = 0.1652       # tinggi bidang scan dari lantai
LASER_YAW = 3.14159  # frame laser diputar 180 derajat (sudah diuji di RViz)
# ===============================================================

# ===== PETA TERSIMPAN =====
MAP_NAME = 'ruang1'      # nama sesuai perintah simpanpeta
USE_SAVED_MAP = False     # False = selalu mapping baru
MAP_DIR = os.path.expanduser('~/nav_test/maps')
# ==========================

# ===== ODOMETRI =====
ODOM_ON_PI = False   # rf2o dan TF laser dijalankan di Pi (robotnav)

TMP_SLAM = '/tmp/slam_nav.yaml'
TMP_NAV2 = '/tmp/nav2_merged.yaml'


def deep_update(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = v


def footprint_string():
    lx, wy = TANK_LENGTH / 2.0, TANK_WIDTH / 2.0
    return (f'[[{lx:.3f}, {wy:.3f}], [{lx:.3f}, {-wy:.3f}], '
            f'[{-lx:.3f}, {-wy:.3f}], [{-lx:.3f}, {wy:.3f}]]')


def write_slam_params():
    params = {'slam_toolbox': {'ros__parameters': {
        'odom_frame': 'odom', 'map_frame': 'map', 'base_frame': 'base_link',
        'scan_topic': '/scan_wifi', 'mode': 'mapping',
        'resolution': 0.05, 'max_laser_range': 12.0,
        'minimum_time_interval': 0.2, 'transform_timeout': 1.0,
        'tf_buffer_duration': 30.0, 'map_update_interval': 1.0,
        'transform_publish_period': 0.02, 'throttle_scans': 1,
        'minimum_travel_distance': 0.1, 'minimum_travel_heading': 0.2,
        'use_scan_matching': True, 'use_scan_barycenter': True,
        'do_loop_closing': True, 'enable_interactive_mode': False,
        'debug_logging': False, 'stack_size_to_use': 40000000,
        'coarse_search_angle_offset': 0.5, 'coarse_angle_resolution': 0.0349,
        'fine_search_angle_offset': 0.00349,
        'link_match_minimum_response_fine': 0.1}}}
    sp = params['slam_toolbox']['ros__parameters']
    posegraph = os.path.join(MAP_DIR, MAP_NAME + '.posegraph')
    if USE_SAVED_MAP and os.path.exists(posegraph):
        sp['mode'] = 'localization'
        sp['map_file_name'] = os.path.join(MAP_DIR, MAP_NAME)
        sp['map_start_at_dock'] = True
        print('[nav.launch] Memuat peta tersimpan:', MAP_NAME)
    else:
        print('[nav.launch] Mapping baru (tidak ada peta tersimpan)')
    with open(TMP_SLAM, 'w') as f:
        yaml.safe_dump(params, f)


def write_nav2_params():
    base_file = os.path.join(get_package_share_directory('nav2_bringup'),
                             'params', 'nav2_params.yaml')
    with open(base_file) as f:
        params = yaml.safe_load(f)

    fp = footprint_string()
    scan = {'topic': '/scan_wifi', 'max_obstacle_height': 2.0, 'clearing': True,
            'marking': True, 'data_type': 'LaserScan',
            'raytrace_max_range': 3.0, 'raytrace_min_range': 0.2,
            'obstacle_max_range': 2.5, 'obstacle_min_range': 0.2}
    obstacle_layer = {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                      'observation_sources': 'scan', 'scan': scan}
    inflation = {'plugin': 'nav2_costmap_2d::InflationLayer',
                 'cost_scaling_factor': 3.0, 'inflation_radius': 0.40}

    overrides = {
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'update_frequency': 5.0, 'publish_frequency': 2.0,
            'rolling_window': True, 'width': 3, 'height': 3, 'resolution': 0.05,
            'footprint': fp, 'transform_tolerance': 1.0,
            'plugins': ['obstacle_layer', 'inflation_layer'],
            'obstacle_layer': obstacle_layer, 'inflation_layer': inflation}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'update_frequency': 1.0, 'publish_frequency': 1.0,
            'resolution': 0.05, 'track_unknown_space': True,
            'footprint': fp, 'transform_tolerance': 1.0,
            'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer',
                             'map_subscribe_transient_local': True},
            'obstacle_layer': obstacle_layer, 'inflation_layer': inflation}}},
        'planner_server': {'ros__parameters': {
            'expected_planner_frequency': 1.0,
            'planner_plugins': ['GridBased'],
            'GridBased': {'plugin': 'nav2_navfn_planner::NavfnPlanner',
                          'tolerance': 0.5, 'use_astar': True,
                          'allow_unknown': True}}},
        'controller_server': {'ros__parameters': {
            'controller_frequency': 10.0, 'failure_tolerance': 1.5,
            'enable_stamped_cmd_vel': False,
            'progress_checker': {
                'plugin': 'nav2_controller::SimpleProgressChecker',
                'required_movement_radius': 0.5,
                'movement_time_allowance': 20.0},
            'general_goal_checker': {
                'stateful': True,
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': 0.20, 'yaw_goal_tolerance': 0.5}}},
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [0.25, 0.0, 1.5],
            'min_velocity': [-0.25, 0.0, -1.5],
            'max_accel': [1.0, 0.0, 3.0],
            'max_decel': [-1.0, 0.0, -3.0]}},
        'collision_monitor': {'ros__parameters': {
            'base_frame_id': 'base_link', 'odom_frame_id': 'odom',
            'cmd_vel_out_topic': 'cmd_vel_safe',
            'scan': {'topic': '/scan_wifi'}}},
    }
    deep_update(params, overrides)
    for node in ('bt_navigator', 'behavior_server', 'collision_monitor'):
        params[node]['ros__parameters']['transform_tolerance'] = 1.5

    # Pengontrol Regulated Pure Pursuit, ditulis penuh agar tidak bercampur dengan bawaan
    params['controller_server']['ros__parameters']['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': 0.15,
        'lookahead_dist': 0.5, 'min_lookahead_dist': 0.3, 'max_lookahead_dist': 0.8,
        'lookahead_time': 1.5, 'transform_tolerance': 2.0,
        'use_velocity_scaled_lookahead_dist': False,
        'min_approach_linear_velocity': 0.08,
        'approach_velocity_scaling_dist': 0.6,
        'use_collision_detection': True,
        'max_allowed_time_to_collision_up_to_carrot': 1.0,
        'use_regulated_linear_velocity_scaling': False,
        'use_cost_regulated_linear_velocity_scaling': False,
        'regulated_linear_scaling_min_radius': 0.9,
        'regulated_linear_scaling_min_speed': 0.1,
        'use_rotate_to_heading': True,
        'rotate_to_heading_angular_vel': 1.0,
        'rotate_to_heading_min_angle': 0.785,
        'allow_reversing': False,
        'max_angular_accel': 3.2,
        'max_robot_pose_search_dist': 10.0}

    with open(TMP_NAV2, 'w') as f:
        yaml.safe_dump(params, f)


def generate_launch_description():
    write_slam_params()
    write_nav2_params()

    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_laser',
        arguments=['--x', str(LASER_X), '--y', '0', '--z', str(LASER_Z),
                   '--yaw', str(LASER_YAW), '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'])

    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='screen',
        parameters=[{
            'laser_scan_topic': '/scan_wifi', 'odom_topic': '/odom',
            'publish_tf': True, 'base_frame_id': 'base_link',
            'odom_frame_id': 'odom', 'init_pose_from_topic': '',
            'freq': 10.0}])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': TMP_SLAM,
                          'use_sim_time': 'false'}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': TMP_NAV2,
                          'use_sim_time': 'false',
                          'autostart': 'true'}.items())

    rviz_cfg = os.path.join(get_package_share_directory('nav2_bringup'),
                            'rviz', 'nav2_default_view.rviz')
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(package='rviz2', executable='rviz2', arguments=rviz_args)

    aksi = [slam, nav2, rviz] if ODOM_ON_PI else [tf_laser, rf2o, slam, nav2, rviz]
    return LaunchDescription(aksi)
