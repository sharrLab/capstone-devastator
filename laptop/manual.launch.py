import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

DIR = os.path.expanduser('~/nav_test')


def generate_launch_description():
    joy = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{'device_id': 0, 'deadzone': 0.05,
                     'autorepeat_rate': 20.0}])

    mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        output='screen',
        parameters=[{
            'use_stamped': False,
            'topics': {
                'manual': {'topic': 'cmd_vel_manual', 'timeout': 0.5, 'priority': 100},
                'nav': {'topic': 'cmd_vel_safe', 'timeout': 0.5, 'priority': 10}}}],
        remappings=[('cmd_vel_out', 'cmd_vel')])

    keeper = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(DIR, 'goal_keeper.py')],
        output='screen')

    teleop = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(DIR, 'teleop_manual.py')],
        output='screen')

    return LaunchDescription([joy, mux, keeper, teleop])
