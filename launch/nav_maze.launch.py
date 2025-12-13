from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sv_maze_nav',
            executable='nav_maze',
            name='navigator',
            output='screen',
        ),
        Node(
            package='sv_maze_nav',
            executable='sensors',
            name='sensor',
            output='screen',
        ),
        Node(
            package='sv_maze_nav',
            executable='move_robot_client',
            name='mover',
            output='screen',
        ),
        Node(
            package='sv_maze_nav',
            executable='sign_id',
            name='sign_id',
            output='screen',
        ),
    ])

