from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nœud R2-D2 (maître)
        Node(
            package='mission_sime',
            executable='r2d2_master',
            name='r2d2_master',
            output='screen',
            parameters=[{
                'initial_x': 5.0,
                'initial_y': 5.0,
                'speed': 2.0,
            }]
        ),
        
        # Nœud C-3PO (suiveur)
        Node(
            package='mission_sime',
            executable='c3po_follower',
            name='c3po_follower',
            output='screen',
            parameters=[{
                'follow_distance': 2.0,
                'elimination_range': 1.5,
            }]
        ),
        
        # Nœud générateur d'ennemis
        Node(
            package='mission_sime',
            executable='enemy_spawner',
            name='enemy_spawner',
            output='screen',
            parameters=[{
                'spawn_rate': 5.0,
                'max_enemies': 5,
            }]
        ),
        
        # Nœud superviseur de mission
        Node(
            package='mission_sime',
            executable='mission_supervisor',
            name='mission_supervisor',
            output='screen',
        ),
    ])
