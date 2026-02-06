# MIT License

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    """
    Launch configuration that starts the particle filter localization stack
    together with a mock sensor data publisher for CPU benchmarking.
    """
    
    # Locate configuration file
    config_dir = get_package_share_directory('particle_filter')
    localize_config_path = os.path.join(config_dir, 'config', 'localize.yaml')
    
    # Read map name from config
    with open(localize_config_path, 'r') as config_file:
        config_data = yaml.safe_load(config_file)
    map_name = config_data['map_server']['ros__parameters']['map']
    
    # Launch argument for config file
    config_arg = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config_path,
        description='Path to localization configuration file'
    )
    
    # Particle filter node
    particle_filter_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')],
        output='screen'
    )
    
    # Map server node
    map_yaml_path = os.path.join(config_dir, 'maps', f'{map_name}.yaml')
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': map_yaml_path},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': False}
        ],
        output='screen'
    )
    
    # Lifecycle manager
    lifecycle_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # Mock data publisher - use ExecuteProcess to run from repo without installation
    # Navigate from installed package share to repo root
    repo_particle_filter_dir = os.path.join(config_dir, '..', '..', 'particle_filter')
    fake_publisher_script = os.path.join(repo_particle_filter_dir, 'fake_data_publisher.py')
    
    mock_publisher_process = ExecuteProcess(
        cmd=['python3', fake_publisher_script],
        name='mock_data_publisher',
        output='screen'
    )
    
    # Assemble launch description
    launch_desc = LaunchDescription([config_arg])
    launch_desc.add_action(lifecycle_mgr_node)
    launch_desc.add_action(map_server_node)
    launch_desc.add_action(particle_filter_node)
    launch_desc.add_action(mock_publisher_process)
    
    return launch_desc
