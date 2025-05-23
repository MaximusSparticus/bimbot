import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('bimbot')
    
    # Path to URDF and SDF files
    builderbot_sdf = os.path.join(pkg_share, 'models', 'builderbot', 'builderbot.sdf')
    inspectorbot_sdf = os.path.join(pkg_share, 'models', 'inspectorbot', 'inspectorbot.sdf')
    
    # World file launch arguments
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value='empty.sdf',
        description='Gazebo world file to load'
    )
    
    # Launch arguments for enabling/disabling robots
    builderbot_arg = DeclareLaunchArgument(
        'spawn_builderbot', 
        default_value='true',
        description='Enable spawning of BuilderBot'
    )
    
    inspectorbot_arg = DeclareLaunchArgument(
        'spawn_inspectorbot', 
        default_value='true',
        description='Enable spawning of InspectorBot'
    )
    
    # Gz Sim launch with specified world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', 
             '-r',  # Run simulation
             LaunchConfiguration('world')
        ],
        output='screen'
    )
    
    # Spawn BuilderBot
    spawn_builderbot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/empty/create', 
             '--reqtype', 'gz.msgs.EntityFactory', 
             '--reptype', 'gz.msgs.Boolean', 
             '--timeout', '1000', 
             '--req', f'sdf_filename: "{builderbot_sdf}", name: "builderbot", pose: {{position: {{x: 0, y: 0, z: 0.5}}}}'],
        condition=IfCondition(LaunchConfiguration('spawn_builderbot')),
        output='screen'
    )
    
    # Spawn InspectorBot
    spawn_inspectorbot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/empty/create', 
             '--reqtype', 'gz.msgs.EntityFactory', 
             '--reptype', 'gz.msgs.Boolean', 
             '--timeout', '1000', 
             '--req', f'sdf_filename: "{inspectorbot_sdf}", name: "inspectorbot", pose: {{position: {{x: 5, y: 0, z: 2}}}}'],
        condition=IfCondition(LaunchConfiguration('spawn_inspectorbot')),
        output='screen'
    )
    
    # Create launch description
    return LaunchDescription([
        world_arg,
        builderbot_arg,
        inspectorbot_arg,
        gz_sim,
        spawn_builderbot,
        spawn_inspectorbot
    ])
