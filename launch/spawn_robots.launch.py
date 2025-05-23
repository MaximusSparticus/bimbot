import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution
)
from launch_ros.actions import Node


def spawn_robots(context, *args, **kwargs):
    """
    Function to spawn robots with the correct world name extracted from the world file
    """
    # Get the world file path
    world_file = LaunchConfiguration('world').perform(context)
    
    # Extract world name from the file path
    # For example: 'construction_site.sdf' -> 'construction_site'
    # or 'worlds/my_world.sdf' -> 'my_world'
    world_name = os.path.splitext(os.path.basename(world_file))[0]
    
    # Get package share directory
    pkg_share = get_package_share_directory('bimbot')
    
    # Path to SDF files
    builderbot_sdf = os.path.join(pkg_share, 'models', 'builderbot', 'builderbot.sdf')
    inspectorbot_sdf = os.path.join(pkg_share, 'models', 'inspectorbot', 'inspectorbot.sdf')
    
    spawn_actions = []
    
    # Spawn BuilderBot if enabled
    if LaunchConfiguration('spawn_builderbot').perform(context) == 'true':
        spawn_builderbot = ExecuteProcess(
            cmd=['gz', 'service', '-s', f'/world/{world_name}/create', 
                 '--reqtype', 'gz.msgs.EntityFactory', 
                 '--reptype', 'gz.msgs.Boolean', 
                 '--timeout', '1000', 
                 '--req', f'sdf_filename: "{builderbot_sdf}", name: "builderbot", pose: {{position: {{x: -5, y: -1, z: 0.5}}}}'],
            output='screen'
        )
        spawn_actions.append(spawn_builderbot)
    
    # Spawn InspectorBot if enabled
    if LaunchConfiguration('spawn_inspectorbot').perform(context) == 'true':
        spawn_inspectorbot = ExecuteProcess(
            cmd=['gz', 'service', '-s', f'/world/{world_name}/create', 
                 '--reqtype', 'gz.msgs.EntityFactory', 
                 '--reptype', 'gz.msgs.Boolean', 
                 '--timeout', '1000', 
                 '--req', f'sdf_filename: "{inspectorbot_sdf}", name: "inspectorbot", pose: {{position: {{x: -5, y: 1, z: 2}}}}'],
            output='screen'
        )
        spawn_actions.append(spawn_inspectorbot)
    
    return spawn_actions


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('bimbot')
    
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
    
    # Use OpaqueFunction to spawn robots with the correct world name
    spawn_robots_action = OpaqueFunction(function=spawn_robots)

    # Bridge Inspector Bot    
    # Get the world file path
    #world_file = LaunchConfiguration('world').perform(context)
    
    # Extract world name from the file path
    # For example: 'construction_site.sdf' -> 'construction_site'
    # or 'worlds/my_world.sdf' -> 'my_world'
    #world_name = os.path.splitext(os.path.basename(world_file))[0]
    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/world/construction_site_from_ifc/model/inspectorbot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/construction_site_from_ifc/model/inspectorbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/world/construction_site_from_ifc/model/inspectorbot/link/gps_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        '/model/inspectorbot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    ],
    output='screen'
)

    # Create launch description
    return LaunchDescription([
        world_arg,
        builderbot_arg,
        inspectorbot_arg,
        gz_sim,
        spawn_robots_action,
        bridge_node
    ])
