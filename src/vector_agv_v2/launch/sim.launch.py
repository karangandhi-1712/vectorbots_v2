import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('vector_agv_v2')
    world_file = os.path.join(pkg_share, 'worlds', 'stadium.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vector_v2.urdf.xacro')

    # Path to the directory that CONTAINS the shelf_big_movai folder
    models_dir = os.path.join(pkg_share, 'models')

    ld = LaunchDescription()

    # 0. Inject model path into Gazebo's search path
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir
    ))

    # 1. Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', f'-r {world_file}')]
    )
    ld.add_action(gz_sim)

    # 2. Setup States and Bridges for 20 bots
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for i in range(20):
        ns = f"agv_{i+1:02d}"
        robot_desc = ParameterValue(
            Command(['xacro ', xacro_file, ' namespace:=', ns]),
            value_type=str
        )
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': robot_desc,
                'frame_prefix': f"{ns}/",
                'use_sim_time': True
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        )
        ld.add_action(rsp)
        bridge_args.append(f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')
        bridge_args.append(f'/{ns}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(bridge_node)

    # 3. Swarm Supervisor
    supervisor = Node(
        package='vector_agv_v2',
        executable='swarm_supervisor.py',
        name='swarm_supervisor',
        output='screen'
    )
    ld.add_action(supervisor)

    return ld