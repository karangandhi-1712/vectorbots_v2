import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('vector_agv_v2')
    world_file = os.path.join(pkg_share, 'worlds', 'stadium.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vector_v2.urdf.xacro')

    ld = LaunchDescription()

    # 1. Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    ld.add_action(gz_sim)

    # 2. Setup States and Bridges for 20 bots
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    
    for i in range(20):
        ns = f"agv_{i+1:02d}"
        doc = xacro.process_file(xacro_file, mappings={'namespace': ns})
        
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{'robot_description': doc.toxml(), 'frame_prefix': f"{ns}/"}]
        )
        ld.add_action(rsp)

        bridge_args.append(f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')
        bridge_args.append(f'/{ns}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args
    )
    ld.add_action(bridge_node)

    # 3. The Intelligent Swarm Supervisor
    supervisor = Node(
        package='vector_agv_v2',
        executable='swarm_supervisor.py',
        name='swarm_supervisor',
        output='screen'
    )
    ld.add_action(supervisor)

    return ld