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

    # Calculate exactly where the 20 bots belong
    poses = []
    # 12 Bots perfectly on the 12 stations (Bottom Straight)
    for i in range(12):
        poses.append((-82.5 + (i * 15.0), -10.0, 0.0))
        
    # 8 Bots exactly 28.65m apart in the Transit Zone (Top + Curves)
    poses.extend([
        (98.55, 5.18, 0.545),    # Bot 13 (Right Curve)
        (71.62, 10.0, 3.141),    # Bot 14 (Top Straight)
        (43.0, 10.0, 3.141),     # Bot 15
        (14.4, 10.0, 3.141),     # Bot 16
        (-14.3, 10.0, 3.141),    # Bot 17
        (-42.9, 10.0, 3.141),    # Bot 18
        (-71.6, 10.0, 3.141),    # Bot 19
        (-98.5, 5.2, 2.59)       # Bot 20 (Left Curve)
    ])

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']

    # Spawning Loop
    for i in range(20):
        ns = f"agv_{i+1:02d}"
        x, y, yaw = poses[i]
        
        # Process Xacro with namespace
        doc = xacro.process_file(xacro_file, mappings={'namespace': ns})
        robot_desc = doc.toxml()

        # RSP Node
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{'robot_description': robot_desc, 'frame_prefix': f"{ns}/"}]
        )
        ld.add_action(rsp)

        # Gazebo Spawner
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-name', ns,
                '-x', str(x), '-y', str(y), '-z', '0.402', '-Y', str(yaw)
            ]
        )
        ld.add_action(spawner)

        # Setup bridge mappings
        bridge_args.append(f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')
        bridge_args.append(f'/{ns}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')

    # Global Bridge Node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args
    )
    ld.add_action(bridge_node)

    return ld