#!/usr/bin/env python3
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class SwarmSupervisor(Node):
    def __init__(self):
        super().__init__('swarm_supervisor')
        self.get_logger().info("Swarm Supervisor online. Giving Gazebo 10 seconds to fully compile shaders...")
        time.sleep(10.0)
        
        # Deploy 12 Robotic Arms at Stations (Y = -12.5 to clear the AGV payload)
        pkg_path = get_package_share_directory('vector_agv_v2') # UPDATE THIS IF YOUR PACKAGE NAME IS DIFFERENT
        arm_urdf_path = os.path.join(pkg_path, 'urdf', 'workcell_arm.urdf.xacro')

        self.get_logger().info("--- DEPLOYING WORKCELL ARMS ---")
        for i in range(12):
            ns = f"arm_{i+1:02d}"
            x = -82.5 + (i * 15.0)
            # Yaw is 1.57 (90 degrees) so the arm faces the track (+Y direction)
            self.spawn_from_file(ns, arm_urdf_path, x, -12.5, 0.0, 1.5708)

        # Deploy 20 AGVs (Unchanged)
        self.get_logger().info("--- DEPLOYING AGV FLEET ---")
        poses = []
        for i in range(12): poses.append((-82.5 + (i * 15.0), -10.0, 0.0))
        poses.extend([
            (98.55, 5.18, 2.116), (71.62, 10.0, 3.141), (43.0, 10.0, 3.141),
            (14.4, 10.0, 3.141), (-14.3, 10.0, 3.141), (-42.9, 10.0, 3.141),
            (-71.6, 10.0, 3.141), (-98.5, 5.2, -2.122)
        ])

        for i in range(20):
            ns = f"agv_{i+1:02d}"
            x, y, yaw = poses[i]
            self.spawn_from_topic(ns, x, y, yaw)
        
        self.get_logger().info(">>> FULL WAREHOUSE DEPLOYMENT COMPLETE. <<<")

    def spawn_from_file(self, ns, file_path, x, y, z, yaw):
        # Parses URDF file and passes the namespace arg
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', file_path,
            '-name', ns, '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)
        ]
        # Xacro args must be passed as environment variables to gz create
        env = os.environ.copy()
        env['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(file_path)
        
        subprocess.run(cmd, env=env, capture_output=True)
        self.get_logger().info(f"[ANCHORED] {ns} secured to factory floor.")
        time.sleep(0.2)

    def spawn_from_topic(self, ns, x, y, yaw):
        # Your original robust spawner logic
        attempt, max_attempts = 1, 5
        while attempt <= max_attempts:
            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-topic', f'/{ns}/robot_description',
                '-name', ns, '-x', str(x), '-y', str(y), '-z', '0.402', '-Y', str(yaw)
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0 or "already exists" in result.stdout.lower() + result.stderr.lower():
                self.get_logger().info(f"[VERIFIED] {ns} successfully anchored to track.")
                time.sleep(0.5) 
                return
            time.sleep(2.0)
            attempt += 1

def main(args=None):
    rclpy.init(args=args)
    node = SwarmSupervisor()
    rclpy.shutdown()

if __name__ == '__main__':
    main()