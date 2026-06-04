#!/usr/bin/env python3
import subprocess
import time
import rclpy
from rclpy.node import Node

class SwarmSupervisor(Node):
    def __init__(self):
        super().__init__('swarm_supervisor')
        self.get_logger().info("Swarm Supervisor online. Giving Gazebo 10 seconds to fully compile shaders...")
        time.sleep(10.0)
        
        # --- COMMENTED OUT FOR ISOLATED TESTING ---
        # poses = []
        # for i in range(12): poses.append((-82.5 + (i * 15.0), -10.0, 0.0))
        # poses.extend([
        #     (98.55, 5.18, 2.116), (71.62, 10.0, 3.141), (43.0, 10.0, 3.141),
        #     (14.4, 10.0, 3.141), (-14.3, 10.0, 3.141), (-42.9, 10.0, 3.141),
        #     (-71.6, 10.0, 3.141), (-98.5, 5.2, -2.122)
        # ])
        #
        # Sequential Deployment Loop
        # for i in range(1):
        #     ns = f"agv_{i+1:02d}"
        #     x, y, yaw = poses[i]
        #     self.spawn_and_verify(ns, x, y, yaw)
        # ------------------------------------------

        # --- NEW SHORTCUT: SPAWN EXACTLY AT STATION 11 ---
        self.get_logger().info(">>> SHORTCUT DEPLOYMENT: Spawning AGV_01 at Station 11 <<<")
        self.spawn_and_verify("agv_01", 67.5, -10.0, 0.0)

    def spawn_and_verify(self, ns, x, y, yaw):
        attempt = 1
        max_attempts = 5
        
        while attempt <= max_attempts:
            self.get_logger().info(f"Deploying {ns} (Attempt {attempt}/{max_attempts})...")
            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-topic', f'/{ns}/robot_description',
                '-name', ns, '-x', str(x), '-y', str(y), '-z', '0.402', '-Y', str(yaw)
            ]
            
            # Execute the spawn command and capture the output
            result = subprocess.run(cmd, capture_output=True, text=True)
            output = result.stdout + result.stderr

            # Verification Check: Exit code 0 OR Gazebo saying it already exists
            if result.returncode == 0 or "already exists" in output.lower():
                self.get_logger().info(f"[VERIFIED] {ns} successfully anchored to track.")
                time.sleep(0.5) # Brief pause before querying the next bot
                return
            else:
                self.get_logger().warn(f"[{ns}] Deployment timed out. Gazebo queue full. Retrying in 2s...")
                time.sleep(2.0)
                attempt += 1
        
        self.get_logger().error(f"CRITICAL: Failed to spawn {ns} after {max_attempts} attempts.")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmSupervisor()
    rclpy.shutdown()

if __name__ == '__main__':
    main()