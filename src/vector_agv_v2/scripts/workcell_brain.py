#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
import threading
import time

class WorkcellBrain(Node):
    def __init__(self):
        super().__init__('workcell_brain')
        # Listen for AGV docking events globally
        self.sub = self.create_subscription(Int32, '/station_trigger', self.trigger_cb, 10)
        
        # Pre-create publishers for all 12 arms (3 joints each)
        self.arm_pubs = {}
        for i in range(1, 13):
            ns = f"arm_{i:02d}"
            self.arm_pubs[ns] = {
                'j1': self.create_publisher(Float64, f"/{ns}/joint1/cmd_pos", 10),
                'j2': self.create_publisher(Float64, f"/{ns}/joint2/cmd_pos", 10),
                'j3': self.create_publisher(Float64, f"/{ns}/joint3/cmd_pos", 10)
            }
        
        self.get_logger().info("Workcell Brain Online. Awaiting station triggers...")

    def trigger_cb(self, msg):
        station_id = msg.data
        if 1 <= station_id <= 12:
            self.get_logger().info(f"Trigger received for arm_{station_id:02d}. Executing sweep.")
            # Run the arm animation in a separate thread so it doesn't block other triggers
            threading.Thread(target=self.execute_arm_sequence, args=(station_id,)).start()

    def execute_arm_sequence(self, station_id):
        ns = f"arm_{station_id:02d}"
        pubs = self.arm_pubs[ns]
        
        def send_cmd(j1, j2, j3):
            pubs['j1'].publish(Float64(data=float(j1)))
            pubs['j2'].publish(Float64(data=float(j2)))
            pubs['j3'].publish(Float64(data=float(j3)))

        # Sequence 1: Extend over the AGV payload
        send_cmd(0.0, 1.2, -1.5)
        time.sleep(1.5)
        
        # Sequence 2: Inspection sweep (rotate base slightly)
        send_cmd(0.5, 1.2, -1.5)
        time.sleep(1.0)
        send_cmd(-0.5, 1.2, -1.5)
        time.sleep(1.0)
        
        # Sequence 3: Retract to safe home position before AGV takes off
        send_cmd(0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = WorkcellBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()