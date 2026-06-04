#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
class SingleBotBrain(Node):
    def __init__(self):
        super().__init__('agv_01_brain')
        
        # WE ARE ONLY CONTROLLING AGV 01
        self.ns = 'agv_01'
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.sub = self.create_subscription(Image, f'/{self.ns}/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        
        # --- STATE MACHINE VARIABLES ---
        self.state = 'FOLLOWING' 
        self.stop_time = 0.0
        self.last_station_time = -20.0 
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(">>> AGV 01: ISOLATED BRAIN ONLINE. Awaiting camera feed... <<<")

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Physics Stabilization Wait
        if (now - self.start_time) < 2.0:
            return 

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        twist = Twist()

        # ==========================================
        # STATE: STOPPED AT STATION
        # ==========================================
        if self.state == 'STOPPED':
            if (now - self.stop_time) > 5.0:  # 5 Second Wait
                self.get_logger().info(">>> AGV 01: Leaving station. Resuming track. <<<")
                self.state = 'FOLLOWING'
                self.last_station_time = now
            else:
                self.pub.publish(twist) # Keep publishing 0 velocity
                return

        # ==========================================
        # STATE: FOLLOWING
        # ==========================================
        # A. Check for Station (Blue)
        blue_mask = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([140, 255, 255]))
        
        if cv2.countNonZero(blue_mask) > 150 and (now - self.last_station_time) > 15.0:
            self.get_logger().info(">>> AGV 01: Station detected! Initiating 5-second hold. <<<")
            self.state = 'STOPPED'
            self.stop_time = now
            self.pub.publish(twist)
            return

        # B. Track the Red Line
        m1 = cv2.inRange(hsv, np.array([0, 40, 40]), np.array([15, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([165, 40, 40]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(m1, m2)

        # Look strictly at the lower 25% of the screen for maximum stability
        scan_row = int(h * 0.75)
        line_pixels = np.where(red_mask[scan_row] > 0)[0]

        if len(line_pixels) < 10:
            # BLIND: Coast forward to find the line
            twist.linear.x = 0.15
            twist.angular.z = 0.0
        else:
            # CENTER-LOCK: Calculate error and steer
            center_x = np.mean(line_pixels)
            error = center_x - (w / 2.0)
            
            # 1. TIGHTER TURNS: Increased from 0.015
            # NOTE: If your robot was previously using a negative sign (e.g., -0.015), 
            # make sure this is -0.025! Keep whichever sign worked for the U-Turn.
            kp = 0.025 
            
            # 2. HIGHER STEERING LIMIT: Increased cap from 0.4 to 0.6 for sharper cornering
            twist.angular.z = float(np.clip(kp * error, -0.6, 0.6))
            
            # 3. DYNAMIC BRAKING: Go 0.35 on straights, but hit the brakes down to 0.15 on sharp turns
            twist.linear.x = float(np.clip(0.35 - abs(twist.angular.z), 0.15, 0.35))
            

def main(args=None):
    rclpy.init(args=args)
    node = SingleBotBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()