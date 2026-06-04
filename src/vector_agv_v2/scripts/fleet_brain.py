#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# --- VISION CONSTANTS ---
RED_LOW_1, RED_HIGH_1 = np.array([0, 40, 40]), np.array([15, 255, 255])
RED_LOW_2, RED_HIGH_2 = np.array([165, 40, 40]), np.array([180, 255, 255])
BLUE_LOW, BLUE_HIGH   = np.array([100, 150, 50]), np.array([140, 255, 255])

class FleetBot(Node):
    def __init__(self, ns, start_station):
        super().__init__(f"{ns}_brain")
        self.ns = ns
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_blue = -20.0 
        self.arrived = False 
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.sub = self.create_subscription(Image, f"/{ns}/camera/image_raw", self.image_cb, 10)
        self.twist = Twist()

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.start_time) < 2.0: return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        # STATION LOGIC
        cycle_phase = now % 35.0
        if cycle_phase >= 30.0: self.arrived = False
        
        blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
        if cv2.countNonZero(blue) > 150 and (now - self.last_blue) > 15.0:
            self.last_blue = now
            self.arrived = True

        # CONTROLLER (SINGLE SOURCE OF TRUTH)
        if self.arrived or cycle_phase >= 30.0:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            m1 = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1)
            m2 = cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)
            mask = cv2.bitwise_or(m1, m2)
            
            rows = [int(h * 0.35), int(h * 0.50), int(h * 0.65)]
            centers = [np.mean(np.where(mask[y] > 0)[0]) for y in rows if len(np.where(mask[y] > 0)[0]) > 20]

            if len(centers) < 2:
                self.twist.linear.x, self.twist.angular.z = 0.15, 0.0
            else:
                # LINE LOCK MATH:
                # KP/KD scheduled for stability. If spinning, flip the KP sign.
                err = ((centers[-1] + centers[0]) * 0.5) - (w / 2.0)
                curv = (centers[-1] - centers[0])
                
                # If spinning, change KP from -0.015 to 0.015
                self.twist.linear.x = 0.3
                self.twist.angular.z = float(np.clip((0.015 * err) + (-0.010 * curv), -0.4, 0.4))

        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=20)
    nodes = [FleetBot(f"agv_{i:02d}", i) for i in range(1, 21)]
    for n in nodes: executor.add_node(n)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        for n in nodes: n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()