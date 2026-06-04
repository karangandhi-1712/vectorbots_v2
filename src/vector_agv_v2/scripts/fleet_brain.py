#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

# --- VISION CONSTANTS ---
RED_LOW_1, RED_HIGH_1 = np.array([0, 40, 40]), np.array([15, 255, 255])
RED_LOW_2, RED_HIGH_2 = np.array([165, 40, 40]), np.array([180, 255, 255])
BLUE_LOW, BLUE_HIGH   = np.array([100, 150, 50]), np.array([140, 255, 255])

class FleetBot(Node):
    def __init__(self, ns, bot_id):
        super().__init__(f"{ns}_brain")
        self.ns = ns
        self.bot_id = bot_id
        
        # PULSE LINE LOGIC: Transit bots (13-20) sprint to close the massive gap
        self.cruise_speed = 0.50 if self.bot_id <= 12 else 1.05
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_blue = -20.0 
        self.arrived = False 
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        
        self.sub = self.create_subscription(
            Image, f"/{ns}/camera/image_raw", self.image_cb, qos_profile_sensor_data)
        
        self.twist = Twist()

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.start_time) < 2.0: return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        # STATION LOGIC (30 sec move, 5 sec hold)
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
            
            # SHIFTED ROWS: Looks above the white bumper
            rows = [int(h * 0.30), int(h * 0.40), int(h * 0.50)]
            centers = [np.mean(np.where(mask[y] > 0)[0]) for y in rows if len(np.where(mask[y] > 0)[0]) > 20]

            if len(centers) < 2:
                # TURN MEMORY: Keep turning if the line slips out of the peripheral vision
                if self.twist.angular.z > 0.2:
                    self.twist.linear.x = 0.05
                    self.twist.angular.z = 0.8
                elif self.twist.angular.z < -0.2:
                    self.twist.linear.x = 0.05
                    self.twist.angular.z = -0.8
                else:
                    self.twist.linear.x = 0.15
                    self.twist.angular.z = 0.0
            else:
                # DYNAMIC STEERING MATH
                err = ((centers[-1] + centers[0]) * 0.5) - (w / 2.0)
                curv = (centers[-1] - centers[0])
                
                kp = 0.050 
                kd = 0.020
                
                self.twist.angular.z = float(np.clip((kp * err) + (kd * curv), -1.5, 1.5))
                
                # DYNAMIC BRAKING: Hit the brakes on corners, but resume Pulse Line speed on straights
                if abs(self.twist.angular.z) > 0.4:
                    self.twist.linear.x = 0.05  
                else:
                    self.twist.linear.x = self.cruise_speed  

        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=20)
    # Passes both the namespace and the bot_id to correctly assign speeds
    nodes = [FleetBot(f"agv_{i:02d}", i) for i in range(1, 21)]
    for n in nodes: executor.add_node(n)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        for n in nodes: n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()