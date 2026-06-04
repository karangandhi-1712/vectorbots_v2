#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor

class PulseBot(Node):
    def __init__(self, ns, start_station):
        super().__init__(f'{ns}_brain')
        self.ns = ns
        self.current_station = start_station
        self.publisher_ = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.subscription = self.create_subscription(Image, f'/{self.ns}/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.arrived = False
        self.last_blue_time = 0.0
        self.last_error = 0.0

    def image_callback(self, msg):
        # The Fleet Heartbeat: 35-second total cycle. 
        # 30s shift phase, 5s wait phase.
        current_time = self.get_clock().now().nanoseconds / 1e9
        cycle_phase = current_time % 35.0

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red Track Mask
        mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask1, mask2)

        # Blue Station Mask
        mask_blue = cv2.inRange(hsv, np.array([110, 150, 50]), np.array([130, 255, 255]))
        sees_blue = cv2.countNonZero(mask_blue) > 200

        # SHIFT PHASE (T=0 to T=30)
        if cycle_phase < 30.0:
            # If we see a blue station marker and haven't triggered it recently
            if sees_blue and (current_time - self.last_blue_time) > 15.0:
                if self.current_station >= 12:
                    self.current_station = 1
                else:
                    self.current_station += 1
                
                self.last_blue_time = current_time
                self.arrived = True
                self.get_logger().info(f"[{self.ns}] Docked at Station {self.current_station}")

            # If we haven't hit our target yet, keep driving!
            if not self.arrived:
                # Zone B (Transit) travels at 1.05 m/s to close the huge gap
                # Zone A (Stations) travels at 0.5 m/s
                speed = 1.05 if self.current_station >= 12 else 0.5
                self.twist.linear.x = speed

                # PD Steering Logic with Region of Interest (ROI)
                h, w, d = cv_image.shape
                
                # Black out everything except a narrow 20-pixel horizontal strip
                # This stops the centroid calculation from jumping around
                search_top = int(h / 2)
                search_bot = search_top + 20
                mask_red[0:search_top, 0:w] = 0
                mask_red[search_bot:h, 0:w] = 0

                contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        error = (w // 2) - cx
                        
                        # Tuned PD Controller for 2,000kg load at 1.05 m/s
                        kp = 0.004   # Drastically reduced to prevent violent oversteer
                        kd = 0.015   # Derivative shock absorber to dampen oscillations
                        
                        derivative = error - self.last_error
                        self.twist.angular.z = float((error * kp) + (derivative * kd))
                        self.last_error = error
                else:
                    self.twist.angular.z = 0.0

        # WAIT PHASE (T=30 to T=35) -> Global Synchronized Stop
        else:
            self.arrived = False
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    # Multi-threading to process 20 OpenCV streams simultaneously
    executor = MultiThreadedExecutor(num_threads=20)
    nodes = []
    
    for i in range(1, 21):
        ns = f"agv_{i:02d}"
        node = PulseBot(ns, start_station=i)
        executor.add_node(node)
        nodes.append(node)
        
    print(">>> VECTOR FLEET PULSE CONTROLLER ONLINE <<<")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()