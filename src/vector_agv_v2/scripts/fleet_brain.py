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
        self.last_turn = 0.0

    def image_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        cycle_phase = current_time % 35.0

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Track & Station Masks
        mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask1, mask2)
        mask_blue = cv2.inRange(hsv, np.array([110, 150, 50]), np.array([130, 255, 255]))
        sees_blue = cv2.countNonZero(mask_blue) > 200

        # SHIFT PHASE (T=0 to T=30)
        if cycle_phase < 30.0:
            if sees_blue and (current_time - self.last_blue_time) > 15.0:
                if self.current_station >= 12:
                    self.current_station = 1
                else:
                    self.current_station += 1
                
                self.last_blue_time = current_time
                self.arrived = True
                self.get_logger().info(f"[{self.ns}] Docked at Station {self.current_station}")

            if not self.arrived:
                # 1.05 m/s for transit bots, 0.5 m/s for station bots
                speed = 1.05 if self.current_station >= 12 else 0.5
                self.twist.linear.x = speed

                # 1. INCREASED LOOKAHEAD DISTANCE
                # Shift the ROI slightly higher up the image to anticipate the curve earlier
                h, w, d = cv_image.shape
                search_top = int(h * 0.4) # Look further ahead
                search_bot = int(h * 0.8) # Ignore the chassis hood directly below
                mask_red[0:search_top, 0:w] = 0
                mask_red[search_bot:h, 0:w] = 0

                contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        error = (w // 2) - cx
                        
                        # --- 2. THE "SOFT CATCH" GAIN SCHEDULER ---
                        if abs(error) < 15:
                            # Straightaway: Locked in tight.
                            kp = 0.002
                            kd = 0.04
                        else:
                            # Curve: Smooth, anticipated turn. 
                            # Dropped Kp to stop the violent whip. Increased Kd to absorb momentum.
                            kp = 0.006  
                            kd = 0.08   
                            
                        derivative = error - self.last_error
                        raw_steering = float((error * kp) + (derivative * kd))
                        
                        # --- 3. ELECTRONIC STEERING DAMPENER ---
                        # Blends 20% new math with 80% physical momentum for an ultra-smooth ride
                        alpha = 0.20 
                        smoothed_steering = (alpha * raw_steering) + ((1.0 - alpha) * self.last_turn)
                        
                        # --- 4. KINEMATIC CLAMP ---
                        # Prevent the wheels from turning faster than the 2000kg chassis can handle
                        self.twist.angular.z = max(min(smoothed_steering, 0.35), -0.35)
                        
                        self.last_error = error
                        self.last_turn = self.twist.angular.z
                else:
                    # MEMORY FALLBACK: If blind, hold the exact curve radius!
                    self.twist.angular.z = self.last_turn
            else:
                self.twist.linear.x = 0.0
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