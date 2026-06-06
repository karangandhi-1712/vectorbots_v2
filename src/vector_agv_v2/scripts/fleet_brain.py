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
BLUE_LOW, BLUE_HIGH = np.array([100, 150, 50]), np.array([140, 255, 255])

# --- SDF HARDCODED TRACK CONSTANTS ---
TRACK_PERIMETER = 422.832
RADIUS = 10.0

S0_MAPPING = [
    7.5, 22.5, 37.5, 52.5, 67.5, 82.5, 97.5, 112.5, 127.5, 142.5, 157.5, 172.5, # Station Bots (1-12)
    201.148, 229.796, 258.416, 287.016, 315.716, 344.316, 373.016, 401.638      # Transit Bots (13-20)
]

class FleetBot(Node):
    def __init__(self, ns, bot_id):
        super().__init__(f"{ns}_brain")
        self.ns = ns
        self.bot_id = bot_id
        
        # CHANGED: Removed the static cruise_speed assignment here.
        self.s = S0_MAPPING[self.bot_id] 
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_time = self.start_time
        self.last_blue = -20.0 
        self.stop_until = 0.0 
        
        self.docking_station = None 
        
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.sub = self.create_subscription(
            Image, f"/{ns}/camera/image_raw", self.image_cb, qos_profile_sensor_data)
        
        self.twist = Twist()

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        if (now - self.start_time) < 2.0: return

        # --- CHANGED: DYNAMIC SPEED SYNCHRONIZATION ---
        # If the robot is currently traversing the 15m gaps between stations...
        if 7.5 <= self.s < 172.5:
            self.cruise_speed = 0.50   # 15m / 30s = 0.50 m/s
        # If the robot is anywhere else (transit zones with ~28.6m gaps)...
        else:
            self.cruise_speed = 0.955  # 28.648m / 30s = 0.955 m/s

        # 1. ADVANCE VIRTUAL RAIL POSITION (Dead Reckoning stays untouched!)
        self.s = (self.s + (self.twist.linear.x * dt)) % TRACK_PERIMETER

        # 2. STATION DETECTION (Untouched)
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)

        if cv2.countNonZero(blue) > 50 and (now - self.last_blue) > 15.0 and self.docking_station is None:
            self.last_blue = now
            projected_s = (self.s + 4.3) % TRACK_PERIMETER
            self.docking_station = min([7.5 + (i*15.0) for i in range(12)], key=lambda st: abs(st - projected_s))

        # 3. KINEMATIC EXECUTION (Untouched)
        if now < self.stop_until:
            # Hard Stop at Station
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            # DECELERATION LOGIC
            if self.docking_station is not None:
                dist_left = self.docking_station - self.s
                
                # Handle loop wrap-around if bot is at 422m and station is at 7.5m
                if dist_left < -100.0: 
                    dist_left += TRACK_PERIMETER 
                
                if dist_left <= 0.05:
                    # Arrived at center! Trigger 5s stop.
                    self.stop_until = now + 5.0
                    self.s = self.docking_station  # Anti-drift snap ONLY upon arrival
                    self.docking_station = None
                    self.twist.linear.x = 0.0
                else:
                    # Smooth deceleration, capping at 0.15m/s so it doesn't stall infinitely
                    self.twist.linear.x = max(0.15, self.cruise_speed * (dist_left / 4.3))
            else:
                self.twist.linear.x = self.cruise_speed
            
            # YOUR PERFECT TURN MATH (Untouched)
            perfect_turn_velocity = self.twist.linear.x / RADIUS
            
            if self.s < 180.0:
                self.twist.angular.z = 0.0                   # Bottom Straight
            elif self.s < 211.416:
                self.twist.angular.z = perfect_turn_velocity # Right Curve
            elif self.s < 391.416:
                self.twist.angular.z = 0.0                   # Top Straight
            else:
                self.twist.angular.z = perfect_turn_velocity # Left Curve

        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=20)
    nodes = [FleetBot(f"agv_{i+1:02d}", i) for i in range(20)]
    for n in nodes: executor.add_node(n)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        for n in nodes: n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()