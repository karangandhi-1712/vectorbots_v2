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

# --- VISION CONSTANTS (Only Blue is needed now!) ---
BLUE_LOW, BLUE_HIGH = np.array([100, 150, 50]), np.array([140, 255, 255])

# --- SDF HARDCODED TRACK CONSTANTS ---
TRACK_PERIMETER = 422.832
RADIUS = 10.0

# Exact starting distance (in meters) along the 422m perimeter for all 20 robots
S0_MAPPING = [
    7.5, 22.5, 37.5, 52.5, 67.5, 82.5, 97.5, 112.5, 127.5, 142.5, 157.5, 172.5, # Station Bots (1-12)
    201.148, 229.796, 258.416, 287.016, 315.716, 344.316, 373.016, 401.638      # Transit Bots (13-20)
]

class FleetBot(Node):
    def __init__(self, ns, bot_id):
        super().__init__(f"{ns}_brain")
        self.ns = ns
        self.bot_id = bot_id
        
        # Speeds: 0.50m/s for Station loopers, 1.05m/s for Transit sprinters
        self.cruise_speed = 0.50 if self.bot_id < 12 else 1.05
        
        # Initialize location on the Virtual Rail
        self.s = S0_MAPPING[self.bot_id] 
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_time = self.start_time
        self.last_blue = -20.0 
        self.stop_until = 0.0 # Strict 5-second timer
        
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

        # 1. ADVANCE VIRTUAL RAIL POSITION (Dead Reckoning)
        self.s = (self.s + (self.twist.linear.x * dt)) % TRACK_PERIMETER

        # 2. STATION DETECTION (RFID Style)
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)

        # Trigger exact 5-second stop
        if cv2.countNonZero(blue) > 150 and (now - self.last_blue) > 15.0:
            self.last_blue = now
            self.stop_until = now + 5.0  # Exactly 5 seconds
            
            # ANTI-DRIFT SNAP: Mathematically correct position based on nearest station
            nearest_station = min([7.5 + (i*15.0) for i in range(12)], key=lambda st: abs(st - self.s))
            self.s = nearest_station

        # 3. KINEMATIC EXECUTION
        if now < self.stop_until:
            # Hard Stop at Station
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            # Drive via Hardcoded SDF Map
            self.twist.linear.x = self.cruise_speed
            
            # Turn radius is v / R. This computes the mathematically flawless wheel speed.
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
    # Notice the index mapping ensures bot_id aligns with S0_MAPPING perfectly (0 to 19)
    nodes = [FleetBot(f"agv_{i+1:02d}", i) for i in range(20)]
    for n in nodes: executor.add_node(n)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        for n in nodes: n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()