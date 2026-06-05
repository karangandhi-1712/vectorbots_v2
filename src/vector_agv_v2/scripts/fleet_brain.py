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
    # THE GOD-BRAIN: Shared memory across all 20 nodes for absolute collision prevention
    GLOBAL_S = {i: S0_MAPPING[i] for i in range(20)}

    def __init__(self, ns, bot_id):
        super().__init__(f"{ns}_brain")
        self.ns = ns
        self.bot_id = bot_id
        
        self.s = S0_MAPPING[self.bot_id] 
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.last_time = self.start_time
        self.last_blue = -20.0 
        self.stop_until = 0.0 
        
        self.docking_station = None 
        self.state = "MOVING"
        self.boost_until = 0.0
        self.is_acc_active = False # Tracks if Virtual Bumper is engaged
        
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.sub = self.create_subscription(
            Image, f"/{ns}/camera/image_raw", self.image_cb, qos_profile_sensor_data)
        
        self.twist = Twist()

    def log_status(self, msg):
        # STRICT LOG FILTER: Logging for AGV 13, 16, 17, and 20
        # Indices: 12=AGV_13, 15=AGV_16, 16=AGV_17, 19=AGV_20
        if self.bot_id in [12, 15, 16, 19]:
            self.get_logger().info(f"[{self.ns}] {msg}")

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        if (now - self.start_time) < 2.0: return

        # 1. ADVANCE VIRTUAL RAIL POSITION & UPDATE GOD-BRAIN
        self.s = (self.s + (self.twist.linear.x * dt)) % TRACK_PERIMETER
        FleetBot.GLOBAL_S[self.bot_id] = self.s

        # 2. THE 3-ZONE DYNAMIC SPEED BRAIN
        if self.s < 180.0:
            current_cruise = 0.50   # Zone A: Station Crawl
        elif self.s < 390.0:
            current_cruise = 1.05   # Zone B: High-Speed Transit
        else:
            dist_to_zone_a = TRACK_PERIMETER - self.s 
            current_cruise = 0.50 + 0.55 * (dist_to_zone_a / 32.832)

        # 3. THE SWARM VIRTUAL BUMPER (Collision Prevention)
        # Identify the robot immediately ahead of us on the loop
        leader_id = (self.bot_id + 1) % 20
        leader_s = FleetBot.GLOBAL_S[leader_id]
        
        # Calculate true gap, wrapping around the loop perimeter
        gap = (leader_s - self.s) % TRACK_PERIMETER
        
        # If gap < 12m (7.5m chassis + 4.5m safety gap), trigger Adaptive Cruise Control
        if gap < 12.0:
            if not self.is_acc_active:
                self.is_acc_active = True
                self.log_status(f"!!! VIRTUAL BUMPER ENGAGED !!! Gap to AGV_{leader_id+1:02d} dropped to {gap:.1f}m. Throttling speed.")
            
            # Scales speed to 0.0 exactly as the gap hits 8.0 meters (0.5m physical clearance)
            safety_multiplier = max(0.0, (gap - 8.0) / 4.0)
            current_cruise = current_cruise * safety_multiplier
        else:
            if self.is_acc_active:
                self.is_acc_active = False
                self.log_status(f"Track Clear. Gap is {gap:.1f}m. Resuming normal operations.")

        # 4. STRICT STATION DETECTION (Geographic Gating)
        if self.s < 180.0 and self.docking_station is None:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)

            if cv2.countNonZero(blue) > 50 and (now - self.last_blue) > 15.0:
                self.last_blue = now
                projected_s = (self.s + 4.3) % TRACK_PERIMETER
                self.docking_station = min([7.5 + (i*15.0) for i in range(12)], key=lambda st: abs(st - projected_s))

        # 5. KINEMATIC EXECUTION & STATE MACHINE
        if now < self.stop_until:
            if self.state != "STOPPED":
                self.state = "STOPPED"
                self.log_status(f"DOCKED at station {self.s:.1f}m. 5-second timer STARTED.")
            
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            if self.state == "STOPPED":
                self.state = "MOVING"
                self.boost_until = now + 1.5 
                self.log_status(f"Timer complete. Firing Diff-Drive Torque Boost to clear station!")

            if self.docking_station is not None:
                if self.state != "BRAKING":
                    self.state = "BRAKING"
                    self.log_status(f"Camera locked onto Station {self.docking_station:.1f}. Initiating glide.")
                
                dist_left = self.docking_station - self.s
                if dist_left < -100.0: dist_left += TRACK_PERIMETER 
                
                if dist_left <= 0.05:
                    self.stop_until = now + 5.0
                    self.s = self.docking_station  
                    self.docking_station = None
                    self.twist.linear.x = 0.0
                else:
                    target_speed = current_cruise * (dist_left / 4.3)
                    self.twist.linear.x = min(current_cruise, max(0.2, target_speed))
            else:
                if self.state == "BRAKING":
                    self.state = "MOVING" 

                if now < self.boost_until and not self.is_acc_active:
                    self.twist.linear.x = current_cruise * 1.5
                else:
                    self.twist.linear.x = current_cruise
            
            # YOUR PERFECT TURN MATH
            perfect_turn_velocity = self.twist.linear.x / RADIUS
            
            if self.s < 180.0:
                self.twist.angular.z = 0.0                   
            elif self.s < 211.416:
                self.twist.angular.z = perfect_turn_velocity 
            elif self.s < 391.416:
                self.twist.angular.z = 0.0                   
            else:
                self.twist.angular.z = perfect_turn_velocity 

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