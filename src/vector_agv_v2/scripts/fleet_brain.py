#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32  # ADDED: For triggering the arm
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
    7.5, 22.5, 37.5, 52.5, 67.5, 82.5, 97.5, 112.5, 127.5, 142.5, 157.5, 172.5,
    201.148, 229.796, 258.416, 287.016, 315.716, 344.316, 373.016, 401.638
]

class FleetBot(Node):
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
        
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        # ADDED: Trigger publisher for the robotic arms
        self.trigger_pub = self.create_publisher(Int32, "/station_trigger", 10)
        
        self.sub = self.create_subscription(
            Image, f"/{ns}/camera/image_raw", self.image_cb, qos_profile_sensor_data)
        self.twist = Twist()

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        if (now - self.start_time) < 2.0: return

        if 7.5 <= self.s < 172.5:
            self.cruise_speed = 0.50
        else:
            self.cruise_speed = 0.955

        self.s = (self.s + (self.twist.linear.x * dt)) % TRACK_PERIMETER

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)

        if cv2.countNonZero(blue) > 50 and (now - self.last_blue) > 15.0 and self.docking_station is None:
            self.last_blue = now
            projected_s = (self.s + 4.3) % TRACK_PERIMETER
            self.docking_station = min([7.5 + (i*15.0) for i in range(12)], key=lambda st: abs(st - projected_s))

        if now < self.stop_until:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            if self.docking_station is not None:
                dist_left = self.docking_station - self.s
                if dist_left < -100.0: dist_left += TRACK_PERIMETER 
                
                if dist_left <= 0.05:
                    self.stop_until = now + 5.0
                    self.s = self.docking_station
                    
                    # ADDED: Trigger the robotic arm mechanism
                    station_idx = int(round((self.docking_station - 7.5) / 15.0)) + 1
                    msg = Int32()
                    msg.data = station_idx
                    self.trigger_pub.publish(msg)
                    
                    self.docking_station = None
                    self.twist.linear.x = 0.0
                else:
                    self.twist.linear.x = max(0.15, self.cruise_speed * (dist_left / 4.3))
            else:
                self.twist.linear.x = self.cruise_speed
            
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