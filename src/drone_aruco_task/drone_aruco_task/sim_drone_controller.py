#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from std_msgs.msg import Int32
import tf_transformations

# --- DEFINICIJA STANJA ---
TAKEOFF = 0       # Ručno dizanje s poda
ASCEND = 1        # Fino podešavanje na 2.0m
SEARCH = 2        # Rotacija dok ne nađemo marker
APPROACH = 3      # Prilaženje na 0.6m
RETURN_HOME = 4   # Povratak na (0,0)
LAND = 5          # Ručno spuštanje
FINISHED = 6      # Gašenje motora
OFF = 7            # Motori isključeni

class SimulationDroneController(Node):
    def __init__(self):
        super().__init__('simulation_drone_controller')



        # --- KONFIGURACIJA ---

        self.declare_parameter("markers_to_visit", 2) 
        self.target_height = 2.0        
        self.target_dist = 0.6          
        self.xy_tolerance = 0.3         
        self.markers_to_visit = self.get_parameter("markers_to_visit").value      
        self.current_marker_id = None 
        self.visited_ids = set()
       
        # --- PUBLISHERS ---
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- SUBSCRIBERS ---
        self.create_subscription(PoseStamped, '/detected_marker_pose', self.marker_callback, 10)
        self.create_subscription(Int32, '/detected_marker_id', self.id_callback, 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)

        # --- VARIJABLE ---
        self.state = TAKEOFF
        self.current_pose = None     
        self.marker_pose_cam = None   
        self.last_marker_time = None  
        self.start_pos = None           
        self.visited_count = 0        

        self.current_vel_x = 0.0 
        self.smoothing_factor = 0.05 

        self.get_logger().info("Sim controller ready. Waiting for /odom...")
        
        self.timer = self.create_timer(0.05, self.control_loop)
    
        
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        
        _, _, yaw = tf_transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        
        self.current_pose = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw}

        if self.start_pos is None:
            self.start_pos = {'x': p.x, 'y': p.y, 'yaw': yaw}

    def id_callback(self, msg):
        self.current_marker_id = msg.data

    def marker_callback(self, msg):
        if self.current_marker_id is None:
            return

        if self.current_marker_id in self.visited_ids:
            return
        
        self.marker_pose_cam = msg.pose.position
        self.last_marker_time = self.get_clock().now()

    def control_loop(self):
        if self.current_pose is None:
            return

        cmd = Twist()

        
        
        if self.state == TAKEOFF:
            if self.current_pose['z'] < 0.5:
                cmd.linear.z = 0.8 
                self.get_logger().info("Liftoff...", throttle_duration_sec=1.0)
            else:
                self.state = ASCEND
                self.get_logger().info("Liftoff successful. Rising to 2.0m.")

        elif self.state == ASCEND:
            
            err_z = self.target_height - self.current_pose['z']
            
            if abs(err_z) > 0.1:
                # cmd.linear.z = 0.8 * err_z  
                
                cmd.linear.z = max(min(0.8 * err_z, 0.5), -0.5)
            else:
                self.get_logger().info(f"Height of 2.0m reached. Searching for markers.")
                cmd.linear.z = 0.0
                self.state = SEARCH

        elif self.state == SEARCH:
            
            cmd.linear.z = 0.5 * (self.target_height - self.current_pose['z'])
            
            cmd.angular.z = 0.4 
            
            
            if self.is_marker_visible():
                self.get_logger().info(f"Marker found, ID {self.current_marker_id}! Approaching.")
                self.state = APPROACH
                # cmd.angular.z = 0.0 

        elif self.state == APPROACH:
            if not self.is_marker_visible():
                self.get_logger().warn("Marker lost, repeating search...")
                self.state = SEARCH
                return


            
            dist_error = self.marker_pose_cam.z - self.target_dist
            yaw_error = -self.marker_pose_cam.x 
            
            
            if abs(dist_error) < 0.1:
                self.get_logger().info(f"Marker reached! ({self.visited_count + 1}/{self.markers_to_visit})")
                
                if self.current_marker_id is not None:
                    self.visited_ids.add(self.current_marker_id)
                    self.get_logger().info(f"Marker ID {self.current_marker_id} marked as visited!")
                    self.get_logger().info(f"Forbidden IDs: {self.visited_ids}")

                self.visited_count += 1
                

                if self.visited_count >= self.markers_to_visit:
                    self.state = RETURN_HOME
                else:
                    self.state = SEARCH
                    
                    
            else:
                
                # cmd.linear.x = 0.4 * dist_error      
                # cmd.angular.z = 0.8 * yaw_error      
                
                # Limitiranje brzina
                cmd.angular.z = max(min(0.8 * yaw_error , 0.4), -0.4)
                cmd.linear.x = max(min(0.4 * dist_error, 0.3), -0.3)
                
                self.current_vel_x = (self.smoothing_factor * cmd.linear.x) + \
                     ((1 - self.smoothing_factor) * self.current_vel_x)
                cmd.linear.x = self.current_vel_x
                
                cmd.linear.z = 0.5 * (self.target_height - self.current_pose['z'])
                

        elif self.state == RETURN_HOME:
            dx = self.start_pos['x'] - self.current_pose['x']
            dy = self.start_pos['y'] - self.current_pose['y']
            dist_home = math.hypot(dx, dy)
            
            if dist_home < self.xy_tolerance:
                self.get_logger().info("Arrived home. Landing...")
                self.state = LAND
            else:
                
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.current_pose['yaw']
                
                
                while angle_diff > math.pi: angle_diff -= 2*math.pi
                while angle_diff < -math.pi: angle_diff += 2*math.pi
                
                
                if abs(angle_diff) > 0.2:
                    # cmd.angular.z = 0.8 * angle_diff
                    cmd.angular.z = max(min(0.8 * angle_diff, 0.4), -0.4)
                else:
                    # cmd.linear.x = 0.4 * dist_home
                    cmd.linear.x = min(0.4 * dist_home, 0.5)
                    self.current_vel_x = (self.smoothing_factor * cmd.linear.x) + \
                     ((1 - self.smoothing_factor) * self.current_vel_x)
                    cmd.linear.x = self.current_vel_x
                    # cmd.angular.z = 0.5 * angle_diff      
                    cmd.angular.z = max(min(0.5 * angle_diff, 0.4), -0.4)
                
                
                cmd.linear.z = 0.5 * (self.target_height - self.current_pose['z'])

        elif self.state == LAND:
            angle_diff = self.start_pos['yaw'] - self.current_pose['yaw']
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi

            if abs(angle_diff) > 0.2:
                cmd.angular.z = max(min(0.8 * angle_diff, 0.4), -0.4)

            elif self.current_pose['z'] > 0.15:
                cmd.linear.z = -0.3  
                cmd.linear.x = 0.0   
                cmd.linear.y = 0.0
                self.get_logger().info(f"Landing... Height: {self.current_pose['z']:.2f}", throttle_duration_sec=1.0)
            else:
                self.state = FINISHED
        
        elif self.state == FINISHED:
            self.get_logger().info("Landing done. Motors turned off.")            
            cmd = Twist()
            self.state = OFF
        

        # self.get_logger().info(f"Linear X: {cmd.linear.x:.2f}, Y: {cmd.linear.y:.2f}, Z: {cmd.linear.z:.2f} | Angular Z: {cmd.angular.z:.2f}")
        self.vel_pub.publish(cmd)

    def is_marker_visible(self):
        if self.current_marker_id in self.visited_ids:
            return False
        if self.last_marker_time is None:
            return False
        
        return (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9 < 1.0

def main(args=None):
    rclpy.init(args=args)
    node = SimulationDroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()