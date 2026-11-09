#!/usr/bin/env python3

import math
import rclpy
import threading
import numpy
import time
import av
import tf2_ros
from tf_transformations import quaternion_from_euler
import cv2
import time
import yaml
import copy
from datetime import timedelta, datetime

from . import tello_ctrl as tc

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

#Tello node
class TelloNode(Node):
    def __init__(self):
        # ROS node
        super().__init__("tello_node_tc")

        # Declare parameters
        self.declare_parameter('connect_timeout', 15.0)
        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('tf_base', 'odom')
        self.declare_parameter('tf_drone', 'drone')
        self.declare_parameter('tf_pub', True)
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('camera_update_rate', 15.0)
        self.declare_parameter('full_status_update_rate', 1.0)
        self.declare_parameter('temp_bat_update_rate', 10.0)
        self.declare_parameter('odom_update_rate', 10.0)

        # Get parameters
        self.connect_timeout = float(self.get_parameter('connect_timeout').value)
        self.tello_ip = str(self.get_parameter('tello_ip').value)
        self.tf_base = str(self.get_parameter('tf_base').value)
        self.tf_drone = str(self.get_parameter('tf_drone').value)
        self.tf_pub = bool(self.get_parameter('tf_pub').value)
        self.camera_info_file = str(self.get_parameter('camera_info_file').value)
        self.camera_update_rate = float(self.get_parameter('camera_update_rate').value)
        self.full_status_update_rate = float(self.get_parameter('full_status_update_rate').value)
        self.temp_bat_update_rate = float(self.get_parameter('temp_bat_update_rate').value)
        self.odom_update_rate = float(self.get_parameter('odom_update_rate').value)

        # Camera information loaded from calibration yaml
        self.camera_info = None
        
        # Check if camera info file was received as argument and load file if it has
        if len(self.camera_info_file) > 0:
            # share_directory = ament_index_python.get_package_share_directory('tello')
            # self.camera_info_file = share_directory + '/ost.yaml'
            with open(self.camera_info_file, 'r') as file:
                self.camera_info = yaml.load(file, Loader=yaml.FullLoader)
                #self.get_logger().info('Loaded Camera information YAML' + self.camera_info.__str__())
        else:
            self.get_logger().error("Camera info YAML not specified!")

        # Connect to drone
        self.get_logger().info('Connecting to drone!')
        # create a tello_ctrl object
        self.tello = tc.tello_ctrl(ip_address=self.tello_ip)
        # Connect to the drone (assuming the base station is connected to the drone's WIFI)
        self.tello.connect(self.connect_timeout)
        self.get_logger().info('Connected to drone!')

        #Internal status variables
        self.flight_data = None

        # Enable tello video stream and set exposure
        self.get_logger().info('Starting video stream.')
        self.tello.start_receiving_video(video_format='rgb24')
        self.tello.set_video_exposure(5)
        
        # OpenCV bridge
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timers()

        self.get_logger().info('Tello node ready!')

    # Setup ROS publishers of the node.
    def setup_publishers(self):
        self.pub_cb = ReentrantCallbackGroup()
        self.pub_image_raw = self.create_publisher(Image, 'camera/image_raw', 1, callback_group=self.pub_cb)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'camera/camera_info', 1, callback_group=self.pub_cb)
        self.pub_status = self.create_publisher(TelloStatus, 'tello_status', 1, callback_group=self.pub_cb)
        #self.pub_id = self.create_publisher(TelloID, 'tello_id', 1, callback_group=self.pub_cb)
        self.pub_imu = self.create_publisher(Imu, 'imu', 1, callback_group=self.pub_cb)
        self.pub_battery = self.create_publisher(BatteryState, 'battery', 1, callback_group=self.pub_cb)
        #self.pub_temperature = self.create_publisher(Temperature, 'temperature', 1, callback_group=self.pub_cb)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 1, callback_group=self.pub_cb)

        # TF broadcaster
        if self.tf_pub:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
    
    # Setup the topic subscribers of the node.
    def setup_subscribers(self):
        self.sub_cb = ReentrantCallbackGroup()
        self.sub_emergency = self.create_subscription(Empty, 'emergency', self.cb_emergency, 1, callback_group=self.sub_cb)
        self.sub_takeoff = self.create_subscription(Empty, 'takeoff', self.cb_takeoff, 1, callback_group=self.sub_cb)
        self.sub_land = self.create_subscription(Empty, 'land', self.cb_land, 1, callback_group=self.sub_cb)
        self.sub_control = self.create_subscription(Twist, 'cmd_vel', self.cb_control, 1, callback_group=self.sub_cb)
        self.sub_flip = self.create_subscription(String, 'flip', self.cb_flip, 1, callback_group=self.sub_cb)
        #self.sub_wifi_config = self.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 1, callback_group=self.sub_cb) #NOT AVAILABLE IN THIS LIBARRY

    # Setup timers for publishers and for updating status from tello
    def setup_timers(self):
        self.t_cb = ReentrantCallbackGroup()
        self.status_update_timer = self.create_timer(0.1, self.update_status_cb, callback_group=self.t_cb) #Getting status from Tello is maximum 10Hz from the physical drone
        self.full_status_pub_timer = self.create_timer(1 / self.full_status_update_rate, self.pub_full_status_cb, callback_group=self.t_cb)
        self.temp_and_bat_pub_timer = self.create_timer(1 / self.temp_bat_update_rate, self.pub_temp_bat_cb, callback_group=self.t_cb)
        self.odom_pub_timer = self.create_timer(1 / self.odom_update_rate, self.pub_odom_cb, callback_group=self.t_cb)
        self.camera_pub_timer = self.create_timer(1 / self.camera_update_rate, self.pub_camera_cb, callback_group=self.t_cb)
    
    def update_status_cb(self):
        self.flight_data = copy.deepcopy(self.tello._tello_ctrl__flight_data) 
        #self.get_logger().info('Status recieved!')

    def pub_full_status_cb(self):
        if self.flight_data == None:
            return
        
        # Tello Status
        msg = TelloStatus()
        msg.acceleration.x = float(self.flight_data.accX)
        msg.acceleration.y = float(self.flight_data.accY)
        msg.acceleration.z = float(self.flight_data.accZ)

        #Using Extended Inertial Measurement Unit measurements combines Inertial Measurement Unit and Monocular Vision Odometry
        msg.speed.x = float(self.flight_data.velX_VO)
        msg.speed.y = float(self.flight_data.velY_VO)
        msg.speed.z = float(self.flight_data.velZ_VO)

        msg.pitch = int(math.degrees(self.flight_data.pitch)) #Pitch, roll any yaw only gets 0???
        msg.roll = int(math.degrees(self.flight_data.roll))
        msg.yaw = int(math.degrees(self.flight_data.yaw))

        msg.barometer = int(self.flight_data.baro_smooth)
        msg.distance_tof = int(self.flight_data.tof)

        msg.fligth_time = int(self.flight_data.fly_time)

        msg.battery = int(self.flight_data.battery_percentage)

        msg.highest_temperature = int(-1) # Temperature not available in this library
        msg.lowest_temperature = int(-1)
        msg.temperature = float(-1.0)

        msg.wifi_snr = str(self.flight_data.wifi_strength)

        self.pub_status.publish(msg)

    def pub_temp_bat_cb(self):
        if self.flight_data == None:
            return
        # Battery
        msg = BatteryState()
        msg.header.frame_id = self.tf_drone
        msg.percentage = float(self.flight_data.battery_percentage) 
        msg.voltage = 3.8
        msg.design_capacity = 1.1
        msg.present = True
        msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
        msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(msg)


        # # Temperature  # Temperature not available in this library
        # templ = float(self.status['templ'])
        # temph = float(self.status['temph'])
        # msg = Temperature()
        # msg.header.frame_id = self.tf_drone
        # msg.temperature = (templ + temph) / 2.0
        # #msg.variance = 0.0
        # self.pub_temperature.publish(msg)

    def pub_odom_cb(self):
        if self.flight_data == None:
            return

        q = (self.flight_data.qX, self.flight_data.qY, self.flight_data.qZ, self.flight_data.qW)
        # TF
        if self.tf_pub:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.tf_base
            t.child_frame_id = self.tf_drone
            t.transform.translation.x = -float(self.flight_data.posX_VO)
            t.transform.translation.y = -float(self.flight_data.posY_VO)
            t.transform.translation.z = -float(self.flight_data.tof)#posZ_VO gives strange values with offset?
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)
                
        # IMU
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone
        msg.linear_acceleration.x = float(self.flight_data.accX)
        msg.linear_acceleration.y = float(self.flight_data.accX)
        msg.linear_acceleration.z = float(self.flight_data.accX)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        self.pub_imu.publish(msg)

        # Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.tf_base
        odom_msg.pose.pose.position.x = -float(self.flight_data.posX_VO)
        odom_msg.pose.pose.position.y = -float(self.flight_data.posY_VO)
        odom_msg.pose.pose.position.z = -float(self.flight_data.tof)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = float(self.flight_data.velX_VO)
        odom_msg.twist.twist.linear.y = float(self.flight_data.velY_VO)
        odom_msg.twist.twist.linear.z = float(self.flight_data.velZ_VO)
        self.pub_odom.publish(odom_msg)

    def pub_camera_cb(self):
        #Publish camera info
        if not self.camera_info == None:
            msg = CameraInfo()
            msg.width = self.camera_info["image_width"]
            msg.height = self.camera_info["image_height"]
            msg.distortion_model = self.camera_info["distortion_model"]
            msg.k = self.camera_info["camera_matrix"]["data"]
            msg.d = self.camera_info["distortion_coefficients"]["data"]
            msg.r = self.camera_info["rectification_matrix"]["data"]
            msg.p = self.camera_info["projection_matrix"]["data"] 
            msg.header.frame_id = self.tf_drone    
            self.pub_camera_info.publish(msg)

        #Publish camera image raw
        # Get frame from drone
        frame = self.tello.get_frame()
        # Publish opencv frame using CV bridge
        msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'rgb8')
        msg.header.frame_id = self.tf_drone
        self.pub_image_raw.publish(msg)

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.get_logger().error(str(err))
        self.tello.land()
        # stop receiving video
        self.tello.stop_receiving_video()
        # clean up
        self.tello.quit()
        #ROS
        rclpy.shutdown()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.move_forward(0)
        self.tello.move_right(0)
        self.tello.move_clockwise(0)
        
    # Drone takeoff message control
    def cb_takeoff(self, msg):
        self.tello.takeoff()

    # Land the drone message callback
    def cb_land(self, msg):
        self.tello.land()

    # Control messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    #
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def cb_control(self, msg):
        self.tello.send_rc_control(int(msg.linear.y), int(msg.linear.x), int(msg.linear.z), int(msg.angular.z))

    # Configure the wifi credential that should be used by the drone.
    #
    # The drone will be restarted after the credentials are changed.
    #NOT AVAILABLE IN THIS LIBRARY
    # def cb_wifi_config(self, msg):
    #     self.tello.set_wifi_credentials(msg.ssid, msg.password)
    
    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        if msg.data == "f":
            self.tello.flip_forward(msg.data)
        elif msg.data == "b":
            self.tello.flip_back(msg.data)
        elif msg.data == "l":
            self.tello.flip_left(msg.data)
        elif msg.data == "r":
            self.tello.flip_right(msg.data)

def main(args=None):
    rclpy.init(args=args)

    drone = TelloNode()

    rclpy.spin(drone, MultiThreadedExecutor())

    drone.cb_shutdown()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
