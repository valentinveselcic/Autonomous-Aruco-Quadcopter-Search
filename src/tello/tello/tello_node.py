#!/usr/bin/env python3

import pprint
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

from .tello import Tello

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# Tello ROS node class, inherits from the Tello controller object.
#
# Can be configured to be used by multiple drones, publishes, all data collected from the drone and provides control using ROS messages.
class TelloNode(Node):
    def __init__(self):
        # ROS node
        super().__init__("tello_node")

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
                self.get_logger().info('Loaded Camera information YAML' + self.camera_info.__str__())
        else:
            self.get_logger().error("Camera info YAML not specified!")

        # Configure drone connection
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        # Connect to drone
        self.get_logger().info('Connecting to drone!')

        self.tello = Tello()
        self.tello.connect()

        self.get_logger().info('Connected to drone!')

        #Internal status variables
        self.status = None
        self.dr_x = 0.0
        self.dr_y = 0.0
        self.dr_z = 0.0
        self.dr_time = None

        # Enable tello stream
        self.tello.streamon()
        self.video_frame_read = self.tello.get_frame_read()
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
        self.pub_temperature = self.create_publisher(Temperature, 'temperature', 1, callback_group=self.pub_cb)
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
        self.sub_wifi_config = self.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 1, callback_group=self.sub_cb)

    # Setup timers for publishers and for updating status from tello
    def setup_timers(self):
        self.t_cb = ReentrantCallbackGroup()
        self.status_update_timer = self.create_timer(0.1, self.update_status_cb, callback_group=self.t_cb) #Getting status from Tello is maximum 10Hz from the physical drone
        self.full_status_pub_timer = self.create_timer(1 / self.full_status_update_rate, self.pub_full_status_cb, callback_group=self.t_cb)
        self.temp_and_bat_pub_timer = self.create_timer(1 / self.temp_bat_update_rate, self.pub_temp_bat_cb, callback_group=self.t_cb)
        self.odom_pub_timer = self.create_timer(1 / self.odom_update_rate, self.pub_odom_cb, callback_group=self.t_cb)
        self.camera_pub_timer = self.create_timer(1 / self.camera_update_rate, self.pub_camera_cb, callback_group=self.t_cb)
    
    def update_status_cb(self):
        self.status = copy.deepcopy(self.tello.get_current_state())
        #self.get_logger().info('Status recieved!')

    def pub_full_status_cb(self):
        if self.status == None:
            return
        # Tello Status
        msg = TelloStatus()
        msg.acceleration.x = float(self.status['agx'])
        msg.acceleration.y = float(self.status['agy'])
        msg.acceleration.z = float(self.status['agz'])

        msg.speed.x = float(self.status['vgx'])
        msg.speed.y = float(self.status['vgy'])
        msg.speed.z = float(self.status['vgz'])

        msg.pitch = int(self.status['pitch'])
        msg.roll = int(self.status['roll'])
        msg.yaw = int(self.status['yaw'])

        msg.barometer = int(self.status['baro'])
        msg.distance_tof = int(self.status['tof'])

        msg.fligth_time = int(self.status['time'])

        msg.battery = int(self.status['bat'])

        msg.highest_temperature = int(self.status['temph'])
        msg.lowest_temperature = int(self.status['templ'])
        msg.temperature = float(msg.lowest_temperature + msg.highest_temperature) / 2.0

        #msg.wifi_snr = self.tello.query_wifi_signal_noise_ratio()

        self.pub_status.publish(msg)

        # # Tello ID
        # msg = TelloID()
        # msg.sdk_version = self.tello.query_sdk_version()
        # msg.serial_number = self.tello.query_serial_number()
        # self.pub_id.publish(msg)

    def pub_temp_bat_cb(self):
        if self.status == None:
            return
        # Battery
        msg = BatteryState()
        msg.header.frame_id = self.tf_drone
        msg.percentage = float(self.status['bat']) 
        # msg.voltage = 3.8
        # msg.design_capacity = 1.1
        msg.present = True
        msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
        msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(msg)

        # Temperature
        templ = float(self.status['templ'])
        temph = float(self.status['temph'])
        msg = Temperature()
        msg.header.frame_id = self.tf_drone
        msg.temperature = (templ + temph) / 2.0
        #msg.variance = 0.0
        self.pub_temperature.publish(msg)

    def pub_odom_cb(self):
        if self.status == None:
            return
        #Calculate dead reckoning position
        time_diff = 1.0
        if not self.dr_time == None:
            time_diff = (self.status['received_at'] - self.dr_time).total_seconds()
        self.dr_x += float(self.status['vgx']) * time_diff / 100.0 #velocity is in cm, need to convert to meters
        self.dr_y += float(self.status['vgy']) * time_diff / 100.0
        self.dr_z += float(self.status['vgz']) * time_diff / 100.0
        self.dr_time = self.status['received_at']

        q = euler_to_quaternion([
            math.radians(float(self.status['yaw'])),
            math.radians(float(self.status['pitch'])),
            math.radians(float(self.status['roll']))
        ])
        # TF
        if self.tf_pub:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.tf_base
            t.child_frame_id = self.tf_drone
            t.transform.translation.x = self.dr_x
            t.transform.translation.y = self.dr_y
            t.transform.translation.z = self.dr_z #Could use tof values if the ground is always the same?
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)
                
        # IMU
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone
        msg.linear_acceleration.x = float(self.status['agx']) / 100.0
        msg.linear_acceleration.y = float(self.status['agy']) / 100.0
        msg.linear_acceleration.z = float(self.status['agz']) / 100.0
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        self.pub_imu.publish(msg)

        # Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.tf_base
        odom_msg.pose.pose.position.x = self.dr_x
        odom_msg.pose.pose.position.x = self.dr_y
        odom_msg.pose.pose.position.x = self.dr_z
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = float(self.status['vgx']) / 100.0
        odom_msg.twist.twist.linear.y = float(self.status['vgy']) / 100.0
        odom_msg.twist.twist.linear.z = float(self.status['vgz']) / 100.0
        self.pub_odom.publish(odom_msg)

    def pub_camera_cb(self):
        #Publish camera info
        if not self.camera_info == None:
            msg = CameraInfo()
            msg.height = self.camera_info.image_height
            msg.width = self.camera_info.image_width
            msg.distortion_model = self.camera_info.distortion_model
            msg.D = self.camera_info.distortion_coefficients
            msg.K = self.camera_info.camera_matrix
            msg.R = self.camera_info.rectification_matrix
            msg.P = self.camera_info.projection_matrix
            self.pub_camera_info.publish(msg)

        #Publish camera image raw
        # Get frame from drone
        frame = self.video_frame_read.frame
        # Publish opencv frame using CV bridge
        msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'rgb8')
        msg.header.frame_id = self.tf_drone
        self.pub_image_raw.publish(msg)

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.node.get_logger().error(str(err))
        self.tello.end()
        rclpy.shutdown()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.emergency()

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
    def cb_wifi_config(self, msg):
        self.tello.set_wifi_credentials(msg.ssid, msg.password)
    
    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        self.tello.flip(msg.data)

# Convert a rotation from euler to quaternion.
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# Convert rotation from quaternion to euler.
def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def main(args=None):
    rclpy.init(args=args)

    drone = TelloNode()

    rclpy.spin(drone, MultiThreadedExecutor())

    drone.cb_shutdown()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
