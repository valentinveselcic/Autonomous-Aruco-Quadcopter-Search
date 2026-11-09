#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32

class ArucoPoseDetector(Node):
    def __init__(self):
        super().__init__('aruco_pose_detector')

        # --- PARAMETRI ---
        self.declare_parameter("marker_size", 0.3) 
        self.marker_size = self.get_parameter("marker_size").value
        
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        
        # --- TF BUFFER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        # --- PUBLISHER ---
        self.marker_pub = self.create_publisher(PoseStamped, '/detected_marker_pose', 10)
        self.id_pub = self.create_publisher(Int32, '/detected_marker_id', 10)
        # --- CV SETUP ---
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.valid_ids = [23, 42] # Prevencija povremenog detektiranja ideove u patternu zida, moguce je implementirati
                                  # da se gleda npr 10 frameova u kojima marker mora biti vidljiv da bi se smatrao validnim
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
                                  
        self.aruco_params = aruco.DetectorParameters_create()

        self.create_subscription(CameraInfo, 
                                 self.get_parameter("camera_info_topic").value, 
                                 self.info_callback, 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, 
                                 self.get_parameter("camera_topic").value, 
                                 self.image_callback, qos)

        self.get_logger().info(f"Aruco Detector ready. Searching for markers of size {self.marker_size}m...")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera Info successfully loaded.")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detekcija
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                if ids[i][0] not in self.valid_ids: # Ignorira fake markere
                    continue
                # --- PRIPREMA POZE ---
                t = TransformStamped()
                current_id = int(ids[i][0])
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "rgb_camera_optical_frame" 
                t.child_frame_id = f"aruco_marker_{ids[i][0]}"

                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]

                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                q = self.rotation_matrix_to_quaternion(rot_matrix)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.tf_broadcaster.sendTransform(t)

                pose_msg = PoseStamped()
                pose_msg.header = t.header
                pose_msg.pose.position.x = t.transform.translation.x
                pose_msg.pose.position.y = t.transform.translation.y
                pose_msg.pose.position.z = t.transform.translation.z
                pose_msg.pose.orientation = t.transform.rotation

                id_msg = Int32()
                id_msg.data = current_id
                self.id_pub.publish(id_msg)
                self.marker_pub.publish(pose_msg)
  

                if hasattr(cv2, 'drawFrameAxes'):
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.2) 
                else:
                    aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.2)

          

            aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("Aruco View", cv_image)
        cv2.waitKey(1)

    def rotation_matrix_to_quaternion(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()