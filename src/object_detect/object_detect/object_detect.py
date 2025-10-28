import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class objectDetect(Node):

    def __init__(self):
        super().__init__('object_detect')
        # depth camera subscriptions
        self.image_sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.arm_image_callback, 10)
        self.point_cloud_sub = self.create_subscription( Image, '/camera/camera/aligned_depth_to_color/image_raw', self.arm_point_cloud_callback, 10)
        self.cam_info_sub = self.create_subscription( CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.arm_image_depth_info_callback,10)
        self.intrinsics = None
        self.depth_image = None

        # Timer definitions
        self.routine_timer = self.create_timer(0.05, self.routine_callback)

        # Transformation Interface
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # General Variables
        self.cv_image = None
        self.mask = None
        self.cv_bridge = CvBridge()

        # ArUco dictionary & parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        print("Object Detection Node Initialized")



    def arm_image_depth_info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return


    # This gets bgr image from the image topic and finds where green in the image is
    def arm_image_callback(self, msg):      
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
        except Exception as e:
            self.get_logger().error(f"Error in arm_image_callback: {str(e)}")

        

    # This gets depth_frame aligned with RGB image
    def arm_point_cloud_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                
        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")


    def pixel_2_global(self, pixel_pt):
        print(f"Pixel Point: {pixel_pt}")
        print(f"Depth Image: {self.depth_image.shape}, Intrinsics: {self.intrinsics is not None}")
        try:
            if self.depth_image is not None and self.intrinsics is not None:
                [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), self.depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
                return [x, y,z]
            else:
                return None

        except Exception as e:
            self.get_logger().error(f"Error in pixel_2_global: {str(e)}")
            cv2.imshow('Image', self.cv_image)
            cv2.waitKey(1)
            return None

    def routine_callback(self):
        """Detect ArUco markers and broadcast TF for each one."""
        if self.cv_image is None:
            return

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(self.cv_image, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            pts = corners[i][0]
            center = np.mean(pts, axis=0).astype(int)
            cv2.circle(self.cv_image, tuple(center), 4, (0, 255, 0), -1)
            print(f"Detected marker ID: {marker_id} at pixel {center}")
            print(pts)
            # Get 3D position
            pos = self.pixel_2_global(center[::-1])
            if pos is None:
                continue

            y, x, z = pos
            self.get_logger().info(f"Marker {marker_id}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = f"aruco_{marker_id}_frame"
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)

    def routine_callback_old(self):

        if (self.cv_image is None):
            return
        

        # TODO: COLOUR MASK TO FIND THE CENTER OF AN OBJECT OF INTEREST

        item_img_global = self.pixel_2_global([360, 240])

        if (item_img_global is None):
            return
        
        x = item_img_global[0] - 0.038 # OFFSET TO ACCOUNT FOR CAMERA OFFSET
        y = item_img_global[1]
        z = item_img_global[2] 
        print(f"x {x} y {y} z {z}")



        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "camera_color_optical_frame"
        transform_stamped.child_frame_id = "blue_object_frame"

        # TODO: COMPLETE TRANSFORMATION OUTPUT

        self.tf_broadcaster.sendTransform(transform_stamped)


def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()


if __name__ == '__main__':
    main()