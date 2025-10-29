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
        
        marker_positions = {}
        marker_length = 0.05  # <-- meters (set this to your marker’s real side length)

        # Convert intrinsics to OpenCV format
        cameraMatrix = np.array([
            [self.intrinsics.fx, 0, self.intrinsics.ppx],
            [0, self.intrinsics.fy, self.intrinsics.ppy],
            [0, 0, 1]
        ])
        distCoeffs = np.zeros(5)  # or use real distortion coefficients if known

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, cameraMatrix, distCoeffs)


        for i, marker_id in enumerate(ids.flatten()):
            pts = corners[i][0]
            rvec = rvecs[i][0]
            center = np.mean(pts, axis=0).astype(int)
            
            R, _ = cv2.Rodrigues(rvec)
            quat = self.rotation_matrix_to_quaternion(R)

            cv2.circle(self.cv_image, tuple(center), 4, (0, 255, 0), -1)
            print(f"Detected marker ID: {marker_id} at pixel {center}")
            print(pts)
            # Get 3D position
            pos = self.pixel_2_global(center[::-1])
            if pos is None:
                continue

            y, x, z = pos
            self.get_logger().info(f"Marker {marker_id}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            marker_positions[marker_id] = (x,y,z, quat[0], quat[1], quat[2], quat[3])

            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = f"new_{marker_id}_frame"
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(transform)

        print(marker_positions)

        # Compute intersection only if marker 1 and 2 are found
        if 1 in marker_positions and 2 in marker_positions:
            try:
                tf1 = self.tf_buffer.lookup_transform(
                    "camera_color_optical_frame", "new_1_frame", rclpy.time.Time()
                )
            except:
                self.get_logger().warn("TF for marker 1 not available yet.")
                return

            # Rotation matrix in WORLD FRAME
            quat = [
                tf1.transform.rotation.x,
                tf1.transform.rotation.y,
                tf1.transform.rotation.z,
                tf1.transform.rotation.w
            ]
            R1 = self.quaternion_to_rotation_matrix(quat)[0:3, 0:3]

            # World-aligned marker axes
            right = R1[:, 0]   # +X axis (red in RViz)
            up    = R1[:, 1]   # +Y axis (green in RViz)



            # Marker positions (already in same world frame!)
            x1, y1, z1, *_ = marker_positions[1]
            x2, y2, z2, *_ = marker_positions[2]


            # Compute intersection
            direction = right + up
            direction = direction / np.linalg.norm(direction)

            t = (y2 - y1) / direction[1]
            ix = x1 + t * direction[0]
            iy = y1 + t * direction[1]
            iz = z1 + t * direction[2]

            distance = np.sqrt(((x2-x1)**2 + (y2-y1)**2)/2)

            p_new = np.array([x1, y1, z1]) + right * distance
            new_x, new_y, new_z = p_new.tolist()

            self.get_logger().info(f"Intersection corrected: x={ix:.3f}, y={iy:.3f}, z={iz:.3f}")
            # Broadcast TF for intersection point
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = "intersection_point"
            transform.transform.translation.x = new_x
            transform.transform.translation.y = new_y
            transform.transform.translation.z = new_z
            transform.transform.rotation.x = tf1.transform.rotation.x
            transform.transform.rotation.y = tf1.transform.rotation.y
            transform.transform.rotation.z = tf1.transform.rotation.z
            transform.transform.rotation.w = tf1.transform.rotation.w
            self.tf_broadcaster.sendTransform(transform)
            
    def rotation_matrix_to_quaternion(self, R):
        """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
        qw = np.sqrt(1 + np.trace(R)) / 2
        qx = (R[2,1] - R[1,2]) / (4*qw)
        qy = (R[0,2] - R[2,0]) / (4*qw)
        qz = (R[1,0] - R[0,1]) / (4*qw)
        return [qx, qy, qz, qw]

    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),   1-2*(x*x+z*z),     2*(y*z - x*w)],
            [2*(x*z - y*w),   2*(y*z + x*w),     1-2*(x*x+y*y)]
        ])
        return R

def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()


if __name__ == '__main__':
    main()