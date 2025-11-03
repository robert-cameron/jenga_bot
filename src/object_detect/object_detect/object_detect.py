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
from tf2_geometry_msgs import do_transform_point

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
        marker_positions_image = {}
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
            marker_positions_image[marker_id] = tuple(center)

            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = f"aruco_{marker_id}"
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
        if not (1 in marker_positions and 2 in marker_positions):
            print("Markers 1 and 2 not both detected; skipping intersection computation.")
            return
        
        try:
            tf1 = self.tf_buffer.lookup_transform(
                "camera_color_optical_frame", "aruco_1", rclpy.time.Time()
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
        right    = R1[:, 0]   # +X axis (red in RViz)
        forward  = R1[:, 1]   # +Y axis (green in RViz)
        vertical = R1[:, 2]   # +Z axis (blue in RViz)



        # Marker positions (already in same world frame!)
        x1, y1, z1, *_ = marker_positions[1]
        x2, y2, z2, *_ = marker_positions[2]


        # Compute intersection
        direction = right + forward
        direction = direction / np.linalg.norm(direction)

        t = (y2 - y1) / direction[1]

        distance = np.sqrt(((x2-x1)**2 + (y2-y1)**2)/2)

        base_point = np.array([x1, y1, z1 ]) + right * distance
        base_x, base_y, base_z = base_point.tolist()

        self.get_logger().info(f"Intersection corrected: x={base_x:.3f}, y={base_y:.3f}, z={base_z:.3f}")
        # Broadcast TF for intersection point
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = "tower_base_vision"
        transform.transform.translation.x = base_x
        transform.transform.translation.y = base_y
        transform.transform.translation.z = base_z
        transform.transform.rotation.x = tf1.transform.rotation.x
        transform.transform.rotation.y = tf1.transform.rotation.y
        transform.transform.rotation.z = tf1.transform.rotation.z
        transform.transform.rotation.w = tf1.transform.rotation.w
        self.tf_broadcaster.sendTransform(transform)

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([35, 28, 77])
        upper_green = np.array([86, 170, 113])

        # Threshold the image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours of the green regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_height = 150  # You can adjust this value as needed
        lowest_point = max(marker_positions_image[1][1], marker_positions_image[2][1])
        left = min(marker_positions_image[1][0], marker_positions_image[2][0])
        right_image = max(marker_positions_image[1][0], marker_positions_image[2][0])
        area_top_left = (left - 20, lowest_point - area_height)
        area_bottom_right = (right_image + 20, lowest_point)

        cv2.rectangle(self.cv_image, area_top_left, area_bottom_right, (255, 0, 0), 2)

        tower_width = 0.08  # meters
        centre_offset = tower_width * 0.15  # meters
        left_offset = -0.0  # meters

        left_one =   base_point - right * tower_width/2 + forward * centre_offset + forward * left_offset
        left_two =   base_point - right * tower_width/2                           + forward * left_offset
        left_three = base_point - right * tower_width/2 - forward * centre_offset + forward * left_offset

        right_one = base_point - forward * tower_width/2 + right * centre_offset
        right_two = base_point - forward * tower_width/2
        right_three = base_point - forward * tower_width/2 - right * centre_offset

        positions = [left_one, left_two, left_three, right_one, right_two, right_three]

        count = 0
        for pos in positions:
            count += 1
            transformNew = TransformStamped()
            transformNew.header.stamp = self.get_clock().now().to_msg()
            transformNew.header.frame_id = "camera_color_optical_frame"
            transformNew.child_frame_id = f"place_{count}"
            x, y, z = pos.tolist()
            print(f"Place position {count}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            transformNew.transform.translation.x = x
            transformNew.transform.translation.y = y
            transformNew.transform.translation.z = z
            transformNew.transform.rotation.x = tf1.transform.rotation.x
            transformNew.transform.rotation.y = tf1.transform.rotation.y
            transformNew.transform.rotation.z = tf1.transform.rotation.z
            transformNew.transform.rotation.w = tf1.transform.rotation.w
            self.tf_broadcaster.sendTransform(transformNew)

        # Draw bounding boxes around detected green regions
        count = 0
        for cnt in contours:
            ix, iy, iw, ih = cv2.boundingRect(cnt)
            if not (ix >= area_top_left[0] and ix + iw <= area_bottom_right[0] and
                iy >= area_top_left[1] and iy + ih <= area_bottom_right[1]):
                continue

            center = (ix + iw // 2, iy + ih // 2)

            pos = self.pixel_2_global(center[::-1])
            if pos is None:
                continue

            y, x, z = pos
            point_camera = PointStamped()
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.header.frame_id = "camera_color_optical_frame"
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            count += 1

            obj_world = np.array([x, y, z])

            right_u   = right   / np.linalg.norm(right)
            forward_u = forward / np.linalg.norm(forward)
            vertical_u = vertical / np.linalg.norm(vertical)

            place_info = []  # list of dicts: {idx, r, f, u, world}
            for i, p in enumerate(positions):
                p = np.array(p)
                vec_p = p - base_point
                p_r = float(np.dot(vec_p, right_u))
                p_f = float(np.dot(vec_p, forward_u))
                p_u = float(np.dot(vec_p, vertical_u))
                place_info.append({"idx": i, "r": p_r, "f": p_f, "u": p_u, "world": p})

            # Sort placements by r (left <-> right) and split by median r
            place_info_sorted_by_r = sorted(place_info, key=lambda x: x["r"])
            # lower 3 -> one side, upper 3 -> other side (works regardless of sign)
            left_side = place_info_sorted_by_r[:3]
            right_side = place_info_sorted_by_r[3:]

            # On each side, sort by forward (f) descending so front-most -> index 1
            left_side_sorted = sorted(left_side, key=lambda x: x["f"], reverse=True)
            right_side_sorted = sorted(right_side, key=lambda x: x["f"], reverse=True)

            # Build mapping: slot_name -> placement dict
            slot_map = {}
            for i, slot in enumerate(left_side_sorted):
                # left_1 = front-most, left_3 = back-most
                slot_map[f"left_{i+1}"] = slot
                slot["slot_name"] = f"left_{i+1}"
            for i, slot in enumerate(right_side_sorted):
                slot_map[f"right_{i+1}"] = slot
                slot["slot_name"] = f"right_{i+1}"

            # Debug: print mapping
            self.get_logger().info("Placement mapping (slot -> r,f,u):")
            for name, info in slot_map.items():
                self.get_logger().info(f"{name}: r={info['r']:.3f}, f={info['f']:.3f}, u={info['u']:.3f}, idx={info['idx']}")

            # Project the detected object into marker frame
            vec_obj = obj_world - base_point
            obj_r = float(np.dot(vec_obj, right_u))
            obj_f = float(np.dot(vec_obj, forward_u))
            obj_u = float(np.dot(vec_obj, vertical_u))
            obj_rf = np.array([obj_r, obj_f])

            # Find the closest placement by RF distance
            best_name = None
            best_info = None
            best_dist = float("inf")
            for name, info in slot_map.items():
                prf = np.array([info["r"], info["f"]])
                d = np.linalg.norm(obj_rf - prf)
                if d < best_dist:
                    best_dist = d
                    best_name = name
                    best_info = info

            # Height above chosen placement (positive = object higher than placement)
            height_above = obj_u - best_info["u"]

            self.get_logger().info(f"Detected assigned to: {best_name}, RF-dist={best_dist:.4f}, height_above={height_above:.4f}")

            # Color-coding by slot (optional)
            # produce a color unique per slot (example mapping)
            color_map = {
                "left_1": (255,0,255), # magenta
                "left_2": (0,255,255), # cyan
                "left_3": (255,255,0), # yellow
                "right_1": (255,0,0), # red
                "right_2": (0,255,0), # green
                "right_3": (0,0,255), # blue
            }
            color = color_map.get(best_name, (0,255,0))
            cv2.rectangle(self.cv_image, (ix, iy), (ix + iw, iy + ih), color, 2)
            cv2.putText(self.cv_image, f"{best_name}", (ix, iy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # OPTIONAL: draw small labels for all placement slots on the image to debug
            for name, info in slot_map.items():
                # convert world point back to pixel to annotate -- you already have intrinsics/depth -> use project
                # Using rs2_project_point_to_pixel requires point in camera coords; we only have world (camera frame), so:
                # If base_point and positions are in camera frame (they are), we can project directly:
                px = int(np.round(self.intrinsics.ppx + (info["world"][0] * self.intrinsics.fx / max(info["world"][2], 1e-6))))
                py = int(np.round(self.intrinsics.ppy + (info["world"][1] * self.intrinsics.fy / max(info["world"][2], 1e-6))))
                cv2.circle(self.cv_image, (px, py), 4, (0,0,0), -1)
                cv2.putText(self.cv_image, name, (px+4, py-4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1)

            # OPTIONAL: draw marker axes (right=red, forward=green, up=blue) from base_point
            axis_scale_px = 80  # length to draw on image
            base_px = ( int(np.round(self.intrinsics.ppx + (base_point[0]*self.intrinsics.fx / max(base_point[2],1e-6)))),
                        int(np.round(self.intrinsics.ppy + (base_point[1]*self.intrinsics.fy / max(base_point[2],1e-6)))) )
            r_end_world = base_point + right_u * 0.08
            f_end_world = base_point + forward_u * 0.08
            u_end_world = base_point + vertical_u * 0.08
            def world_to_px(world_pt):
                return ( int(np.round(self.intrinsics.ppx + (world_pt[0]*self.intrinsics.fx / max(world_pt[2],1e-6)))),
                        int(np.round(self.intrinsics.ppy + (world_pt[1]*self.intrinsics.fy / max(world_pt[2],1e-6)))) )
            r_px = world_to_px(r_end_world)
            f_px = world_to_px(f_end_world)
            u_px = world_to_px(u_end_world)
            cv2.line(self.cv_image, base_px, r_px, (0,0,255), 2)  # right=red
            cv2.line(self.cv_image, base_px, f_px, (0,255,0), 2)  # forward=green
            cv2.line(self.cv_image, base_px, u_px, (255,0,0), 2)  # up=blue


        cv2.imshow('Image', self.cv_image)
        cv2.waitKey(1)


        
            
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