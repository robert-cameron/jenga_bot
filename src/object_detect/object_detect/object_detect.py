import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
import math
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point, PoseArray, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
from scipy.cluster.vq import kmeans2
from scipy.cluster.hierarchy import linkage, fcluster

from collections import deque

block_length = 0.075  # meters
block_width = 0.025  # meters
block_height = 0.015  # meters

# Used for vision, thus the width is greater
tower_width = 0.13  # meters
centre_offset = tower_width * 0.26  # meters
left_offset = -0.017  # meters

def map_value(value, in_min, in_max, out_min, out_max):
    # Figure out how 'wide' each range is
    left_span = in_max - in_min
    right_span = out_max - out_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - in_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return out_min + (value_scaled * right_span)


def solveMarkerPositons(corners, marker_length, cameraMatrix, distCoeffs):
    rvecs = []
    tvecs = []

    for corner in corners:
        # Each `corner` is 4x2 array of the marker’s corners in pixels
        # Define the 3D points of the marker corners in its local frame
        obj_points = np.array([
            [-marker_length/2,  marker_length/2, 0],
            [ marker_length/2,  marker_length/2, 0],
            [ marker_length/2, -marker_length/2, 0],
            [-marker_length/2, -marker_length/2, 0]
        ], dtype=np.float32)

        img_points = corner[0].astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, cameraMatrix, distCoeffs)
        if success:
            rvecs.append(rvec)
            tvecs.append(tvec)

    rvecs = np.array(rvecs, dtype=np.float32)
    tvecs = np.array(tvecs, dtype=np.float32)
    return rvecs, tvecs

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
        self.cv_image_raw = None
        self.mask = None
        self.cv_bridge = CvBridge()

        # ArUco dictionary & parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Publishers
        self.block_marker_pub = self.create_publisher(MarkerArray, 'vision/markers', 10)
        self.blocks_publisher = self.create_publisher(PoseArray, 'vision/blocks', 10)

        self.k = 100  # size of smoothing window
        self.base_history = deque(maxlen=self.k)
        self.rot_history = deque(maxlen=self.k)      # rotation smoothing


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
            self.cv_image_raw = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
        except Exception as e:
            self.get_logger().error(f"Error in arm_image_callback: {str(e)}")

        

    # This gets depth_frame aligned with RGB image
    def arm_point_cloud_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                
        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")

    def transform_point_to_camera(self, point, from_frame="tower_base"):
        # point = (x,y,z)
        ps = PointStamped()
        ps.header.frame_id = from_frame
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.point.x, ps.point.y, ps.point.z = point

        ps_out = self.tf_buffer.transform(
            ps, "camera_color_optical_frame", timeout=rclpy.duration.Duration(seconds=0.3)
        )
        return np.array([ps_out.point.x, ps_out.point.y, ps_out.point.z])
    
    def global_2_pixel(self, point_3d_camera_frame):
        # point_3d_camera_frame = [X, Y, Z] in camera optical frame (meters)
        pixel = rs.rs2_project_point_to_pixel(
            self.intrinsics,
            [point_3d_camera_frame[0], point_3d_camera_frame[1], point_3d_camera_frame[2]]
        )
        # pixel returned as (u, v) = (x_pixel, y_pixel)
        return int(pixel[0]), int(pixel[1])

    def pixel_2_global(self, pixel_pt):
        try:
            if self.depth_image is not None and self.intrinsics is not None:
                [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), self.depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
                return [x, y,z]
            else:
                return None

        except Exception as e:
            self.get_logger().error(f"Error in pixel_2_global: {str(e)}")
            return None
        
    def calculate_tower(self):
        try:
            """Detect ArUco markers and broadcast TF for each one."""
            if self.cv_image_raw is None:
                return
            
            self.cv_image = self.cv_image_raw.copy()

            # Detect ArUco markers
            corners, ids, _ = self.aruco_detector.detectMarkers(self.cv_image)
            if ids is None:
                return
            
            marker_positions = {}
            marker_positions_image = {}
            marker_length = 0.1  # <-- meters (set this to your marker’s real side length)

            # Convert intrinsics to OpenCV format
            cameraMatrix = np.array([
                [self.intrinsics.fx, 0, self.intrinsics.ppx],
                [0, self.intrinsics.fy, self.intrinsics.ppy],
                [0, 0, 1]
            ])
            distCoeffs = np.zeros(5)  # or use real distortion coefficients if known

            # Estimate pose of each marker
            rvecs, tvecs = solveMarkerPositons(
                corners, marker_length, cameraMatrix, distCoeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                pts = corners[i][0]
                rvec = rvecs[i]
                tvec = tvecs[i]
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
                tx, ty, tz = tvec.flatten()
                self.get_logger().info(f"Marker {marker_id}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
                self.get_logger().info(f"ARUCO  {marker_id}: x={tx:.3f}, y={ty:.3f}, z={tz:.3f}")
                marker_positions[marker_id] = (tx,ty, tz, quat[0], quat[1], quat[2], quat[3])
                marker_positions_image[marker_id] = tuple(center)

                # Publish TF transform
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = "camera_color_optical_frame"
                transform.child_frame_id = f"aruco_{marker_id}"
                transform.transform.translation.x = float(tx)
                transform.transform.translation.y = float(ty)
                transform.transform.translation.z = float(tz)
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
            print("Base point before correction:", base_point)

           # Store base translation
            self.base_history.append(base_point)

            # Store rotation from marker 1
            current_q = [
                tf1.transform.rotation.x,
                tf1.transform.rotation.y,
                tf1.transform.rotation.z,
                tf1.transform.rotation.w,
            ]
            self.rot_history.append(current_q)

            left_bottom_edge_point = base_point - (tower_width / 2) * right + (tower_width / 2) * forward
            # left_bottom_edge_point = base_point - (tower_width / 2) * right 
            right_bottom_edge_point = base_point + (tower_width / 2) * right - (tower_width / 2) * forward
            # right_bottom_edge_point = base_point - (tower_width / 2) * forward
            centre_bottom_edge_point = base_point - (tower_width / 2) * right - (tower_width / 2) * forward
            centre_top_edge_point = base_point - (tower_width / 2) * right - (tower_width / 2) * forward + vertical * 0.6
            # bottom_point_camera = self.transform_point_to_camera(bottom_point, from_frame="tower_base")
            (lbx, lby) = self.global_2_pixel([left_bottom_edge_point[0], left_bottom_edge_point[1], left_bottom_edge_point[2]])
            (rbx, rby) = self.global_2_pixel([right_bottom_edge_point[0], right_bottom_edge_point[1], right_bottom_edge_point[2]])
            (cbx, cby) = self.global_2_pixel([centre_bottom_edge_point[0], centre_bottom_edge_point[1], centre_bottom_edge_point[2]])
            (ctx, cty) = self.global_2_pixel([centre_top_edge_point[0], centre_top_edge_point[1], centre_top_edge_point[2]])
            # (ctx, cty) = self.global_2_pixel([base_point[0], base_point[1], base_point[2]])
            # ctx = (lbx + rbx) // 2
            # cty = (lby + rby) // 2

            hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            lower_green = np.array([90,208,129])
            upper_green = np.array([107,255,201])

            # Threshold the image to get only green colors
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Define a kernel (structuring element)
            kernel = np.ones((2,2), np.uint8) # A 5x5 square kernel

            # Apply dilation
            # mask = cv2.dilate(mask, kernel, iterations = 1)

            # Find contours of the green regions
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            area_height = 400  # You can adjust this value as needed
            lowest_point = max(marker_positions_image[1][1], marker_positions_image[2][1])
            left = min(marker_positions_image[1][0], marker_positions_image[2][0])
            right_image = max(marker_positions_image[1][0], marker_positions_image[2][0])
            area_top_left = (left - 70, lowest_point - area_height)
            area_bottom_right = (right_image + 70, lowest_point)

            cv2.rectangle(self.cv_image, area_top_left, area_bottom_right, (255, 0, 0), 2)

            cv2.line(self.cv_image, (lbx, lby), (cbx, cby), (255, 0, 0), 2)
            cv2.line(self.cv_image, (cbx, cby), (rbx, rby), (255, 0, 0), 2)
            cv2.line(self.cv_image, (cbx, cby), (ctx, cty), (0, 0, 255), 2)

            # positions = np.array([
            #     [-tower_width/2,  centre_offset + left_offset,      0],   # left_one
            #     [-tower_width/2,  0             + left_offset,      0],   # left_two
            #     [-tower_width/2, -centre_offset + left_offset,      0],   # left_three
            #     [-centre_offset, -tower_width/2,                    0],   # right_one
            #     [0,              -tower_width/2,                    0],   # right_two
            #     [centre_offset,  -tower_width/2,                    0],   # right_three
            # ])

            # position_names = [
            #     "left_one", "left_two", "left_three",
            #     "right_one", "right_two", "right_three"
            # ]
            # try:
            #     # Transform from frame A -> B
            #     transform = self.tf_buffer.lookup_transform(
            #         'tower_base',             # target frame
            #         'camera_color_optical_frame',      # source frame
            #         rclpy.time.Time()
            #     )
            # except Exception as e:
            #     self.get_logger().warn(f"Transform failed: {e}")
            #     return

            # count = 0
            # for pos in positions:
            #     count += 1
            #     transformNew = TransformStamped()
            #     transformNew.header.stamp = self.get_clock().now().to_msg()
            #     transformNew.header.frame_id = "tower_base"
            #     transformNew.child_frame_id = f"place_{count}"
            #     x, y, z = pos.tolist()
            #     print(f"Place position {count}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            #     transformNew.transform.translation.x = x
            #     transformNew.transform.translation.y = y
            #     transformNew.transform.translation.z = z
            #     transformNew.transform.rotation.x = 0.0
            #     transformNew.transform.rotation.y = 0.0
            #     transformNew.transform.rotation.z = 0.0
            #     transformNew.transform.rotation.w = 1.0
            #     self.tf_broadcaster.sendTransform(transformNew)

            # positions_x_y = np.array([ [p[0], p[1]] for p in positions ])
            points_x = np.empty((0, 1))
            
            points = []

            print(f"cbx: {cbx}, cby: {cby}, ctx: {ctx}, cty: {cty}")
            print(f"lbx: {lbx}, lby: {lby}, rbx: {rbx}, rby: {rby}")

            # Draw bounding boxes around detected green regions
            for cnt in contours:
                ix, iy, iw, ih = cv2.boundingRect(cnt)
                if not (ix >= area_top_left[0] and ix + iw <= area_bottom_right[0] and
                    iy >= area_top_left[1] and iy + ih <= area_bottom_right[1]):
                    continue
                if ih < 10:
                    # print("Width rejected:", iw)
                    continue

                center = (ix + iw // 2, iy + ih // 2)

                left_right_offset = map_value(center[1], cby, cty, cbx, ctx)

                bottom_side_x = lbx if center[0] < left_right_offset else rbx
                bottom_side_y = lby if center[0] < left_right_offset else rby

                base_y = map_value(center[0] - left_right_offset + cbx, cbx, bottom_side_x, cby, bottom_side_y)


                point_x = center[0] - left_right_offset + cbx
                point_y = base_y - center[1]
                print(f"{point_x}, {point_y}")

                points_x = np.append(points_x, [[point_x]])
                points.append(((point_x, point_y), (ix, iy, iw, ih)))

                color = (0, 255, 0) if cby - base_y < 0.0 else (0, 0, 255)

            position_guesses = np.array([
                float(cbx) - math.fabs(float(cbx - lbx)) * 0.95,
                float(cbx) - math.fabs(float(cbx - lbx)) * 0.57,
                float(cbx) - math.fabs(float(cbx - lbx)) * 0.16,
                float(cbx) + math.fabs(float(cbx - rbx)) * 0.16,
                float(cbx) + math.fabs(float(cbx - rbx)) * 0.57,
                float(cbx) + math.fabs(float(cbx - rbx)) * 0.95,
            ])

            print(position_guesses)

            kmeans_positions, labels = kmeans2(points_x, position_guesses, iter=10)

            kmeans_positions_to_labels = [ (i, kmeans_positions[i]) for i in range(len(kmeans_positions)) ]

            print(kmeans_positions)
            print(kmeans_positions_to_labels)

            sorted_x_groups = sorted(kmeans_positions_to_labels, key=lambda item: item[1])
            print(sorted_x_groups)

            unsorted_labels_x_groups = [ (i, sorted_x_groups[i][0], sorted_x_groups[i][1]) for i in range(len(sorted_x_groups)) ]
            print(unsorted_labels_x_groups)

            labels_to_x_groups = sorted(unsorted_labels_x_groups, key=lambda item: item[1])
            print(labels_to_x_groups)


            points_side_z = np.array([ [kmeans_positions[labels[i]] <= cbx, points[i][0][1]] for i in range(len(points)) ])
            print("points_side_z", points_side_z)
            
            # points_side_z: Nx2 -> [is_left_bool, z_value]
            points_side_z = np.array(points_side_z)

            # Extract masks
            left_mask = points_side_z[:, 0] == 1
            right_mask = points_side_z[:, 0] == 0

            # Extract Z values
            left_Z_vals = points_side_z[left_mask][:, 1].reshape(-1, 1)
            right_Z_vals = points_side_z[right_mask][:, 1].reshape(-1, 1)

            # Hierarchical clustering
            left_linkage = linkage(left_Z_vals, method='ward')
            right_linkage = linkage(right_Z_vals, method='ward')

            # Instead of number of clusters, use a distance threshold
            distance_threshold = 20  # meters; tweak depending on your tower
            left_clusters = fcluster(left_linkage, distance_threshold, criterion='distance')
            right_clusters = fcluster(right_linkage, distance_threshold, criterion='distance')

            # Make an array that will hold final vertical cluster index for every original point
            height_cluster_labels = np.zeros(len(points), dtype=int)

            # Assign back to original indices
            height_cluster_labels[left_mask] = left_clusters
            height_cluster_labels[right_mask] = right_clusters

            # --- Compute representative Z per cluster ---
            def compute_cluster_heights(z_values, cluster_labels):
                cluster_heights = {}
                for cluster_id in np.unique(cluster_labels):
                    cluster_heights[cluster_id] = np.mean(z_values[cluster_labels == cluster_id])
                return cluster_heights

            # Left side
            left_cluster_heights = compute_cluster_heights(left_Z_vals.flatten(), left_clusters)
            # Right side
            right_cluster_heights = compute_cluster_heights(right_Z_vals.flatten(), right_clusters)

            # Optional: Combine into single dict
            all_cluster_heights = {}
            for mask, clusters_dict in zip([left_mask, right_mask], [left_cluster_heights, right_cluster_heights]):
                for i, cluster_id in enumerate(clusters_dict):
                    # Map cluster_id to Z value
                    all_cluster_heights[("left" if mask is left_mask else "right", cluster_id)] = clusters_dict[cluster_id]


            print("height cluster labels:", height_cluster_labels)

            sorted_clusters = sorted(all_cluster_heights.items(), key=lambda item: item[1])
            print("height cluster:", sorted_clusters)

            cluster_order_map = { (side, cid): i for i, ((side, cid), _) in enumerate(sorted_clusters) }

            print("Cluster order map:", cluster_order_map)

            tower_occupancy = {} 

            for i in range(len(points)):
                (_px, _py), (ix, iy, iw, ih) = points[i]
                side = "left" if points_side_z[i, 0] == 1 else "right"
                cluster_id = height_cluster_labels[i]

                level = cluster_order_map[(side, cluster_id)]
                pos = labels_to_x_groups[labels[i]][0] % 3
                print(f"Point {i}: side={side}, pos={pos}, poslabel={labels[i]}, level={level}, x={_px:.3f}, y={_py:.3f}, w={iw}, h={ih}")
                level_occupancy = tower_occupancy.get(level)
                if level_occupancy is None:
                    level_occupancy = {
                        "side": side,
                        "occupancy": [False, False, False]  # three positions per side
                    }

                level_occupancy["occupancy"][pos] = True
                tower_occupancy[level] = level_occupancy

                label = height_cluster_labels[i]
                if label == 0:
                    color = (255, 0, 255)  # left_one - magenta
                elif label == 1:
                    color = (0, 255, 255)  # left_two - yellow
                elif label == 2:
                    color = (255, 255, 0)  # left_three - cyan
                elif label == 3:
                    color = (255, 0, 0)  # right_one - blue
                elif label == 4:
                    color = (0, 255, 0)  # right_two - green
                else:
                    color = (0, 0, 255)  # right_three - red

                cv2.rectangle(self.cv_image, (ix, iy), (ix + iw, iy + ih), color, 2)

            block_poses = PoseArray()
            block_poses.header.stamp = transform.header.stamp
            block_poses.header.frame_id = "tower_base"

            marker_array = MarkerArray()
            marker_id = 0  # unique ID for each marker

            delete_all_marker = Marker()
            delete_all_marker.action = Marker.DELETEALL
            delete_all_marker.header.frame_id = "tower_base"
            delete_all_marker.header.stamp = transform.header.stamp
            marker_array.markers.append(delete_all_marker)

            print("Tower Occupancy:")
            for level_key in sorted(tower_occupancy.keys()):
                level_info = tower_occupancy[level_key]
                side = level_info["side"]
                occupancy = level_info["occupancy"]  # list of 3 bools for left/right positions in this level
                level_index = level_key              # use height cluster index (lowest = 1)

                for i, occupied in enumerate(occupancy):
                    if not occupied:
                        continue

                    pose = Pose()
                    # Compute X, Y, Z based on side and horizontal slot
                    x_pos = 0.0
                    y_pos = [block_width, 0.0, -block_width][i]  # left/middle/right positions
                    z_pos = (level_index - 1) * block_height + block_height / 2  # center of block

                    if side == "left":
                        pose.position = Point(x=x_pos, y=y_pos, z=z_pos)
                        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    else:  # right side, rotate 90 deg for example
                        pose.position = Point(x=-y_pos, y=x_pos, z=z_pos)
                        qz = math.sin(math.pi / 4)
                        qw = math.cos(math.pi / 4)
                        pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

                    block_poses.poses.append(pose)

                    # Create Marker
                    marker = Marker()
                    marker.header.frame_id = "tower_base"
                    marker.header.stamp = transform.header.stamp
                    marker.ns = "tower_blocks"
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose = pose
                    marker.scale.x = block_length
                    marker.scale.y = block_width
                    marker.scale.z = block_height
                    marker.color.r = 1.0 if side == "left" else 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0 if side == "right" else 0.0
                    marker.color.a = 1.0

                    marker_array.markers.append(marker)

                print(f" Level {level_key} ({side}): {occupancy}")

            self.blocks_publisher.publish(block_poses)
            self.block_marker_pub.publish(marker_array)

            
        except Exception as e:
            self.get_logger().error(f"Error in routine_callback: {str(e)}")

    def routine_callback(self):
        self.calculate_tower()
        
        try:
            # === AVERAGE TRANSLATION ===
            avg_pos = np.mean(self.base_history, axis=0)
            avg_x, avg_y, avg_z = avg_pos.tolist()

            # === AVERAGE ROTATION ===
            avg_q = self.average_quaternions(self.rot_history)
            avg_qx, avg_qy, avg_qz, avg_qw = avg_q

            base_point_cam = PoseStamped()
            base_point_cam.header.stamp = self.get_clock().now().to_msg()
            base_point_cam.header.frame_id = "camera_color_optical_frame"
            base_point_cam.pose.position.x = float(avg_x)
            base_point_cam.pose.position.y = float(avg_y)
            base_point_cam.pose.position.z = float(avg_z)
            base_point_cam.pose.orientation.x = float(avg_qx)
            base_point_cam.pose.orientation.y = float(avg_qy)
            base_point_cam.pose.orientation.z = float(avg_qz)
            base_point_cam.pose.orientation.w = float(avg_qw)

            try:
                base_point_in_base = self.tf_buffer.transform(
                    base_point_cam,
                    "base_link",
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                return

            self.get_logger().info(
                f"Averaged base: x={avg_x:.3f}, y={avg_y:.3f}, z={avg_z:.3f} | "
                f"rot=({avg_qx:.3f}, {avg_qy:.3f}, {avg_qz:.3f}, {avg_qw:.3f})"
            )

            # Broadcast averaged TF
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "tower_base"

            # Smoothed translation
            transform.transform.translation.x = base_point_in_base.pose.position.x
            transform.transform.translation.y = base_point_in_base.pose.position.y
            transform.transform.translation.z = base_point_in_base.pose.position.z

            # Smoothed rotation
            transform.transform.rotation.x = base_point_in_base.pose.orientation.x
            transform.transform.rotation.y = base_point_in_base.pose.orientation.y
            transform.transform.rotation.z = base_point_in_base.pose.orientation.z
            transform.transform.rotation.w = base_point_in_base.pose.orientation.w

            self.tf_broadcaster.sendTransform(transform)

            cv2.imshow('Image', self.cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {str(e)}")
            
        
            
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
    
    def average_quaternions(self, quaternions):
        """
        Average a list of quaternions using the Markley method.
        Returns a normalized average quaternion.
        """
        M = np.zeros((4, 4))

        for q in quaternions:
            q = np.array(q, dtype=float).reshape(4, 1)
            M += q @ q.T

        # Eigenvector with largest eigenvalue
        eigenvalues, eigenvectors = np.linalg.eigh(M)
        avg_q = eigenvectors[:, np.argmax(eigenvalues)]
        
        # Normalize
        avg_q = avg_q / np.linalg.norm(avg_q)
        return avg_q.tolist()


def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()


if __name__ == '__main__':
    main()