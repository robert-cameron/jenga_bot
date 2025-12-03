import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
import math
import copy
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
from tower_interfaces.msg import Tower, TowerRow
import matplotlib.pyplot as plt

from collections import deque

block_length = 0.13  # meters
block_width = 0.04  # meters
block_height = 0.03  # meters

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
        self.tower_publisher = self.create_publisher(Tower, 'vision/tower', 10)

        self.k = 100  # size of smoothing window
        self.base_history = deque(maxlen=self.k)
        self.rot_history = deque(maxlen=self.k)      # rotation smoothing
        self.tower_history = deque(maxlen=self.k)    # tower smoothing

        self.tower_orientation = None
        self.tower_offset = 0.0


        plt.ion()  # turn on interactive mode

        self.fig, ax = plt.subplots(figsize=(6, 10))
        self.scatter = ax.scatter([], [])
        ax.set_xlabel("X position")
        ax.set_ylabel("Layer")
        ax.set_title("Detected Blocks by Layer and X Position")
        ax.grid(True)
        self.ax = ax
        plt.tight_layout()
        plt.show(block=False)

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

            market_count = 0



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
                if marker_id >= 1 and marker_id <= 4:
                    market_count += 1

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

            print("Marker count:", market_count, len(self.rot_history))

            # Compute intersection only if marker 1 and 2 are found
            if market_count < 2:
                print("Less than 2 markers detected not both detected; skipping intersection computation.")
                return
            
            measuredQuaternions = np.empty((0, 4))

            for i in range(1, 5):
                if i in marker_positions:
                    quat = [
                        marker_positions[i][3],
                        marker_positions[i][4],
                        marker_positions[i][5],
                        marker_positions[i][6]
                    ]
                    measuredQuaternions = np.append(measuredQuaternions, [quat], axis=0)

            
            # Rotation matrix in WORLD FRAME
            measuredQuaternion = self.average_quaternions(measuredQuaternions)

            print("Marker Count:", market_count)
            print("Measured Quaternion:", measuredQuaternion)
            print("rot_history length:", len(self.rot_history))
            self.rot_history.append(measuredQuaternion)

            if self.tower_orientation is not None:
                print("Using stored tower orientation for rotation matrix.")
                quat = self.tower_orientation
            R1 = self.quaternion_to_rotation_matrix(quat)[0:3, 0:3]

            # World-aligned marker axes
            right    = R1[:, 0]   # +X axis (red in RViz)
            forward  = R1[:, 1]   # +Y axis (green in RViz)
            vertical = R1[:, 2]   # +Z axis (blue in RViz)



            # Marker positions (already in same world frame!)

            if 1 in marker_positions:
                x1, y1, z1, *_ = marker_positions[1]
                if 2 in marker_positions:
                    x2, y2, z2, *_ = marker_positions[2]
                    print("Using marker 1 and 2 for direction")
                    direction = right
                    tangent = forward
                    self.tower_offset = 0.0
                elif 4 in marker_positions:
                    x2, y2, z2, *_ = marker_positions[4]
                    print("Using marker 1 and 4 for direction")
                    direction = right
                    tangent = -forward
                    self.tower_offset = -np.pi / 2.0
            elif 3 in marker_positions:
                x1, y1, z1, *_ = marker_positions[3]
                if 2 in marker_positions:
                    x2, y2, z2, *_ = marker_positions[2]
                    print("Using marker 3 and 2 for direction")
                    direction = -right
                    tangent = forward
                    self.tower_offset = np.pi / 2.0
                elif 4 in marker_positions:
                    x2, y2, z2, *_ = marker_positions[4]
                    print("Using marker 3 and 4 for direction")
                    direction = -right
                    tangent = -forward
                    self.tower_offset = np.pi

            distance = np.sqrt(((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)/ 2.0)

            base_point = np.array([x1, y1, z1 ]) + direction * distance
            print("Base point before correction:", base_point)

           # Store base translation
            self.base_history.append(base_point)

            left_bottom_edge_point = base_point - (tower_width / 2) * direction + (tower_width / 2) * tangent
            # left_bottom_edge_point = base_point - (tower_width / 2) * right 
            right_bottom_edge_point = base_point + (tower_width / 2) * direction - (tower_width / 2) * tangent
            # right_bottom_edge_point = base_point - (tower_width / 2) * forward
            centre_bottom_edge_point = base_point - (tower_width / 2) * direction - (tower_width / 2) * tangent
            centre_top_edge_point = base_point - (tower_width / 2) * direction - (tower_width / 2) * tangent + vertical * 0.6
            # bottom_point_camera = self.transform_point_to_camera(bottom_point, from_frame="tower_base")
            (lbx, lby) = self.global_2_pixel([left_bottom_edge_point[0], left_bottom_edge_point[1], left_bottom_edge_point[2]])
            (rbx, rby) = self.global_2_pixel([right_bottom_edge_point[0], right_bottom_edge_point[1], right_bottom_edge_point[2]])
            (cbx, cby) = self.global_2_pixel([centre_bottom_edge_point[0], centre_bottom_edge_point[1], centre_bottom_edge_point[2]])
            (ctx, cty) = self.global_2_pixel([centre_top_edge_point[0], centre_top_edge_point[1], centre_top_edge_point[2]])
            ctx = (ctx + cbx) // 2
            # (ctx, cty) = self.global_2_pixel([base_point[0], base_point[1], base_point[2]])
            # ctx = (lbx + rbx) // 2
            # cty = (lby + rby) // 2

            hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            # lower_green = np.array([0,178,110])
            # upper_green = np.array([8,255,198])

            # lower_green = np.array([90,208,129])
            # upper_green = np.array([107,255,201])

            # Threshold the image to get only green colors
            # mask = cv2.inRange(hsv, lower_green, upper_green)

            mask1 = cv2.inRange(hsv, (0, 116, 68), (8, 255, 179))
            mask2 = cv2.inRange(hsv, (168, 74, 98), (180, 167, 169))

            mask = mask1 + mask2

            # Define a kernel (structuring element)
            # kernel = np.ones((2,2), np.uint8) # A 5x5 square kernel

            # Apply dilation
            # mask = cv2.dilate(mask, kernel, iterations = 1)

            # Find contours of the green regions
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            markers = []
            for i in range(1, 5):
                if i in marker_positions_image:
                    markers.append(marker_positions_image[i])


            area_height = 400  # You can adjust this value as needed
            lowest_point = max(markers, key=lambda item: item[1])[1]
            left = min(markers, key=lambda item: item[0])[0]
            right_image = max(markers, key=lambda item: item[0])[0]
            area_top_left = (left - 70, lowest_point - area_height)
            area_bottom_right = (right_image + 70, lowest_point)

            cv2.rectangle(self.cv_image, area_top_left, area_bottom_right, (255, 0, 0), 2)

            cv2.line(self.cv_image, (lbx, lby), (cbx, cby), (255, 0, 0), 2)
            cv2.line(self.cv_image, (cbx, cby), (rbx, rby), (255, 0, 0), 2)
            cv2.line(self.cv_image, (cbx, cby), (ctx, cty), (0, 0, 255), 2)

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

            print("points_x", points_x)

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
            distance_threshold = 30  # pixels; tweak depending on your tower
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

            plot_x = []
            plot_y = []
            plot_colors = []

            for i in range(len(points)):
                (_px, _py), (ix, iy, iw, ih) = points[i]
                side = "left" if points_side_z[i, 0] == 1 else "right"
                cluster_id = height_cluster_labels[i]
                # x position is points_x[i]

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

                label = labels[i]
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

                rgb = (color[2] / 255.0, color[1] / 255.0, color[0] / 255.0)

                # add to lists for plotting
                plot_x.append(points_x[i])
                plot_y.append(level)
                plot_colors.append(rgb)

            for position in kmeans_positions:
                plot_x.append(position)
                plot_y.append(float(-1))  # plot below all levels
                plot_colors.append((0.5, 0.5, 0.5))  # grey

            for position in position_guesses:

                plot_x.append(position)
                plot_y.append(float(-2))  # plot below all levels
                plot_colors.append((0.5, 0.5, 0.5))  # grey

            # Convert to Nx2 numpy array
            points_arr = np.column_stack((plot_x, plot_y))  # shape (N, 2)

            # Update positions
            self.scatter.set_offsets(points_arr)

            # Update colours
            self.scatter.set_color(plot_colors)

            if len(plot_x) > 0:
                self.ax.set_xlim(min(plot_x) - 10, max(plot_x) + 10)
            if len(plot_y) > 0:
                self.ax.set_ylim(max(plot_y) + 1, min(plot_y) - 1)

            # 3. Force a real redraw
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

            # 4. Allow GUI backend time to update
            plt.pause(0.001)

            self.tower_history.append(copy.deepcopy(tower_occupancy))

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

            tower = Tower()

            tower_occupancy = self.compute_mode_tower()

            print("Tower Occupancy:")
            for level_key in sorted(tower_occupancy.keys()):
                level_info = tower_occupancy[level_key]
                side = level_info["side"]
                occupancy = level_info["occupancy"]  # list of 3 bools for left/right positions in this level
                level_index = level_key              # use height cluster index (lowest = 1)
                if side == "left":
                    tower_row = TowerRow(pos1=occupancy[0], pos2=occupancy[1], pos3=occupancy[2])
                else:
                    tower_row = TowerRow(pos1=occupancy[2], pos2=occupancy[1], pos3=occupancy[0])
                if level_key != len(tower.rows):
                    self.get_logger().warn(f"Non-sequential tower level detected: {level_key} (expected {len(tower.rows)})")

                tower.rows.append(tower_row)

                for i, occupied in enumerate(occupancy):
                    if not occupied:
                        continue

                    pose = Pose()
                    # Compute X, Y, Z based on side and horizontal slot
                    x_pos = 0.0
                    y_pos = [-block_width, 0.0, block_width][i]  # left/middle/right positions
                    z_pos = (level_index) * block_height + block_height / 2  # center of block

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
                    marker.header.frame_id = "tower_base_rotated"
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
            self.tower_publisher.publish(tower)

            
        except Exception as e:
            self.get_logger().error(f"Error in routine_callback: {str(e)}")

    def routine_callback(self):
        self.calculate_tower()
        
        try:
            # === AVERAGE TRANSLATION ===
            avg_pos = np.mean(self.base_history, axis=0)
            avg_x, avg_y, avg_z = avg_pos.tolist()

            # === AVERAGE ROTATION === (not used for final orientation, but kept for transform step)
            avg_q = self.average_quaternions(self.rot_history)
            avg_qx, avg_qy, avg_qz, avg_qw = avg_q

            # Build pose for camera frame → base_link transform
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

            # Transform into base_link frame
            try:
                base_point_in_base = self.tf_buffer.transform(
                    base_point_cam,
                    "base_link",
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                return

            # Extract smoothed X,Y
            bx = base_point_in_base.pose.position.x
            by = base_point_in_base.pose.position.y

            # === FIXED Z VALUE ===
            bz = 0.020     # <-- choose your fixed Z height in base_link frame

            # === FORCE ORIENTATION TO BE PARALLEL TO base_link Z AXIS ===
            # i.e., zero roll, zero pitch, yaw only
            # Compute yaw from original quaternion (optional) or set yaw=0
            orig_q = base_point_in_base.pose.orientation
            # Convert quaternion → yaw
            yaw = np.arctan2(
                2.0 * (orig_q.w * orig_q.z + orig_q.x * orig_q.y),
                1.0 - 2.0 * (orig_q.y**2 + orig_q.z**2)
            # ) - np.pi / 2.0  # rotate 90 degrees to align with tower
            ) 
            

            final_yaw = yaw + self.tower_offset + 3.0 * np.pi / 2.0
            # Build yaw-only quaternion (roll=0, pitch=0)
            final_qx = 0.0
            final_qy = 0.0
            final_qz = np.sin(final_yaw / 2.0)
            final_qw = np.cos(final_yaw / 2.0)

            # === BROADCAST TRANSFORM ===
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "tower_base"

            transform.transform.translation.x = bx
            transform.transform.translation.y = by
            transform.transform.translation.z = bz  # <- fixed Z

            transform.transform.rotation.x = final_qx
            transform.transform.rotation.y = final_qy
            transform.transform.rotation.z = final_qz
            transform.transform.rotation.w = final_qw

            self.tf_broadcaster.sendTransform(transform)

            mode_tower = self.compute_mode_tower()

            if mode_tower is not None and mode_tower[0]["side"] == "right":
                blocks_yaw = yaw + self.tower_offset + 3.0 * np.pi / 2.0
            else:
                blocks_yaw = yaw + self.tower_offset + np.pi

            rotated_qx = 0.0
            rotated_qy = 0.0
            rotated_qz = np.sin(blocks_yaw / 2.0)
            rotated_qw = np.cos(blocks_yaw / 2.0)

            transform_rotated = TransformStamped()
            transform_rotated.header.stamp = self.get_clock().now().to_msg()
            transform_rotated.header.frame_id = "base_link"
            transform_rotated.child_frame_id = "tower_base_rotated"

            transform_rotated.transform.translation.x = bx
            transform_rotated.transform.translation.y = by
            transform_rotated.transform.translation.z = bz  # <- fixed Z

            transform_rotated.transform.rotation.x = rotated_qx
            transform_rotated.transform.rotation.y = rotated_qy
            transform_rotated.transform.rotation.z = rotated_qz
            transform_rotated.transform.rotation.w = rotated_qw

            self.tf_broadcaster.sendTransform(transform_rotated)

            # === Create pose in base_link BEFORE broadcasting TF ===
            pose_base = PoseStamped()
            pose_base.header.stamp = self.get_clock().now().to_msg()
            pose_base.header.frame_id = "base_link"

            pose_base.pose.position.x = bx
            pose_base.pose.position.y = by
            pose_base.pose.position.z = base_point_in_base.pose.position.z  # use original Z before fix

            pose_base.pose.orientation.x = rotated_qx
            pose_base.pose.orientation.y = rotated_qy
            pose_base.pose.orientation.z = rotated_qz
            pose_base.pose.orientation.w = rotated_qw

            # === Transform that pose into camera_color_optical_frame ===
            try:
                pose_in_camera = self.tf_buffer.transform(
                    pose_base,
                    "camera_color_optical_frame",
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
            except Exception as e:
                self.get_logger().warn(f"Transform back to camera frame failed: {e}")
                return

            # You can now use pose_in_camera anywhere you want
            self.get_logger().info(
                f"tower_base in camera frame: "
                f"x={pose_in_camera.pose.position.x:.3f}, "
                f"y={pose_in_camera.pose.position.y:.3f}, "
                f"z={pose_in_camera.pose.position.z:.3f}"
            )

            # === FINALLY broadcast tower_base TF in camera_color_optical_frame ===
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = "tower_base_inaccurate_z"

            transform.transform.translation.x = pose_in_camera.pose.position.x
            transform.transform.translation.y = pose_in_camera.pose.position.y
            transform.transform.translation.z = pose_in_camera.pose.position.z

            transform.transform.rotation.x = pose_in_camera.pose.orientation.x
            transform.transform.rotation.y = pose_in_camera.pose.orientation.y
            transform.transform.rotation.z = pose_in_camera.pose.orientation.z
            transform.transform.rotation.w = pose_in_camera.pose.orientation.w

            self.tf_broadcaster.sendTransform(transform)

            # Build yaw-only quaternion (roll=0, pitch=0)
            image_qx = 0.0
            image_qy = 0.0
            image_qz = np.sin(yaw / 2.0)
            image_qw = np.cos(yaw / 2.0)

            pose_base = PoseStamped()
            pose_base.header.stamp = self.get_clock().now().to_msg()
            pose_base.header.frame_id = "base_link"

            pose_base.pose.position.x = bx
            pose_base.pose.position.y = by
            pose_base.pose.position.z = base_point_in_base.pose.position.z  # use original Z before fix

            pose_base.pose.orientation.x = image_qx
            pose_base.pose.orientation.y = image_qy
            pose_base.pose.orientation.z = image_qz
            pose_base.pose.orientation.w = image_qw

            # === Transform that pose into camera_color_optical_frame ===
            try:
                image_pose_in_camera = self.tf_buffer.transform(
                    pose_base,
                    "camera_color_optical_frame",
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
            except Exception as e:
                self.get_logger().warn(f"Transform back to camera frame failed: {e}")
                return

            self.tower_orientation = [
                image_pose_in_camera.pose.orientation.x,
                image_pose_in_camera.pose.orientation.y,
                image_pose_in_camera.pose.orientation.z,
                image_pose_in_camera.pose.orientation.w
            ]

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
    
    def tower_occupancy_to_string(self, tower_occupancy):
        result = ""
        for level in sorted(tower_occupancy.keys()):
            side = "L" if tower_occupancy[level]["side"] == "left" else "R"
            occupancy = tower_occupancy[level]["occupancy"]
            result += f"{side}{occupancy[0]}{occupancy[1]}{occupancy[2]}" + " "
        return result
    
    def compute_mode_tower(self):
        if len(self.tower_history) == 0:
            return None
        
        towers = {}
        tower_map = {}
        for tower in self.tower_history:
            tower_str = self.tower_occupancy_to_string(tower)
            if tower_str in towers:
                towers[tower_str] += 1
            else:
                towers[tower_str] = 1
                tower_map[tower_str] = tower

        # Find the most common tower configuration
        print("Tower configurations and their counts:", towers)
        mode_tower = max(towers.items(), key=lambda item: item[1])[0]
        return tower_map[mode_tower]

def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()


if __name__ == '__main__':
    main()