#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np


class CameraCalibration(Node):
    def __init__(self):
        super().__init__("camera_calibration")

        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.base = "base_link"
        self.ee = "end_eff_contact"
        self.camera = "camera_link"
        self.tower = "block42b"

        self.timer = self.create_timer(0.5, self.compute)

    # ------------------------------------------
    # TF lookup helper
    # ------------------------------------------
    def lookup(self, parent, child):
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed for {parent}->{child}: {e}"
            )
            return None

    # ------------------------------------------
    # Convert TransformStamped to 4x4 matrix
    # ------------------------------------------
    def transform_to_matrix(self, t: TransformStamped):
        trans = t.transform.translation
        rot = t.transform.rotation

        # Quaternion → rotation matrix
        q = np.array([rot.x, rot.y, rot.z, rot.w], dtype=float)
        R = self.quat_to_rot(q)

        # Assemble 4x4 transform
        M = np.eye(4)
        M[0:3, 0:3] = R
        M[0:3, 3] = [trans.x, trans.y, trans.z]
        return M

    # ------------------------------------------
    # Convert 4x4 matrix back to (translation, quaternion)
    # ------------------------------------------
    def matrix_to_transform(self, M):
        t = M[0:3, 3]
        q = self.rot_to_quat(M[0:3, 0:3])
        return t, q

    # ------------------------------------------
    # Quaternion to rotation matrix
    # ------------------------------------------
    def quat_to_rot(self, q):
        x, y, z, w = q
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z

        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
            [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
        ])
        return R

    # ------------------------------------------
    # Rotation matrix → quaternion
    # ------------------------------------------
    def rot_to_quat(self, R):
        # Uses stable algorithm
        K = np.array([
            [R[0,0] - R[1,1] - R[2,2], 0, 0, 0],
            [R[1,0] + R[0,1], R[1,1] - R[0,0] - R[2,2], 0, 0],
            [R[2,0] + R[0,2], R[2,1] + R[1,2], R[2,2] - R[0,0] - R[1,1], 0],
            [R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1], R[0,0] + R[1,1] + R[2,2]]
        ])
        K = K / 3.0

        w, V = np.linalg.eigh(K)
        q = V[:, np.argmax(w)]
        q = np.array([q[0], q[1], q[2], q[3]])
        if q[3] < 0:
            q = -q
        return q

    # ------------------------------------------
    # Main calibration calculation
    # ------------------------------------------
    def compute(self):
        T_base_ee = self.lookup(self.base, self.ee)
        T_cam_tower = self.lookup(self.camera, self.tower)

        if T_base_ee is None or T_cam_tower is None:
            return

        M_base_ee = self.transform_to_matrix(T_base_ee)
        M_cam_tower = self.transform_to_matrix(T_cam_tower)

        # Invert camera→tower to get tower→camera
        M_tower_cam = np.linalg.inv(M_cam_tower)

        # Compute new base→camera
        M_base_camera = M_base_ee @ M_tower_cam

        # Convert back to translation + quaternion
        t, q = self.matrix_to_transform(M_base_camera)

        # Output static transform format
        print("\nCorrected base_link → camera_link transform:\n")
        print("---- paste into a ROS2 static_transform_publisher ----")
        print(
            f"ros2 run tf2_ros static_transform_publisher "
            f"{t[0]} {t[1]} {t[2]} "
            f"{q[0]} {q[1]} {q[2]} {q[3]} "
            f"{self.base} {self.camera}"
        )
        print("-------------------------------------------------------\n")

        # YAML form
        print("YAML format:")
        print(f"translation: [{t[0]}, {t[1]}, {t[2]}]")
        print(f"rotation: [{q[0]}, {q[1]}, {q[2]}, {q[3]}]")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraCalibration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
