#!/usr/bin/env python3
"""Synthetic dual-camera point cloud publisher for full calibration demo.

Supports two motion types controlled via the ~/set_motion service:
  - "pivot"    : pure rotation about Z axis (default at startup)
  - "straight" : pure forward translation along Z axis
  - "rest"     : reference pose (identity transform)
"""

import math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import SetBool, SetBoolResponse


def rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply_transform(points, T):
    hom = np.hstack([points, np.ones((points.shape[0], 1))])
    out = (T @ hom.T).T
    return out[:, :3]


def cloud_msg(frame_id, points):
    fields = [
        point_cloud2.PointField("x", 0, point_cloud2.PointField.FLOAT32, 1),
        point_cloud2.PointField("y", 4, point_cloud2.PointField.FLOAT32, 1),
        point_cloud2.PointField("z", 8, point_cloud2.PointField.FLOAT32, 1),
    ]
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return point_cloud2.create_cloud(header, fields, points.astype(np.float32).tolist())


class SyntheticFullPublisher:
    def __init__(self):
        self.r1 = np.array([0.30, 0.20, 0.0])
        self.r2 = np.array([-0.25, 0.15, 0.0])
        self.X = make_transform(rot_z(0.15), np.array([0.50, 0.10, 0.05]))

        self.pivot_angles = [0.35, -0.40, 0.55]
        self.straight_dist = 0.30
        self.pivot_idx = 0

        # mode: "rest", "pivot", "straight"
        self.mode = "rest"

        rng = np.random.default_rng(42)
        self.ref_cam1 = rng.uniform(-1.0, 1.0, size=(800, 3))
        self.ref_cam1[:, 2] = rng.uniform(0.5, 2.0, size=800)
        self.ref_cam2 = apply_transform(self.ref_cam1, np.linalg.inv(self.X))

        self.pub1 = rospy.Publisher("/cam1/points", PointCloud2, queue_size=1)
        self.pub2 = rospy.Publisher("/cam2/points", PointCloud2, queue_size=1)

        # SetBool.data == True  → switch to "pivot" cycling
        # SetBool.data == False → switch to "straight"
        self.srv_mode = rospy.Service("~set_motion", SetBool, self.handle_set_motion)
        # Trigger-like reset to "rest"
        self.srv_rest = rospy.Service("~set_rest", SetBool, self.handle_set_rest)

    def handle_set_motion(self, req):
        if req.data:
            self.mode = "pivot"
            self.pivot_idx = 0
            return SetBoolResponse(success=True, message="Switched to pivot mode")
        else:
            self.mode = "straight"
            return SetBoolResponse(success=True, message="Switched to straight mode")

    def handle_set_rest(self, req):
        self.mode = "rest"
        return SetBoolResponse(success=True, message="Switched to rest mode")

    def pivot_motion(self, lever_arm, theta):
        R = rot_z(theta)
        t = (R - np.eye(3)) @ lever_arm
        return make_transform(R, t)

    def straight_motion_cam1(self, dist):
        """Pure translation along camera-Z (forward), as observed by Cam1."""
        return make_transform(np.eye(3), np.array([0.0, 0.0, dist]))

    def publish_rest(self):
        self.pub1.publish(cloud_msg("cam1", self.ref_cam1))
        self.pub2.publish(cloud_msg("cam2", self.ref_cam2))

    def publish_pivot(self):
        theta = self.pivot_angles[self.pivot_idx % len(self.pivot_angles)]
        A = self.pivot_motion(self.r1, theta)
        B = np.linalg.inv(self.X) @ A @ self.X

        cur1 = apply_transform(self.ref_cam1, np.linalg.inv(A))
        cur2 = apply_transform(self.ref_cam2, np.linalg.inv(B))

        self.pub1.publish(cloud_msg("cam1", cur1))
        self.pub2.publish(cloud_msg("cam2", cur2))
        rospy.loginfo("Published pivot clouds theta=%.2f rad", theta)
        self.pivot_idx += 1

    def publish_straight(self):
        A = self.straight_motion_cam1(self.straight_dist)
        cur1 = apply_transform(self.ref_cam1, np.linalg.inv(A))

        B = np.linalg.inv(self.X) @ A @ self.X
        cur2 = apply_transform(self.ref_cam2, np.linalg.inv(B))

        self.pub1.publish(cloud_msg("cam1", cur1))
        self.pub2.publish(cloud_msg("cam2", cur2))
        rospy.loginfo("Published straight clouds dist=%.2f m", self.straight_dist)

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.mode == "pivot":
                self.publish_pivot()
            elif self.mode == "straight":
                self.publish_straight()
            else:
                self.publish_rest()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("synthetic_full_publisher")
    SyntheticFullPublisher().spin()
