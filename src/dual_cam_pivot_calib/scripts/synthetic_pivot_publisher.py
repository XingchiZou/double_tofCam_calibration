#!/usr/bin/env python3
"""Synthetic dual-camera point cloud publisher for calibration demo."""

import math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


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


class SyntheticPivotPublisher:
    def __init__(self):
        self.r1 = np.array([0.30, 0.20, 0.0])
        self.r2 = np.array([-0.25, 0.15, 0.0])
        self.X = make_transform(rot_z(0.15), np.array([0.50, 0.10, 0.05]))
        self.angles = [0.0, 0.35, -0.40, 0.55]
        self.idx = 0

        rng = np.random.default_rng(42)
        self.ref_cam1 = rng.uniform(-1.0, 1.0, size=(800, 3))
        self.ref_cam1[:, 2] = rng.uniform(0.5, 2.0, size=800)
        self.ref_cam2 = apply_transform(self.ref_cam1, np.linalg.inv(self.X))

        self.pub1 = rospy.Publisher("/cam1/points", PointCloud2, queue_size=1)
        self.pub2 = rospy.Publisher("/cam2/points", PointCloud2, queue_size=1)

    def motion(self, lever_arm, theta):
        R = rot_z(theta)
        t = (R - np.eye(3)) @ lever_arm
        return make_transform(R, t)

    def publish_pose(self, theta):
        A = self.motion(self.r1, theta)
        B = np.linalg.inv(self.X) @ A @ self.X

        cur1 = apply_transform(self.ref_cam1, np.linalg.inv(A))
        cur2 = apply_transform(self.ref_cam2, np.linalg.inv(B))

        self.pub1.publish(cloud_msg("cam1", cur1))
        self.pub2.publish(cloud_msg("cam2", cur2))

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            theta = self.angles[self.idx % len(self.angles)]
            self.publish_pose(theta)
            rospy.loginfo("Published synthetic clouds at theta=%.2f rad", theta)
            self.idx += 1
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("synthetic_pivot_publisher")
    SyntheticPivotPublisher().spin()
