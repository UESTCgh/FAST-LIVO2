#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import (
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_slerp,
)


def tuple_all_finite(values):
    return all(math.isfinite(value) for value in values)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def pose_to_matrix(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    mat = quaternion_matrix(q)
    mat[0, 3] = pose.position.x
    mat[1, 3] = pose.position.y
    mat[2, 3] = pose.position.z
    return mat


def matrix_to_pose(mat, pose):
    q = quaternion_from_matrix(mat)
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])


class TruthLocalizationBridge:
    def __init__(self):
        self.source_topic = rospy.get_param("~source_topic", "/aft_mapped_to_init")
        self.reference_topic = rospy.get_param("~reference_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.pose_topic_out = rospy.get_param("~pose_topic_out", "/fast_livo/pose_truth_refined")
        self.odom_topic_out = rospy.get_param("~odom_topic_out", "/fast_livo/odom_truth_refined")
        self.path_topic_out = rospy.get_param("~path_topic_out", "/fast_livo/path_truth_refined")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "base_link")
        self.mode = rospy.get_param("~mode", "blend").strip().lower()
        self.position_alpha = max(0.0, min(1.0, float(rospy.get_param("~position_alpha", 0.35))))
        self.orientation_alpha = max(0.0, min(1.0, float(rospy.get_param("~orientation_alpha", 0.2))))
        self.use_reference_position = rospy.get_param("~use_reference_position", True)
        self.use_reference_orientation = rospy.get_param("~use_reference_orientation", True)
        self.reference_timeout = max(0.01, float(rospy.get_param("~reference_timeout", 0.2)))
        self.velocity_alpha = max(0.0, min(1.0, float(rospy.get_param("~velocity_alpha", 0.5))))
        self.publish_path = rospy.get_param("~publish_path", True)
        self.max_path_length = int(rospy.get_param("~max_path_length", 0))

        self.reference_msg = None
        self.align_matrix = None
        self.last_output_stamp = None
        self.last_output_position = None
        self.last_output_yaw = None
        self.last_body_velocity = (0.0, 0.0, 0.0)

        self.pose_pub = rospy.Publisher(self.pose_topic_out, PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic_out, Odometry, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic_out, Path, queue_size=10) if self.publish_path else None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        self.ref_sub = rospy.Subscriber(self.reference_topic, PoseStamped, self.reference_cb, queue_size=20)
        self.src_sub = rospy.Subscriber(self.source_topic, Odometry, self.source_cb, queue_size=20)

        rospy.loginfo(
            "truth_localization_bridge source=%s reference=%s mode=%s pose_out=%s odom_out=%s",
            self.source_topic,
            self.reference_topic,
            self.mode,
            self.pose_topic_out,
            self.odom_topic_out,
        )

    def reference_cb(self, msg):
        values = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if not tuple_all_finite(values):
            rospy.logwarn_throttle(1.0, "truth_localization_bridge reference pose contains non-finite values, skip sample")
            return
        self.reference_msg = msg

    def reference_fresh(self, stamp):
        if self.reference_msg is None:
            return False
        ref_stamp = self.reference_msg.header.stamp if not self.reference_msg.header.stamp.is_zero() else rospy.Time.now()
        return abs((stamp - ref_stamp).to_sec()) <= self.reference_timeout

    def world_to_body_velocity(self, orientation, world_velocity):
        rot = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])[:3, :3]
        return (
            rot[0, 0] * world_velocity[0] + rot[1, 0] * world_velocity[1] + rot[2, 0] * world_velocity[2],
            rot[0, 1] * world_velocity[0] + rot[1, 1] * world_velocity[1] + rot[2, 1] * world_velocity[2],
            rot[0, 2] * world_velocity[0] + rot[1, 2] * world_velocity[1] + rot[2, 2] * world_velocity[2],
        )

    def blend_matrices(self, aligned_mat, reference_pose):
        ref_mat = pose_to_matrix(reference_pose)
        out_mat = aligned_mat.copy()

        if self.use_reference_position:
            out_mat[0, 3] = (1.0 - self.position_alpha) * aligned_mat[0, 3] + self.position_alpha * ref_mat[0, 3]
            out_mat[1, 3] = (1.0 - self.position_alpha) * aligned_mat[1, 3] + self.position_alpha * ref_mat[1, 3]
            out_mat[2, 3] = (1.0 - self.position_alpha) * aligned_mat[2, 3] + self.position_alpha * ref_mat[2, 3]

        if self.use_reference_orientation:
            q_aligned = quaternion_from_matrix(aligned_mat)
            q_ref = [
                reference_pose.orientation.x,
                reference_pose.orientation.y,
                reference_pose.orientation.z,
                reference_pose.orientation.w,
            ]
            q_out = quaternion_slerp(q_aligned, q_ref, self.orientation_alpha)
            rot_mat = quaternion_matrix(q_out)
            out_mat[:3, :3] = rot_mat[:3, :3]

        return out_mat

    def compute_output_matrix(self, source_pose, stamp):
        if self.reference_msg is None:
            return None

        src_mat = pose_to_matrix(source_pose)
        ref_mat = pose_to_matrix(self.reference_msg.pose)

        if self.align_matrix is None:
            self.align_matrix = ref_mat.dot(inverse_matrix(src_mat))
            rospy.loginfo("truth_localization_bridge initialized source-to-reference alignment")

        aligned_mat = self.align_matrix.dot(src_mat)
        if not self.reference_fresh(stamp):
            return aligned_mat

        if self.mode == "truth":
            return ref_mat
        if self.mode == "blend":
            return self.blend_matrices(aligned_mat, self.reference_msg.pose)
        return aligned_mat

    def publish_output(self, stamp, out_pose):
        yaw = quaternion_to_yaw(out_pose.pose.orientation)
        if not math.isfinite(yaw):
            rospy.logwarn_throttle(1.0, "truth_localization_bridge output yaw is non-finite, skip sample")
            return

        body_vel = (0.0, 0.0, 0.0)
        yaw_rate = 0.0
        if self.last_output_stamp is not None:
            dt = max((stamp - self.last_output_stamp).to_sec(), 1e-3)
            pos = out_pose.pose.position
            world_vel = (
                (pos.x - self.last_output_position[0]) / dt,
                (pos.y - self.last_output_position[1]) / dt,
                (pos.z - self.last_output_position[2]) / dt,
            )
            if tuple_all_finite(world_vel):
                raw_body_vel = self.world_to_body_velocity(out_pose.pose.orientation, world_vel)
                body_vel = (
                    self.velocity_alpha * raw_body_vel[0] + (1.0 - self.velocity_alpha) * self.last_body_velocity[0],
                    self.velocity_alpha * raw_body_vel[1] + (1.0 - self.velocity_alpha) * self.last_body_velocity[1],
                    self.velocity_alpha * raw_body_vel[2] + (1.0 - self.velocity_alpha) * self.last_body_velocity[2],
                )
            yaw_rate = normalize_angle(yaw - self.last_output_yaw) / dt

        self.last_output_stamp = stamp
        self.last_output_position = (
            out_pose.pose.position.x,
            out_pose.pose.position.y,
            out_pose.pose.position.z,
        )
        self.last_output_yaw = yaw
        self.last_body_velocity = body_vel

        odom_out = Odometry()
        odom_out.header = out_pose.header
        odom_out.child_frame_id = self.body_frame
        odom_out.pose.pose = out_pose.pose
        odom_out.twist.twist.linear.x = body_vel[0]
        odom_out.twist.twist.linear.y = body_vel[1]
        odom_out.twist.twist.linear.z = body_vel[2]
        odom_out.twist.twist.angular.z = yaw_rate if math.isfinite(yaw_rate) else 0.0

        self.pose_pub.publish(out_pose)
        self.odom_pub.publish(odom_out)

        if self.path_pub is not None:
            self.path_msg.header = out_pose.header
            self.path_msg.poses.append(out_pose)
            if self.max_path_length > 0 and len(self.path_msg.poses) > self.max_path_length:
                self.path_msg.poses = self.path_msg.poses[-self.max_path_length :]
            self.path_pub.publish(self.path_msg)

    def source_cb(self, msg):
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not tuple_all_finite(values):
            rospy.logwarn_throttle(1.0, "truth_localization_bridge source pose contains non-finite values, skip sample")
            return

        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        out_mat = self.compute_output_matrix(pose, stamp)
        if out_mat is None:
            return

        out_pose = PoseStamped()
        out_pose.header.stamp = stamp
        out_pose.header.frame_id = self.world_frame
        matrix_to_pose(out_mat, out_pose.pose)
        self.publish_output(stamp, out_pose)


if __name__ == "__main__":
    rospy.init_node("truth_localization_bridge")
    TruthLocalizationBridge()
    rospy.spin()
