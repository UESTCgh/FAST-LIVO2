#!/usr/bin/env python3

import csv
import math
import os

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Float64MultiArray
from tf.transformations import euler_from_quaternion, inverse_matrix, quaternion_from_matrix, quaternion_matrix, translation_from_matrix


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


class TrajErrorMonitor:
    def __init__(self):
        self.source_topic = rospy.get_param("~source_topic", "/aft_mapped_to_init")
        self.reference_topic = rospy.get_param("~reference_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.reference_timeout = max(0.01, float(rospy.get_param("~reference_timeout", 0.2)))
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.publish_path = rospy.get_param("~publish_path", True)
        self.max_path_length = max(10, int(rospy.get_param("~max_path_length", 4000)))
        self.csv_path = rospy.get_param("~csv_path", "")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.tf_child_frame = rospy.get_param("~tf_child_frame", "")

        self.aligned_path_topic = rospy.get_param("~aligned_path_topic", "/fast_livo/eval/aligned_path")
        self.reference_path_topic = rospy.get_param("~reference_path_topic", "/fast_livo/eval/reference_path")
        self.position_error_topic = rospy.get_param("~position_error_topic", "/fast_livo/eval/position_error")
        self.yaw_error_topic = rospy.get_param("~yaw_error_topic", "/fast_livo/eval/yaw_error_deg")
        self.rmse_position_topic = rospy.get_param("~rmse_position_topic", "/fast_livo/eval/rmse_position")
        self.summary_topic = rospy.get_param("~summary_topic", "/fast_livo/eval/summary")

        self.reference_msg = None
        self.align_matrix = None
        self.source_frame = ""
        self.sample_count = 0
        self.pos_sq_sum = 0.0
        self.yaw_sq_sum = 0.0
        self.max_pos_err = 0.0
        self.max_yaw_err = 0.0

        self.aligned_path = Path()
        self.aligned_path.header.frame_id = self.world_frame
        self.reference_path = Path()
        self.reference_path.header.frame_id = self.world_frame

        self.aligned_path_pub = rospy.Publisher(self.aligned_path_topic, Path, queue_size=10) if self.publish_path else None
        self.reference_path_pub = rospy.Publisher(self.reference_path_topic, Path, queue_size=10) if self.publish_path else None
        self.position_error_pub = rospy.Publisher(self.position_error_topic, Float64, queue_size=10)
        self.yaw_error_pub = rospy.Publisher(self.yaw_error_topic, Float64, queue_size=10)
        self.rmse_position_pub = rospy.Publisher(self.rmse_position_topic, Float64, queue_size=10)
        self.summary_pub = rospy.Publisher(self.summary_topic, Float64MultiArray, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster() if self.publish_tf else None

        self.csv_file = None
        self.csv_writer = None
        self.init_csv()

        self.ref_sub = rospy.Subscriber(self.reference_topic, PoseStamped, self.reference_cb, queue_size=50)
        self.src_sub = rospy.Subscriber(self.source_topic, Odometry, self.source_cb, queue_size=50)

        rospy.loginfo(
            "traj_error_monitor source=%s reference=%s csv=%s",
            self.source_topic,
            self.reference_topic,
            self.csv_path if self.csv_path else "<disabled>",
        )

    def init_csv(self):
        if not self.csv_path:
            return
        csv_dir = os.path.dirname(self.csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "stamp",
            "src_x", "src_y", "src_z",
            "aligned_x", "aligned_y", "aligned_z",
            "ref_x", "ref_y", "ref_z",
            "pos_err",
            "yaw_err_deg",
            "rmse_pos",
            "rmse_yaw_deg",
            "max_pos_err",
            "max_yaw_err_deg",
        ])

    def reference_cb(self, msg):
        pose = msg.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        if not tuple_all_finite(values):
            rospy.logwarn_throttle(1.0, "traj_error_monitor reference pose contains non-finite values, skip sample")
            return
        self.reference_msg = msg

    def reference_fresh(self, stamp):
        if self.reference_msg is None:
            return False
        ref_stamp = self.reference_msg.header.stamp if not self.reference_msg.header.stamp.is_zero() else rospy.Time.now()
        return abs((stamp - ref_stamp).to_sec()) <= self.reference_timeout

    def publish_paths(self, stamp, aligned_pose, ref_pose):
        if self.aligned_path_pub is None or self.reference_path_pub is None:
            return

        aligned_msg = PoseStamped()
        aligned_msg.header.stamp = stamp
        aligned_msg.header.frame_id = self.world_frame
        aligned_msg.pose = aligned_pose
        self.aligned_path.header = aligned_msg.header
        self.aligned_path.poses.append(aligned_msg)
        if len(self.aligned_path.poses) > self.max_path_length:
            self.aligned_path.poses = self.aligned_path.poses[-self.max_path_length :]
        self.aligned_path_pub.publish(self.aligned_path)

        ref_msg = PoseStamped()
        ref_msg.header.stamp = stamp
        ref_msg.header.frame_id = self.world_frame
        ref_msg.pose = ref_pose
        self.reference_path.header = ref_msg.header
        self.reference_path.poses.append(ref_msg)
        if len(self.reference_path.poses) > self.max_path_length:
            self.reference_path.poses = self.reference_path.poses[-self.max_path_length :]
        self.reference_path_pub.publish(self.reference_path)

    def publish_alignment_tf(self, stamp):
        if self.tf_broadcaster is None or self.align_matrix is None:
            return
        child_frame = self.tf_child_frame or self.source_frame
        if not child_frame or child_frame == self.world_frame:
            return
        trans = translation_from_matrix(self.align_matrix)
        quat = quaternion_from_matrix(self.align_matrix)
        self.tf_broadcaster.sendTransform(
            (float(trans[0]), float(trans[1]), float(trans[2])),
            (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            stamp,
            child_frame,
            self.world_frame,
        )

    def source_cb(self, msg):
        if self.reference_msg is None:
            return

        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        self.source_frame = msg.header.frame_id or "camera_init"
        if not self.reference_fresh(stamp):
            return

        src_pose = msg.pose.pose
        src_values = (
            src_pose.position.x, src_pose.position.y, src_pose.position.z,
            src_pose.orientation.x, src_pose.orientation.y, src_pose.orientation.z, src_pose.orientation.w,
        )
        if not tuple_all_finite(src_values):
            rospy.logwarn_throttle(1.0, "traj_error_monitor source pose contains non-finite values, skip sample")
            return

        src_mat = pose_to_matrix(src_pose)
        ref_mat = pose_to_matrix(self.reference_msg.pose)
        if self.align_matrix is None:
            self.align_matrix = ref_mat.dot(inverse_matrix(src_mat))
            rospy.loginfo("traj_error_monitor initialized first-frame alignment")

        self.publish_alignment_tf(stamp)

        aligned_mat = self.align_matrix.dot(src_mat)
        aligned_pose = PoseStamped()
        aligned_pose.header.stamp = stamp
        aligned_pose.header.frame_id = self.world_frame
        matrix_to_pose(aligned_mat, aligned_pose.pose)

        dx = self.reference_msg.pose.position.x - aligned_pose.pose.position.x
        dy = self.reference_msg.pose.position.y - aligned_pose.pose.position.y
        dz = self.reference_msg.pose.position.z - aligned_pose.pose.position.z
        pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)

        yaw_ref = quaternion_to_yaw(self.reference_msg.pose.orientation)
        yaw_est = quaternion_to_yaw(aligned_pose.pose.orientation)
        yaw_err_deg = abs(normalize_angle(yaw_ref - yaw_est)) * 180.0 / math.pi

        self.sample_count += 1
        self.pos_sq_sum += pos_err * pos_err
        self.yaw_sq_sum += yaw_err_deg * yaw_err_deg
        self.max_pos_err = max(self.max_pos_err, pos_err)
        self.max_yaw_err = max(self.max_yaw_err, yaw_err_deg)

        rmse_pos = math.sqrt(self.pos_sq_sum / self.sample_count)
        rmse_yaw = math.sqrt(self.yaw_sq_sum / self.sample_count)

        self.position_error_pub.publish(Float64(pos_err))
        self.yaw_error_pub.publish(Float64(yaw_err_deg))
        self.rmse_position_pub.publish(Float64(rmse_pos))

        summary = Float64MultiArray()
        summary.data = [
            float(self.sample_count),
            pos_err,
            yaw_err_deg,
            rmse_pos,
            rmse_yaw,
            self.max_pos_err,
            self.max_yaw_err,
        ]
        self.summary_pub.publish(summary)

        self.publish_paths(stamp, aligned_pose.pose, self.reference_msg.pose)

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                "{:.6f}".format(stamp.to_sec()),
                src_pose.position.x, src_pose.position.y, src_pose.position.z,
                aligned_pose.pose.position.x, aligned_pose.pose.position.y, aligned_pose.pose.position.z,
                self.reference_msg.pose.position.x, self.reference_msg.pose.position.y, self.reference_msg.pose.position.z,
                pos_err,
                yaw_err_deg,
                rmse_pos,
                rmse_yaw,
                self.max_pos_err,
                self.max_yaw_err,
            ])
            self.csv_file.flush()

        rospy.loginfo_throttle(
            1.0,
            "traj_error_monitor samples=%d pos=%.3f m yaw=%.2f deg rmse_pos=%.3f m rmse_yaw=%.2f deg max_pos=%.3f m",
            self.sample_count,
            pos_err,
            yaw_err_deg,
            rmse_pos,
            rmse_yaw,
            self.max_pos_err,
        )


if __name__ == "__main__":
    rospy.init_node("traj_error_monitor")
    monitor = TrajErrorMonitor()
    rospy.on_shutdown(lambda: monitor.csv_file.close() if monitor.csv_file is not None else None)
    rospy.spin()
