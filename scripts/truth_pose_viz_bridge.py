#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class TruthPoseVizBridge:
    def __init__(self):
        self.source_topic = rospy.get_param("~source_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.pose_topic_out = rospy.get_param("~pose_topic_out", "/airsim_node/drone_1/debug/pose_gt_complete")
        self.odom_topic_out = rospy.get_param("~odom_topic_out", "/airsim_node/drone_1/debug/odom_gt_complete")
        self.path_topic_out = rospy.get_param("~path_topic_out", "/airsim_node/drone_1/debug/path_gt_complete")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "pose_gt_complete")
        self.publish_path = rospy.get_param("~publish_path", True)
        self.max_path_length = int(rospy.get_param("~max_path_length", 0))

        self.pose_pub = rospy.Publisher(self.pose_topic_out, PoseStamped, queue_size=20)
        self.odom_pub = rospy.Publisher(self.odom_topic_out, Odometry, queue_size=20)
        self.path_pub = rospy.Publisher(self.path_topic_out, Path, queue_size=10) if self.publish_path else None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        self.sub = rospy.Subscriber(self.source_topic, PoseStamped, self.callback, queue_size=100)

        rospy.loginfo(
            "truth_pose_viz_bridge source=%s pose_out=%s odom_out=%s path_out=%s",
            self.source_topic,
            self.pose_topic_out,
            self.odom_topic_out,
            self.path_topic_out,
        )

    def callback(self, msg):
        pose_out = PoseStamped()
        pose_out.header = msg.header
        if pose_out.header.stamp.is_zero():
            pose_out.header.stamp = rospy.Time.now()
        pose_out.header.frame_id = self.world_frame
        pose_out.pose = msg.pose
        self.pose_pub.publish(pose_out)

        odom_out = Odometry()
        odom_out.header = pose_out.header
        odom_out.child_frame_id = self.body_frame
        odom_out.pose.pose = pose_out.pose
        self.odom_pub.publish(odom_out)

        if self.path_pub is not None:
            self.path_msg.header = pose_out.header
            self.path_msg.poses.append(pose_out)
            if self.max_path_length > 0 and len(self.path_msg.poses) > self.max_path_length:
                self.path_msg.poses = self.path_msg.poses[-self.max_path_length :]
            self.path_pub.publish(self.path_msg)


if __name__ == "__main__":
    rospy.init_node("truth_pose_viz_bridge")
    TruthPoseVizBridge()
    rospy.spin()
