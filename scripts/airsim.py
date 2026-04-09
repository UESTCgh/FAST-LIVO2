#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


def imu_cb(msg):
    out = Imu()
    out.header = msg.header
    out.header.frame_id = "body"
    # NED -> FLU: y, z 取反
    out.linear_acceleration.x =  msg.linear_acceleration.x
    out.linear_acceleration.y = -msg.linear_acceleration.y
    out.linear_acceleration.z = -msg.linear_acceleration.z
    out.angular_velocity.x =  msg.angular_velocity.x
    out.angular_velocity.y = -msg.angular_velocity.y
    out.angular_velocity.z = -msg.angular_velocity.z
    out.orientation.w = 1.0
    imu_pub.publish(out)


LIDAR_FIELDS = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
]


def lidar_cb(msg):
    pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    new_pts = [(p[0], -p[1], -p[2]) for p in pts]
    out = pc2.create_cloud(msg.header, LIDAR_FIELDS, new_pts)
    out.header.frame_id = "lidar"
    lidar_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("airsim_ned2flu")

    imu_pub   = rospy.Publisher("/imu_flu",   Imu,         queue_size=200)
    lidar_pub = rospy.Publisher("/lidar_flu", PointCloud2, queue_size=20)

    rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu_cb,
                     queue_size=200, tcp_nodelay=True)
    rospy.Subscriber("/airsim_node/drone_1/lidar",   PointCloud2, lidar_cb,
                     queue_size=20,  tcp_nodelay=True)

    rospy.loginfo("airsim_ned2flu running:")
    rospy.loginfo("  /airsim_node/drone_1/imu/imu -> /imu_flu")
    rospy.loginfo("  /airsim_node/drone_1/lidar   -> /lidar_flu")
    rospy.spin()
