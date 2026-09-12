#!/usr/bin/env python3
import sys
import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class MapAndPathPublisher:
    def __init__(self, pcd_file="carla_map_downsampled1.pcd"):
        rospy.init_node('pcd_and_path_publisher')

        # Publishers
        self.pcd_pub = rospy.Publisher('/map_pointcloud', PointCloud2, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/carla/ego_vehicle/path', Path, queue_size=10)

        # Path message structure
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # Load and publish PCD Map
        self.publish_pcd_map(pcd_file)

        # Subscribe to vehicle odometry to construct real-time path
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_cb)
        rospy.loginfo("PCD Map and Path Publisher running...")

    def publish_pcd_map(self, file_path):
        try:
            rospy.loginfo(f"Loading PCD map file: {file_path}")
            pcd = o3d.io.read_point_cloud(file_path)
            points = np.asarray(pcd.points)

            if points.shape[0] == 0:
                rospy.logerr("PCD file is empty or missing!")
                return

            header = Header()
            header.frame_id = "map"
            header.stamp = rospy.Time.now()

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]

            pc2_msg = pc2.create_cloud(header, fields, points)
            self.pcd_pub.publish(pc2_msg)
            rospy.loginfo(f"Successfully published map ({len(points)} points) to /map_pointcloud")
        except Exception as e:
            rospy.logerr(f"Failed to load or publish PCD map: {e}")

    def odom_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

if __name__ == '__main__':
    pcd_path = rospy.get_param('~pcd_file', 'carla_map_downsampled1.pcd')
    publisher = MapAndPathPublisher(pcd_path)
    rospy.spin()