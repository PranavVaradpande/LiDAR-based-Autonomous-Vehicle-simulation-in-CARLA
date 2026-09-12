#!/usr/bin/env python3
import rospy
import math
import tf
from nav_msgs.msg import Odometry

class WaypointRecorder:
    def __init__(self, filename="waypoints.txt", dist_threshold=0.5):
        rospy.init_node('waypoint_recorder')
        self.filename = filename
        self.dist_threshold = dist_threshold
        self.last_pos = None
        
        # Clear/initialize output file
        with open(self.filename, 'w') as f:
            f.write("# x, y, z, yaw\n")
            
        rospy.loginfo(f"Recording path waypoints to {self.filename}...")
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_cb)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Extract Euler yaw from quaternion
        q = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        
        if self.last_pos is not None:
            dist = math.hypot(pos.x - self.last_pos[0], pos.y - self.last_pos[1])
            if dist < self.dist_threshold:
                return
                
        self.last_pos = (pos.x, pos.y)
        
        # Write format: x, y, z, yaw
        with open(self.filename, 'a') as f:
            f.write(f"{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}, {yaw:.4f}\n")
        rospy.loginfo(f"Waypoint logged -> X: {pos.x:.2f}, Y: {pos.y:.2f}, Yaw: {yaw:.2f}")

if __name__ == '__main__':
    recorder = WaypointRecorder()
    rospy.spin()