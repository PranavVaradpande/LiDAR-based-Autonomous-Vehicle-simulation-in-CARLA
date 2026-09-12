#!/usr/bin/env python3
import carla
import rospy
import math
import tf
import sys
import time
import subprocess
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

def get_windows_host_ip():
    """Dynamically resolves the Windows Host IP address from inside WSL2."""
    try:
        return subprocess.check_output("ip route | awk '/default/ { print $3 }'", shell=True).decode().strip()
    except Exception:
        return '127.0.0.1'

class CarlaRosBridge:
    def __init__(self):
        # Force sim_time to False so rospy uses system time
        rospy.set_param('/use_sim_time', False)
        rospy.init_node('carla_ros_bridge_custom')
        
        default_host = get_windows_host_ip()
        host = rospy.get_param('~host', default_host)
        port = rospy.get_param('~port', 2000)
        timeout = rospy.get_param('~timeout', 10.0)
        
        # Connect to CARLA
        self.client = self._connect_to_carla(host, port, timeout)
        self.world = self.client.get_world()
        
        # Force Asynchronous Mode to avoid ticks hanging
        settings = self.world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
            rospy.loginfo("Successfully reset CARLA engine to Asynchronous mode.")
        
        # Fetch or spawn ego vehicle
        self.vehicle = self._get_ego_vehicle()
        
        self.wheelbase = 2.875
        self.max_steer_angle = 0.70  # ~40 degrees max steer
        self.max_speed_ref = 10.0    # Reference speed (m/s) corresponding to 1.0 throttle
        
        self.odom_pub = rospy.Publisher('/carla/ego_vehicle/odometry', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, self.ackermann_cmd_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.twist_cmd_callback)

    def _connect_to_carla(self, host, port, timeout):
        rospy.loginfo(f"Connecting to CARLA simulator at {host}:{port}...")
        attempts = 0
        max_attempts = 5
        while not rospy.is_shutdown() and attempts < max_attempts:
            try:
                client = carla.Client(host, port)
                client.set_timeout(timeout)
                _ = client.get_world()
                rospy.loginfo(f"Successfully connected to CARLA at {host}:{port}")
                return client
            except RuntimeError as e:
                attempts += 1
                rospy.logwarn(f"Connection attempt {attempts}/{max_attempts} failed: {e}")
                if attempts < max_attempts:
                    time.sleep(2)
        
        rospy.logerr(f"Could not connect to CARLA at {host}:{port}.")
        sys.exit(1)

    def _get_ego_vehicle(self):
        actors = self.world.get_actors().filter('vehicle.*')
        for actor in actors:
            if actor.attributes.get('role_name') in ['hero', 'ego_vehicle']:
                return actor
        if len(actors) > 0:
            return actors[0]
        
        bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        bp.set_attribute('role_name', 'hero')
        spawn_point = self.world.get_map().get_spawn_points()[0]
        return self.world.spawn_actor(bp, spawn_point)

    def ackermann_cmd_callback(self, msg):
        self._apply_vehicle_control(target_speed=msg.speed, steering_angle=msg.steering_angle)

    def twist_cmd_callback(self, msg):
        target_speed = msg.linear.x
        if abs(target_speed) > 0.1:
            steering_angle = math.atan2(self.wheelbase * msg.angular.z, abs(target_speed))
        else:
            steering_angle = msg.angular.z * 0.5
            
        self._apply_vehicle_control(target_speed=target_speed, steering_angle=steering_angle)

    def _apply_vehicle_control(self, target_speed, steering_angle):
        control = carla.VehicleControl()
        
        # Steering angle normalization [-1.0, 1.0]
        control.steer = max(-1.0, min(1.0, steering_angle / self.max_steer_angle))
        
        # Direction check
        if target_speed < 0:
            control.reverse = True
            speed = abs(target_speed)
        else:
            control.reverse = False
            speed = target_speed

        # Direct open-loop throttle & brake (No P-controller feedback)
        if speed == 0.0:
            control.throttle = 0.0
            control.brake = 1.0
        else:
            control.throttle = min(1.0, speed / self.max_speed_ref)
            control.brake = 0.0
            
        self.vehicle.apply_control(control)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            transform = self.vehicle.get_transform()
            velocity = self.vehicle.get_velocity()
            
            ros_x = transform.location.x
            ros_y = -transform.location.y
            ros_z = transform.location.z
            
            roll = math.radians(transform.rotation.roll)
            pitch = math.radians(-transform.rotation.pitch)
            yaw = math.radians(-transform.rotation.yaw)
            
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            ros_vx_world = velocity.x
            ros_vy_world = -velocity.y
            
            body_vx = ros_vx_world * cos_yaw + ros_vy_world * sin_yaw
            body_vy = -ros_vx_world * sin_yaw + ros_vy_world * cos_yaw
            
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            
            odom.pose.pose.position.x = ros_x
            odom.pose.pose.position.y = ros_y
            odom.pose.pose.position.z = ros_z
            
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            
            odom.twist.twist.linear.x = body_vx
            odom.twist.twist.linear.y = body_vy
            
            self.odom_pub.publish(odom)
            
            self.tf_broadcaster.sendTransform(
                (ros_x, ros_y, ros_z),
                q,
                rospy.Time.now(),
                "base_link",
                "map"
            )
            rate.sleep()

if __name__ == '__main__':
    bridge = CarlaRosBridge()
    bridge.run()