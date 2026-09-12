#!/usr/bin/env python3
import carla
import numpy as np
import open3d as o3d
import signal
import subprocess
import sys


def get_windows_host_ip():
    """Dynamically resolves the Windows Host IP address from inside WSL2."""
    try:
        return (
            subprocess.check_output(
                "ip route | awk '/default/ { print $3 }'", shell=True
            )
            .decode()
            .strip()
        )
    except Exception:
        return '127.0.0.1'


class PointCloudMapper:

    def __init__(self):
        host_ip = get_windows_host_ip()
        print(f"Connecting to CARLA at {host_ip}:2000...")
        self.client = carla.Client(host_ip, 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        # Attach to existing vehicle spawned by ROS bridge or spawn a new one
        self.vehicle = self._get_ego_vehicle()

        # Attach LiDAR Sensor
        lidar_bp = self.bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '20')

        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.0))
        self.lidar = self.world.spawn_actor(
            lidar_bp, lidar_transform, attach_to=self.vehicle
        )

        self.pcd_aggregated = o3d.geometry.PointCloud()
        self.lidar.listen(lambda data: self.lidar_callback(data))
        print(
            "Mapping in progress... Drive vehicle around using teleop. Press Ctrl+C to save map."
        )

    def _get_ego_vehicle(self):
        actors = self.world.get_actors().filter('vehicle.*')
        for actor in actors:
            if actor.attributes.get('role_name') in ['hero', 'ego_vehicle']:
                return actor
        if len(actors) > 0:
            return actors[0]

        spawn_point = self.world.get_map().get_spawn_points()[0]
        vehicle_bp = self.bp_lib.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'hero')
        return self.world.spawn_actor(vehicle_bp, spawn_point)

    def lidar_callback(self, point_cloud_data):
        # Convert raw LiDAR buffer to N x 3 array
        data = np.frombuffer(point_cloud_data.raw_data, dtype=np.float32)
        points = np.reshape(data, (-1, 4))[:, :3]

        # Transform local LiDAR points to CARLA global world frame
        trans = self.lidar.get_transform().get_matrix()
        homo_points = np.hstack((points, np.ones((points.shape[0], 1))))
        carla_world_points = (trans @ homo_points.T).T[:, :3]

        # Convert CARLA left-hand coordinate frame to ROS right-hand coordinate frame
        ros_world_points = carla_world_points.copy()
        ros_world_points[:, 1] = -carla_world_points[:, 1]  # Invert Y axis

        # Downsample and aggregate frame
        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(ros_world_points)
        downsampled_frame = frame_pcd.voxel_down_sample(voxel_size=0.2)
        self.pcd_aggregated += downsampled_frame

    def save_and_exit(self):
        print("\nSaving point cloud map to carla_map.pcd...")
        self.pcd_aggregated = self.pcd_aggregated.voxel_down_sample(
            voxel_size=0.1
        )
        o3d.io.write_point_cloud("carla_map.pcd", self.pcd_aggregated)

        # Cleanup ONLY the LiDAR actor so vehicle remains active for navigation
        self.lidar.destroy()
        print(
            "Map saved successfully in ROS coordinate frame. Cleaned up LiDAR sensor actor."
        )
        sys.exit(0)


if __name__ == '__main__':
    mapper = PointCloudMapper()
    signal.signal(signal.SIGINT, lambda sig, frame: mapper.save_and_exit())
    signal.pause()