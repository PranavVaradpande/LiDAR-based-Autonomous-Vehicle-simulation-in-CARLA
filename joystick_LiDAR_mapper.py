import carla
import pygame
import numpy as np
import time
import os
import open3d as o3d

# ==============================================================================
# -- PLY File Exporter (Fast Binary Format) ------------------------------------
# ==============================================================================
def save_ply_binary(points, filename):
    print(f"\n[MAPPER] Saving {len(points)} points to {filename}...")
    with open(filename, 'wb') as f:
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        )
        f.write(header.encode('ascii'))
        points_float32 = np.asarray(points, dtype=np.float32)
        f.write(points_float32.tobytes())
    print(f"[MAPPER] Map successfully saved to {os.path.abspath(filename)}")

# ==============================================================================
# -- Global Variables for Thread-Safe Visualization ----------------------------
# ==============================================================================
current_lidar_frame = None  

# ==============================================================================
# -- Main Application ----------------------------------------------------------
# ==============================================================================
def main():
    global current_lidar_frame

    # 1. Initialize Pygame and Joystick
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("ERROR: No joystick/gamepad detected. Please plug one in.")
        return
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected Joystick: {joystick.get_name()}")

    # 2. Initialize Open3D Visualizer (The "RViz" Alternative)
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name='Real-Time LiDAR (Open3D)',
        width=960,
        height=540,
        left=480,
        top=270
    )
    vis.get_render_option().background_color = [0.05, 0.05, 0.05] 
    vis.get_render_option().point_size = 2.0
    vis.get_render_option().show_coordinate_frame = True

    point_list = o3d.geometry.PointCloud()
    
    # --- FIX 1: Track when to add geometry so we don't add it empty ---
    geometry_added = False

    # 3. Connect to CARLA
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    vehicle = None
    sensor_list = []
    global_point_cloud = []

    try:
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned Vehicle ID: {vehicle.id}")

        # IMU
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(x=0, y=0, z=1.5))
        imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
        sensor_list.append(imu_sensor)

        def imu_callback(imu_msg):
            pass
            
        imu_sensor.listen(lambda data: imu_callback(data))

        # LiDAR
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50.0')
        lidar_bp.set_attribute('rotation_frequency', '20.0')
        lidar_bp.set_attribute('points_per_second', '250000')
        
        lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.4))
        lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        sensor_list.append(lidar_sensor)

        def lidar_callback(lidar_msg):
            global current_lidar_frame

            points = np.frombuffer(lidar_msg.raw_data, dtype=np.float32)
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            
            local_points = points[:, :3].copy() 
            
            # Open3D is Right-Handed, CARLA is Left-Handed
            local_points[:, 1] = -local_points[:, 1] 
            
            # --- COLOR MAPPING BY Z-HEIGHT ---
            z_vals = local_points[:, 2]
            z_norm = (z_vals - np.min(z_vals)) / (np.ptp(z_vals) + 1e-6) 
            
            colors = np.zeros((local_points.shape[0], 3))
            colors[:, 0] = z_norm * 2.0                 # Red channel
            colors[:, 1] = 1.0 - (z_norm * 1.5)         # Green channel
            colors[:, 2] = 0.0                          # Blue channel
            
            colors = np.clip(colors, 0.0, 1.0)

            # Send points and colors to main loop
            current_lidar_frame = (local_points, colors)

            # Map Stitching (Global Points)
            sensor_matrix = np.array(lidar_msg.transform.get_matrix())
            local_points_homo = np.hstack((points[:, :3], np.ones((points.shape[0], 1))))
            global_points = np.dot(sensor_matrix, local_points_homo.T).T[:, :3]
            global_point_cloud.extend(global_points.tolist())

        lidar_sensor.listen(lambda data: lidar_callback(data))
        
        spectator = world.get_spectator()
        
        print("Mapping & Visualization started...")
        print("=> Forward Steer: Axis 0 | Reverse Steer: Axis 2")
        print("=> Forward Gas: Button 7 | Reverse Gas: Button 2")
        print("=> Brake: Button 6")
        print("=> Press CTRL+C in the terminal to STOP and SAVE the map.")

        control = carla.VehicleControl()
        clock = pygame.time.Clock()

        while True:
            # --- OPEN3D VISUALIZATION UPDATE ---
            if current_lidar_frame is not None:
                pts, colors = current_lidar_frame
                
                # --- FIX 2: Cast to float64 for Windows Open3D stability ---
                point_list.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
                point_list.colors = o3d.utility.Vector3dVector(colors.astype(np.float64)) 
                
                # --- FIX 3: Only add geometry AFTER points arrive, then reset camera ---
                if not geometry_added:
                    vis.add_geometry(point_list)
                    geometry_added = True
                    vis.reset_view_point(True)
                else:
                    vis.update_geometry(point_list)
            
            vis.poll_events()
            vis.update_renderer()

            # --- PYGAME & CONTROLS ---
            pygame.event.pump() 

            # Camera follow
            transform = vehicle.get_transform()
            yaw_rad = np.radians(transform.rotation.yaw)
            
            cam_x = transform.location.x - 6.0 * np.cos(yaw_rad)
            cam_y = transform.location.y - 6.0 * np.sin(yaw_rad)
            cam_z = transform.location.z + 3.0
            spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw, roll=0.0)
            ))

            # Joystick inputs
            forward_btn = joystick.get_button(7)
            reverse_btn = joystick.get_button(2)
            brake_btn = joystick.get_button(6) 
            
            if reverse_btn:
                control.reverse = True
                control.throttle = float(reverse_btn)
                steer_axis = joystick.get_axis(2) 
            else:
                control.reverse = False
                control.throttle = float(forward_btn)
                steer_axis = joystick.get_axis(0) 

            if abs(steer_axis) < 0.1: steer_axis = 0.0 

            control.steer = steer_axis
            control.brake = float(brake_btn) 

            vehicle.apply_control(control)
            clock.tick(60) 

    except KeyboardInterrupt:
        print("\n\nUser stopped the simulation. Compiling Map...")
        
    finally:
        print("\nCleaning up...")
        for sensor in sensor_list:
            sensor.destroy()
        if vehicle:
            vehicle.destroy()
        
        vis.destroy_window()
        pygame.quit()

        if len(global_point_cloud) > 0:
            downsampled_cloud = global_point_cloud[::3] 
            save_ply_binary(downsampled_cloud, "carla_point_cloud_map.ply")
        else:
            print("No points collected!")

if __name__ == '__main__':
    main()