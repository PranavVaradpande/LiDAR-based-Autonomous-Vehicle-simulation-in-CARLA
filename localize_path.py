import carla
import pygame
import numpy as np
import open3d as o3d
import os

# ==============================================================================
# -- Global Variables ----------------------------------------------------------
# ==============================================================================
current_live_points = None  
recorded_path = []

# ==============================================================================
# -- Curvature Math ------------------------------------------------------------
# ==============================================================================
def calculate_curvature(path_points):
    """Calculates the 2D curvature of the path using mathematical gradients."""
    path = np.array(path_points)
    if len(path) < 3:
        return np.zeros(len(path))
    
    x = path[:, 0]
    y = path[:, 1]
    
    # First derivatives
    dx = np.gradient(x)
    dy = np.gradient(y)
    
    # Second derivatives
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature formula: |dx*ddy - dy*ddx| / (dx^2 + dy^2)^(3/2)
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = (dx**2 + dy**2)**1.5
    
    # Prevent division by zero if the car is perfectly still
    denominator[denominator == 0] = 1e-8 
    
    curvature = numerator / denominator
    return curvature

# ==============================================================================
# -- Main Application ----------------------------------------------------------
# ==============================================================================
def main():
    global current_live_points, recorded_path

    # 1. Load the Pre-built Global Map
    map_filename = "carla_point_cloud_map1.ply"
    if not os.path.exists(map_filename):
        print(f"ERROR: Could not find {map_filename}. Please run the mapper first.")
        return

    print(f"Loading Global Map: {map_filename}...")
    global_map_pcd = o3d.io.read_point_cloud(map_filename)
    
    # Downsample slightly to ensure the visualizer stays at 60 FPS
    global_map_pcd = global_map_pcd.voxel_down_sample(voxel_size=0.3)
    
    # Convert left-handed CARLA coordinates to right-handed Open3D coordinates
    map_pts = np.asarray(global_map_pcd.points).copy()
    map_pts[:, 1] = -map_pts[:, 1] 
    global_map_pcd.points = o3d.utility.Vector3dVector(map_pts)
    
    # Paint the global map a dark, dim blue so the live data pops out
    global_map_pcd.paint_uniform_color([0.1, 0.2, 0.4])

    # 2. Setup Open3D Geometries for Live Data and Path
    live_pcd = o3d.geometry.PointCloud()
    path_pcd = o3d.geometry.PointCloud()

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Live Localization & Path Recording', width=1280, height=720)
    vis.get_render_option().background_color = [0.05, 0.05, 0.08] 
    vis.get_render_option().point_size = 3.0
    
    # Add geometries to the window
    vis.add_geometry(global_map_pcd)
    vis.add_geometry(live_pcd)
    vis.add_geometry(path_pcd)

    # 3. Initialize Pygame & Controls
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: No joystick detected.")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # 4. Connect to CARLA
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    vehicle = None
    sensor_list = []

    try:
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        # Spawn at the exact same point to ensure we start inside the map
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        spectator = world.get_spectator()
        
        # Setup LiDAR
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '40.0') 
        lidar_bp.set_attribute('rotation_frequency', '20.0')
        lidar_bp.set_attribute('points_per_second', '150000')
        
        lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.4))
        lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        sensor_list.append(lidar_sensor)

        def lidar_callback(lidar_msg):
            global current_live_points
            
            # Extract raw points
            points = np.frombuffer(lidar_msg.raw_data, dtype=np.float32)
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            
            # Transform local points into GLOBAL coordinates using the vehicle's exact pose
            sensor_matrix = np.array(lidar_msg.transform.get_matrix())
            local_points_homo = np.hstack((points[:, :3], np.ones((points.shape[0], 1))))
            global_points = np.dot(sensor_matrix, local_points_homo.T).T[:, :3]
            
            # Convert to right-handed Open3D coordinates
            global_points[:, 1] = -global_points[:, 1]
            current_live_points = global_points.copy()

        lidar_sensor.listen(lambda data: lidar_callback(data))
        
        print("\n--- READY ---")
        print("Drive the car using your joystick.")
        print("The Open3D camera will now follow your car. Forward is UP.")
        print("Your path (Hot Pink) is being recorded.")
        print("Press CTRL+C in the terminal to save your waypoints and curvature.\n")

        control = carla.VehicleControl()
        clock = pygame.time.Clock()
        first_frame = True

        while True:
            # --- UPDATE VISUALIZER ---
            loc = vehicle.get_transform().location

            if current_live_points is not None:
                # Update live LiDAR overlay
                live_pcd.points = o3d.utility.Vector3dVector(current_live_points.astype(np.float64))
                live_pcd.paint_uniform_color([0.1, 1.0, 0.1]) # Neon Green
                vis.update_geometry(live_pcd)

                # Record and update path visualization
                recorded_path.append([loc.x, -loc.y, loc.z])
                
                # Only show every 5th point so the path doesn't get too dense in the viewer
                path_pts_np = np.array(recorded_path[::5], dtype=np.float64)
                path_pcd.points = o3d.utility.Vector3dVector(path_pts_np)
                path_pcd.paint_uniform_color([1.0, 0.0, 1.0]) # Hot Pink
                vis.update_geometry(path_pcd)

                view_control = vis.get_view_control()

                if first_frame:
                    vis.reset_view_point(True)
                    # Set a top-down camera
                    view_control.set_front([0, 0, 1])
                    # FLIPPED 180: Driving forward now visually moves "up"
                    view_control.set_up([0, 1, 0]) 
                    # Zoom in so we can clearly see the car navigating the map
                    view_control.set_zoom(0.3)
                    first_frame = False

                # OPEN3D CAMERA TRACKING: Follow the car continuously
                view_control.set_lookat([loc.x, -loc.y, loc.z])

            vis.poll_events()
            vis.update_renderer()

            # --- PYGAME & CONTROLS ---
            pygame.event.pump() 

            # CARLA Spectator Camera Follow (3rd Person)
            transform = vehicle.get_transform()
            yaw_rad = np.radians(transform.rotation.yaw)
            cam_x = transform.location.x - 6.0 * np.cos(yaw_rad)
            cam_y = transform.location.y - 6.0 * np.sin(yaw_rad)
            cam_z = transform.location.z + 3.0
            spectator.set_transform(carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw, roll=0.0)
            ))
            
            # Joystick Inputs
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
        print("\nStopping simulation and calculating curvature...")
        
    finally:
        for sensor in sensor_list:
            sensor.destroy()
        if vehicle:
            vehicle.destroy()
        vis.destroy_window()
        pygame.quit()

        # Save the path to a Text file with Curvature!
        if len(recorded_path) > 0:
            # We revert the Y-axis so the saved coordinates exactly match CARLA's raw map format
            original_coords = [[p[0], -p[1], p[2]] for p in recorded_path] 
            
            # Calculate curvature for the recorded path
            curvatures = calculate_curvature(original_coords)
            
            # Stack X, Y, Z, and Curvature together side-by-side
            final_data = np.column_stack((original_coords, curvatures))
            
            # Save to a clean text file
            txt_filename = "waypoints_with_curvature.txt"
            np.savetxt(
                txt_filename, 
                final_data, 
                fmt='%.6f', 
                delimiter=", ", 
                header="X, Y, Z, Curvature", 
                comments=""
            )
            print(f"Success! Saved {len(final_data)} points to '{txt_filename}'.")

if __name__ == '__main__':
    main()