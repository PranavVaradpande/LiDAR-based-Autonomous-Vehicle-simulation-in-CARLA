import carla
import math
import time
import numpy as np
import open3d as o3d

def load_waypoints(filename):
    """
    Reads the waypoints.txt file. 
    Expects format: X, Y, Z, Curvature
    """
    waypoints = []
    with open(filename, 'r') as f:
        for line in f:
            if not line.strip() or line.startswith('X'):
                continue
            
            data = line.strip().replace(',', ' ').split()
            if len(data) >= 3:
                x, y, z = float(data[0]), float(data[1]), float(data[2])
                waypoints.append(carla.Location(x=x, y=y, z=z))
    return waypoints

def get_target_waypoint(vehicle_location, waypoints, lookahead_dist=5.0):
    """
    Finds the next waypoint to track based on a lookahead distance.
    """
    closest_dist = float('inf')
    closest_index = 0
    
    for i, wp in enumerate(waypoints):
        dist = vehicle_location.distance(wp)
        if dist < closest_dist:
            closest_dist = dist
            closest_index = i

    target_index = closest_index
    for i in range(closest_index, len(waypoints)):
        if vehicle_location.distance(waypoints[i]) >= lookahead_dist:
            target_index = i
            break
            
    return waypoints[target_index], target_index

def calculate_curvature(wp1, wp2, wp3):
    """
    Calculates the discrete Menger curvature from 3 consecutive waypoints.
    """
    x1, y1 = wp1.x, wp1.y
    x2, y2 = wp2.x, wp2.y
    x3, y3 = wp3.x, wp3.y
    
    # Area of the triangle formed by the 3 points
    triangle_area_term = abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))
    
    # Distances between the points
    d12 = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    d23 = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
    d13 = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
    
    denominator = d12 * d23 * d13
    
    if denominator == 0:
        return 0.0 # Straight line if points overlap
        
    curvature = (2.0 * triangle_area_term) / denominator
    return curvature

def get_target_speed(waypoints, target_index, max_speed_kmh=40.0, lat_accel_max=2.5):
    """
    Calculates a safe speed based on the curvature at the target waypoint.
    """
    # Need at least 3 points to calculate curvature
    if target_index < 1 or target_index >= len(waypoints) - 1:
        return max_speed_kmh
        
    wp1 = waypoints[target_index - 1]
    wp2 = waypoints[target_index]
    wp3 = waypoints[target_index + 1]
    
    kappa = calculate_curvature(wp1, wp2, wp3)
    
    if kappa < 1e-5: # Essentially a straight line
        return max_speed_kmh
        
    # v = sqrt(a_max / curvature) in m/s
    safe_speed_ms = math.sqrt(lat_accel_max / kappa)
    safe_speed_kmh = safe_speed_ms * 3.6
    
    # Bound the target speed
    return min(max_speed_kmh, safe_speed_kmh)

def main():
    # =========================================================================
    # 1. CARLA Setup
    # =========================================================================
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    waypoints = load_waypoints('waypoints_with_curvature.txt')
    if not waypoints:
        print("No waypoints loaded. Check the file path and format.")
        return

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('model3')[0]
    
    start_loc = waypoints[0]
    next_loc = waypoints[1] if len(waypoints) > 1 else start_loc
    yaw = math.degrees(math.atan2(next_loc.y - start_loc.y, next_loc.x - start_loc.x))
    
    spawn_transform = carla.Transform(
        carla.Location(start_loc.x, start_loc.y, start_loc.z + 1.0), 
        carla.Rotation(yaw=yaw)
    )
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
    spectator = world.get_spectator()
    print("Vehicle spawned! Starting autonomous tracking...")

    # =========================================================================
    # 2. Open3D Visualization Setup
    # =========================================================================
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='CARLA Path Tracking', width=1024, height=768)

    try:
        pcd_map = o3d.io.read_point_cloud("your_map.pcd") 
        vis.add_geometry(pcd_map)
    except Exception as e:
        print(f"Warning: Could not load point cloud map. Error: {e}")

    wp_points = np.array([[wp.x, wp.y, wp.z] for wp in waypoints])
    wp_pcd = o3d.geometry.PointCloud()
    wp_pcd.points = o3d.utility.Vector3dVector(wp_points)
    wp_pcd.paint_uniform_color([0.0, 1.0, 0.0])
    vis.add_geometry(wp_pcd)

    vehicle_marker = o3d.geometry.TriangleMesh.create_sphere(radius=1.5)
    vehicle_marker.paint_uniform_color([1.0, 0.0, 0.0])
    vis.add_geometry(vehicle_marker)

    # =========================================================================
    # 3. Control Loop Variables
    # =========================================================================
    # PD Controller Gains
    Kp = 0.8  # Proportional gain
    Kd = 0.3  # Derivative gain
    
    prev_alpha = 0.0
    prev_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - prev_time
            if dt == 0:
                dt = 0.001 # Prevent division by zero
                
            # --- CARLA State ---
            vehicle_transform = vehicle.get_transform()
            vehicle_loc = vehicle_transform.location
            vehicle_yaw_rad = math.radians(vehicle_transform.rotation.yaw)
            
            # --- Spectator Follow Camera ---
            cam_offset_x = -8.0 * math.cos(vehicle_yaw_rad)
            cam_offset_y = -8.0 * math.sin(vehicle_yaw_rad)
            camera_transform = carla.Transform(
                carla.Location(
                    x=vehicle_loc.x + cam_offset_x, 
                    y=vehicle_loc.y + cam_offset_y, 
                    z=vehicle_loc.z + 3.0
                ),
                carla.Rotation(pitch=-15.0, yaw=vehicle_transform.rotation.yaw, roll=0.0)
            )
            spectator.set_transform(camera_transform)
            
            # --- Waypoint Lookahead ---
            velocity = vehicle.get_velocity()
            current_speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            lookahead = max(5.0, current_speed_kmh / 3.6 * 1.5)
            target_wp, wp_index = get_target_waypoint(vehicle_loc, waypoints, lookahead_dist=lookahead)
            
            if wp_index >= len(waypoints) - 1 and vehicle_loc.distance(waypoints[-1]) < 2.0:
                print("Reached the final waypoint!")
                break
                
            # --- Curvature Speed Decision ---
            target_speed_kmh = get_target_speed(waypoints, wp_index, max_speed_kmh=40.0, lat_accel_max=2.5)
            
            # --- PD LATERAL CONTROL (Steering) ---
            dx = target_wp.x - vehicle_loc.x
            dy = target_wp.y - vehicle_loc.y
            target_yaw = math.atan2(dy, dx)
            
            alpha = target_yaw - vehicle_yaw_rad
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi # Normalize error between -pi and pi
            
            # Derivative calculation
            alpha_derivative = (alpha - prev_alpha) / dt
            
            # PD Control equation
            steering_signal = (Kp * alpha) + (Kd * alpha_derivative)
            steering_angle = max(-1.0, min(1.0, steering_signal)) # Clamp between -1 and 1
            
            # Update state for next tick
            prev_alpha = alpha
            prev_time = current_time
            
            # --- P LONGITUDINAL CONTROL (Throttle/Brake) ---
            speed_error = target_speed_kmh - current_speed_kmh
            throttle = max(0.0, min(1.0, speed_error * 0.1))
            brake = max(0.0, min(1.0, -speed_error * 0.1)) if speed_error < 0 else 0.0
            
            control = carla.VehicleControl()
            control.steer = steering_angle
            control.throttle = throttle
            control.brake = brake
            vehicle.apply_control(control)
            
            # --- Open3D Update ---
            current_center = vehicle_marker.get_center()
            target_center = np.array([vehicle_loc.x, vehicle_loc.y, vehicle_loc.z])
            vehicle_marker.translate(target_center - current_center)
            
            vis.update_geometry(vehicle_marker)
            vis.poll_events()
            vis.update_renderer()
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    finally:
        print("Destroying vehicle...")
        if vehicle is not None:
            vehicle.destroy()
        
        vis.destroy_window()
        print("Done.")

if __name__ == '__main__':
    main()