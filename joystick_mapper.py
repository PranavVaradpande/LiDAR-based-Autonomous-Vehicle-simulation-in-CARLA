import carla
import pygame
import numpy as np
import time
import os

# ==============================================================================
# -- PLY File Exporter (Fast Binary Format) ------------------------------------
# ==============================================================================
def save_ply_binary(points, filename):
    print(f"\n[MAPPER] Saving {len(points)} points to {filename}...")
    
    with open(filename, 'wb') as f:
        # Write PLY Header
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
        
        # Write binary point data
        points_float32 = np.asarray(points, dtype=np.float32)
        f.write(points_float32.tobytes())
        
    print(f"[MAPPER] Map successfully saved to {os.path.abspath(filename)}")

# ==============================================================================
# -- Main Application ----------------------------------------------------------
# ==============================================================================
def main():
    # 1. Initialize Pygame and Joystick
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("ERROR: No joystick/gamepad detected. Please plug one in.")
        return
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected Joystick: {joystick.get_name()}")

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    vehicle = None
    sensor_list = []
    global_point_cloud = []  # Stores all transformed map points

    try:
        # 2. Spawn Vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned Vehicle ID: {vehicle.id}")

        # 3. Spawn & Attach IMU
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(x=0, y=0, z=1.5))
        imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
        sensor_list.append(imu_sensor)

        def imu_callback(imu_msg):
            # We are not actively using IMU math here since CARLA provides perfect transforms,
            # but this keeps the sensor active for future LIO algorithm implementation.
            pass
            
        imu_sensor.listen(lambda data: imu_callback(data))

        # 4. Spawn & Attach LiDAR
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50.0')
        lidar_bp.set_attribute('rotation_frequency', '20.0')
        lidar_bp.set_attribute('points_per_second', '250000')
        
        # Place LiDAR on the roof
        lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.4))
        lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        sensor_list.append(lidar_sensor)

        def lidar_callback(lidar_msg):
            # Convert raw LiDAR data to numpy array
            points = np.frombuffer(lidar_msg.raw_data, dtype=np.float32)
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            local_points = points[:, :3] # We only need X, Y, Z

            # Get the EXACT global position and rotation of the sensor at this exact frame
            sensor_matrix = np.array(lidar_msg.transform.get_matrix())

            # Convert local points to homogeneous coordinates (add a column of 1s)
            local_points_homo = np.hstack((local_points, np.ones((local_points.shape[0], 1))))
            
            # Multiply by transformation matrix to get global Map Coordinates
            global_points = np.dot(sensor_matrix, local_points_homo.T).T[:, :3]
            
            # Save to our map buffer
            global_point_cloud.extend(global_points.tolist())

        lidar_sensor.listen(lambda data: lidar_callback(data))
        
        # Get the spectator (the view in the CARLA window)
        spectator = world.get_spectator()
        
        print("Sensors active. Mapping started...")
        print("=> Forward Steer: Axis 0 | Reverse Steer: Axis 2")
        print("=> Forward Gas: Button 7 | Reverse Gas: Button 2")
        print("=> Brake: Button 6")
        print("=> Press CTRL+C in the terminal to STOP and SAVE the map.")

        # 5. Main Control Loop
        control = carla.VehicleControl()
        clock = pygame.time.Clock()

        while True:
            pygame.event.pump() # Update joystick states

            # --- SPECTATOR / CAMERA FOLLOW ---
            # Calculate position behind and above the car
            transform = vehicle.get_transform()
            yaw_rad = np.radians(transform.rotation.yaw)
            
            # Offset: 6 meters behind, 3 meters above
            cam_x = transform.location.x - 6.0 * np.cos(yaw_rad)
            cam_y = transform.location.y - 6.0 * np.sin(yaw_rad)
            cam_z = transform.location.z + 3.0
            
            cam_transform = carla.Transform(
                carla.Location(x=cam_x, y=cam_y, z=cam_z),
                carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw, roll=0.0)
            )
            spectator.set_transform(cam_transform)

            # --- CUSTOM JOYSTICK MAPPING ---
            
            # Read buttons
            forward_btn = joystick.get_button(7)
            reverse_btn = joystick.get_button(2)
            brake_btn = joystick.get_button(0) 
            
            # Determine gear, throttle, and which steering axis to use
            if reverse_btn:
                # If Reverse Button (2) is pressed: Reverse gear, throttle on, steer with Axis 2
                control.reverse = True
                control.throttle = float(reverse_btn)
                steer_axis = joystick.get_axis(2) 
            else:
                # Default to Forward gear: throttle via Button 7, steer with Axis 0
                control.reverse = False
                control.throttle = float(forward_btn)
                steer_axis = joystick.get_axis(0) 

            # Apply limits and deadzones to steering to stop drift
            if abs(steer_axis) < 0.1: steer_axis = 0.0 

            control.steer = steer_axis
            control.brake = float(brake_btn) 

            vehicle.apply_control(control)
            clock.tick(60) # Run Pygame loop at 60 FPS

    except KeyboardInterrupt:
        print("\n\nUser stopped the simulation. Compiling Map...")
        
    finally:
        print("\nCleaning up actors...")
        for sensor in sensor_list:
            sensor.destroy()
        if vehicle:
            vehicle.destroy()
        
        pygame.quit()

        # 6. Save the Map to Disk
        if len(global_point_cloud) > 0:
            downsampled_cloud = global_point_cloud[::3] 
            save_ply_binary(downsampled_cloud, "carla_point_cloud_map.ply")
        else:
            print("No points collected!")

if __name__ == '__main__':
    main()