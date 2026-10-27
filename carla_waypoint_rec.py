import carla
import pygame
import numpy as np
import math
import sys

def get_speed(vehicle):
    """Calculates vehicle speed in km/h from its 3D velocity vector."""
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

def main():
    # Initialize Pygame & Font
    pygame.init()
    pygame.font.init()
    width, height = 1280, 720
    display = pygame.display.set_mode((width, height))
    pygame.display.set_caption("CARLA - Ego Vehicle Teleop & Waypoint Logger")
    font = pygame.font.SysFont("Courier", 18, bold=True)

    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Get CARLA native window spectator reference
    spectator = world.get_spectator()

    blueprint_library = world.get_blueprint_library()

    # 1. Spawn Vehicle as Ego Vehicle at exact Spawn Point [0]
    vehicle_bp = blueprint_library.filter('vehicle.micro.microlino')
    if not vehicle_bp:
        vehicle_bp = blueprint_library.filter('vehicle.citroen.c3')
    bp = vehicle_bp[0] if vehicle_bp else blueprint_library.filter('vehicle.*')[0]

    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', 'hero')

    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0] if spawn_points else carla.Transform()
    vehicle = world.spawn_actor(bp, spawn_point)

    # 2. Attach Pygame Teleop RGB Camera
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(width))
    cam_bp.set_attribute('image_size_y', str(height))
    cam_bp.set_attribute('fov', '90')

    cam_transform = carla.Transform(
        carla.Location(x=-4.0, z=2.0),
        carla.Rotation(pitch=-12)
    )
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

    camera_image = [None]
    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]  # BGRA to RGB
        camera_image[0] = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    camera.listen(camera_callback)

    clock = pygame.time.Clock()
    waypoints = []

    # Initialize tracking position
    loc = vehicle.get_location()
    last_pt = np.array([loc.x, loc.y, loc.z])
    waypoints.append(last_pt)

    MAX_SPEED_KMH = 80.0
    STEP_DIST = 0.5  # Meters between waypoints
    running = True

    try:
        while running:
            clock.tick_busy_loop(60)

            # --- Update CARLA Native Spectator Camera (Follow Ego Vehicle) ---
            veh_trans = vehicle.get_transform()
            spec_loc = veh_trans.transform(carla.Location(x=-4.0, z=2.0))
            spec_rot = carla.Rotation(
                pitch=veh_trans.rotation.pitch - 12.0,
                yaw=veh_trans.rotation.yaw,
                roll=veh_trans.rotation.roll
            )
            spectator.set_transform(carla.Transform(spec_loc, spec_rot))

            # --- Event Handling ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_s:
                        np.savetxt("waypoints.txt", np.array(waypoints), fmt="%.4f", header="x,y,z", comments="")
                        print(f"Saved {len(waypoints)} waypoints to 'waypoints.txt'")

            # --- Keyboard Driving Controls ---
            keys = pygame.key.get_pressed()
            throttle, steer, brake = 0.0, 0.0, 0.0

            if keys[pygame.K_UP]:
                throttle = 1.0
            if keys[pygame.K_DOWN]:
                brake = 1.0
            if keys[pygame.K_LEFT]:
                steer = -0.5
            elif keys[pygame.K_RIGHT]:
                steer = 0.5

            current_speed = get_speed(vehicle)

            # Cap maximum speed
            if current_speed >= MAX_SPEED_KMH:
                throttle = 0.0

            control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
            vehicle.apply_control(control)

            # --- Waypoint Interpolation (0.5m exact step) ---
            curr_loc = vehicle.get_location()
            curr_pt = np.array([curr_loc.x, curr_loc.y, curr_loc.z])
            dist = np.linalg.norm(curr_pt - last_pt)

            if dist >= STEP_DIST:
                num_steps = int(dist // STEP_DIST)
                unit_vec = (curr_pt - last_pt) / dist
                for i in range(1, num_steps + 1):
                    interp_pt = last_pt + unit_vec * (i * STEP_DIST)
                    waypoints.append(interp_pt)
                last_pt = waypoints[-1]

            # --- Render Camera View in Pygame Window ---
            if camera_image[0] is not None:
                display.blit(camera_image[0], (0, 0))

            # --- UI Overlay (HUD) ---
            hud = pygame.Surface((340, 160))
            hud.set_alpha(200)
            hud.fill((15, 15, 15))
            display.blit(hud, (15, 15))

            hud_lines = [
                f"Speed: {current_speed:.1f} / {MAX_SPEED_KMH:.0f} km/h",
                f"X: {curr_loc.x:.2f} m",
                f"Y: {curr_loc.y:.2f} m",
                f"Z: {curr_loc.z:.2f} m",
                f"Waypoints Logged: {len(waypoints)}",
                "[S]: Save TXT | [ESC]: Exit"
            ]

            for i, line in enumerate(hud_lines):
                text_surf = font.render(line, True, (0, 255, 150) if i == 0 else (255, 255, 255))
                display.blit(text_surf, (25, 22 + i * 22))

            pygame.display.flip()

    finally:
        # Auto-save waypoints on exit
        np.savetxt("waypoints.txt", np.array(waypoints), fmt="%.4f", header="x,y,z", comments="")
        print(f"Waypoints saved to 'waypoints.txt' ({len(waypoints)} points).")

        camera.destroy()
        vehicle.destroy()
        pygame.quit()

if __name__ == '__main__':
    main()