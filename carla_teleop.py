#!/usr/bin/env python3

import os
import math
import subprocess
import sys
import carla
import numpy as np
import pygame
import rospy
from geometry_msgs.msg import Twist

# Suppress ALSA sound card warnings in WSL2
os.environ['SDL_AUDIODRIVER'] = 'dummy'


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


class CarlaRosKeyboardController:

    def __init__(self):
        rospy.set_param('/use_sim_time', False)
        rospy.init_node('carla_keyboard_controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        host_ip = get_windows_host_ip()
        rospy.loginfo(f'Connecting to CARLA at {host_ip}:2000...')
        self.client = carla.Client(host_ip, 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        settings = self.world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            rospy.loginfo("Reset CARLA engine to Asynchronous mode.")

        self.spectator = self.world.get_spectator()

        self.vehicle = self._get_or_spawn_vehicle()
        self.vehicle.set_simulate_physics(True)
        self.vehicle.set_autopilot(False)

        self.control = carla.VehicleControl()
        self.control.hand_brake = False
        self.control.brake = 0.0
        self.control.throttle = 0.0
        self.control.steer = 0.0
        self.control.manual_gear_shift = False
        self.vehicle.apply_control(self.control)

        bp_lib = self.world.get_blueprint_library()
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_bp.set_attribute('fov', '90')
        cam_tf = carla.Transform(
            carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=-15)
        )
        self.camera = self.world.spawn_actor(cam_bp, cam_tf, attach_to=self.vehicle)
        self.surface = None
        self.camera.listen(self._on_camera_data)

        pygame.init()
        pygame.font.init()
        pygame.display.set_caption('CARLA Teleop (Click inside window to drive)')
        self.display = pygame.display.set_mode((800, 600))
        self.font = pygame.font.SysFont('Arial', 18)
        self.clock = pygame.time.Clock()

    def _get_or_spawn_vehicle(self):
        actors = self.world.get_actors().filter('vehicle.*')
        for actor in actors:
            if actor.attributes.get('role_name') in ['hero', 'ego_vehicle']:
                rospy.loginfo(f"Reusing existing ego vehicle ID: {actor.id}")
                return actor

        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find('vehicle.tesla.model3')
        vehicle_bp.set_attribute('role_name', 'hero')
        spawn_points = self.world.get_map().get_spawn_points()

        for sp in spawn_points:
            sp.location.z += 1.0
            vehicle = self.world.try_spawn_actor(vehicle_bp, sp)
            if vehicle is not None:
                rospy.loginfo(f'Successfully spawned vehicle ID: {vehicle.id}')
                return vehicle

        rospy.logerr('Failed to spawn vehicle: All spawn points blocked.')
        sys.exit(1)

    def _on_camera_data(self, image):
        """Processes raw CARLA image data into a correctly oriented Pygame surface."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = array[:, :, ::-1]  # Convert BGR to RGB (no horizontal flip)
        self.surface = pygame.surfarray.make_surface(np.swapaxes(array, 0, 1))

    def update_spectator_camera(self):
        if self.vehicle is not None:
            veh_tf = self.vehicle.get_transform()
            spectator_rot = carla.Rotation(
                pitch=veh_tf.rotation.pitch - 15,
                yaw=veh_tf.rotation.yaw,
                roll=veh_tf.rotation.roll,
            )
            spectator_loc = veh_tf.transform(carla.Location(x=-6.0, z=3.0))
            self.spectator.set_transform(
                carla.Transform(spectator_loc, spectator_rot)
            )

    def process_inputs(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        keys = pygame.key.get_pressed()

        # 1. Handbrake & Direct Throttle Control
        if keys[pygame.K_SPACE]:
            self.control.hand_brake = True
            self.control.throttle = 0.0
            self.control.brake = 1.0
        else:
            self.control.hand_brake = False

            if keys[pygame.K_w] or keys[pygame.K_UP]:
                self.control.reverse = False
                self.control.throttle = 1.0
                self.control.brake = 0.0
            elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
                self.control.reverse = True
                self.control.throttle = 1.0
                self.control.brake = 0.0
            else:
                self.control.throttle = 0.0
                self.control.brake = 0.0

        # 2. Steering Control
        steer_step = 0.03
        max_steer = 0.25

        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            self.control.steer = min(self.control.steer - steer_step, -max_steer)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self.control.steer = max(self.control.steer + steer_step, max_steer)
        else:
            if self.control.steer > steer_step:
                self.control.steer -= steer_step
            elif self.control.steer < -steer_step:
                self.control.steer += steer_step
            else:
                self.control.steer = 0.0

        self.vehicle.apply_control(self.control)
        return True

    def run(self):
        while not rospy.is_shutdown():
            if not self.process_inputs():
                return

            self.update_spectator_camera()

            if self.surface is not None:
                self.display.blit(self.surface, (0, 0))
            else:
                self.display.fill((30, 30, 30))

            v = self.vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

            hud_bg = pygame.Surface((760, 80))
            hud_bg.set_alpha(180)
            hud_bg.fill((0, 0, 0))
            self.display.blit(hud_bg, (20, 20))

            t1 = self.font.render(
                f'Speed: {speed_kmh:.1f} km/h | Gear: {"REV" if self.control.reverse else "FWD"} | [W/UP]: Forward | [S/DOWN]: Reverse',
                True,
                (0, 255, 0),
            )
            t2 = self.font.render(
                f'[SPACE]: Brake | Throttle: {self.control.throttle:.2f} | Steer: {self.control.steer:.2f} | Brake: {self.control.brake:.2f}',
                True,
                (255, 255, 0),
            )
            self.display.blit(t1, (30, 30))
            self.display.blit(t2, (30, 60))

            pygame.display.flip()
            self.clock.tick(60)

    def cleanup(self):
        rospy.loginfo('Cleaning up actors and shutting down...')
        if hasattr(self, 'camera') and self.camera is not None:
            self.camera.destroy()
        pygame.quit()


if __name__ == '__main__':
    controller = CarlaRosKeyboardController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()