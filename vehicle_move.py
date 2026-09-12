#!/usr/bin/env python3

import math

import subprocess

import sys

import carla

import numpy as np

import pygame

import rospy

from geometry_msgs.msg import Twist





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

        rospy.init_node('carla_keyboard_controller', anonymous=True)



        # 1. ROS Publisher

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)



        # 2. Key State Flags

        self.pressing_up = False

        self.pressing_down = False

        self.pressing_left = False

        self.pressing_right = False

        self.pressing_space = False



        # 3. Connect to CARLA Server

        host_ip = get_windows_host_ip()

        rospy.loginfo(f'Connecting to CARLA at {host_ip}:2000...')

        self.client = carla.Client(host_ip, 2000)

        self.client.set_timeout(10.0)

        self.world = self.client.get_world()



        # Force Async mode so world physics updates continuously

        settings = self.world.get_settings()

        settings.synchronous_mode = False

        settings.fixed_delta_seconds = None

        self.world.apply_settings(settings)



        self.spectator = self.world.get_spectator()



        # 4. Spawn Ego Vehicle

        bp_lib = self.world.get_blueprint_library()

        vehicle_bp = bp_lib.find('vehicle.tesla.model3')

        spawn_points = self.world.get_map().get_spawn_points()



        self.vehicle = None

        for sp in spawn_points:

            sp.location.z += 1.5  # Elevate spawn height to unfreeze PhysX wheels

            self.vehicle = self.world.try_spawn_actor(vehicle_bp, sp)

            if self.vehicle is not None:

                self.vehicle.set_simulate_physics(True)

                self.vehicle.set_autopilot(False)

                rospy.loginfo(f'Successfully spawned vehicle ID: {self.vehicle.id}')

                break



        if self.vehicle is None:

            rospy.logerr('Failed to spawn vehicle: All spawn points blocked.')

            sys.exit(1)



        # 5. Attach Following RGB Camera Sensor

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



        # 6. Pygame Window & Clock Setup

        pygame.init()

        pygame.font.init()

        pygame.display.set_caption('CARLA Third-Person Teleop (Click inside to drive)')

        self.display = pygame.display.set_mode((800, 600))

        self.font = pygame.font.SysFont('Arial', 18)

        self.clock = pygame.time.Clock()

        self.control = carla.VehicleControl()



    def _on_camera_data(self, image):

        """Converts raw CARLA camera frame into Pygame surface."""

        array = np.frombuffer(image.raw_data, dtype=np.dtype('uint8'))

        array = np.reshape(array, (image.height, image.width, 4))[:, :, :3]

        array = array[:, :, ::-1]  # BGR to RGB

        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))



    def update_spectator_camera(self):

        """Updates CARLA desktop view to follow behind the vehicle."""

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



    def handle_events(self):

        """Captures discrete KEYDOWN/KEYUP events."""

        for event in pygame.event.get():

            if event.type == pygame.QUIT:

                return False



            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_UP:

                    self.pressing_up = True

                elif event.key == pygame.K_DOWN:

                    self.pressing_down = True

                elif event.key == pygame.K_LEFT:

                    self.pressing_left = True

                elif event.key == pygame.K_RIGHT:

                    self.pressing_right = True

                elif event.key == pygame.K_SPACE:

                    self.pressing_space = True



            elif event.type == pygame.KEYUP:

                if event.key == pygame.K_UP:

                    self.pressing_up = False

                elif event.key == pygame.K_DOWN:

                    self.pressing_down = False

                elif event.key == pygame.K_LEFT:

                    self.pressing_left = False

                elif event.key == pygame.K_RIGHT:

                    self.pressing_right = False

                elif event.key == pygame.K_SPACE:

                    self.pressing_space = False



        return True



    def apply_vehicle_control(self):

        """Translates input flags into CARLA physics commands."""

        self.control.manual_gear_shift = False

        self.control.hand_brake = self.pressing_space



        v = self.vehicle.get_velocity()

        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)



        # Throttle / Reverse

        if self.pressing_up:

            self.control.throttle = 1.0

            self.control.reverse = False

            self.control.brake = 0.0



            if speed < 0.1:

                fwd = self.vehicle.get_transform().get_forward_vector()

                self.vehicle.set_target_velocity(

                    carla.Vector3D(x=fwd.x * 2.0, y=fwd.y * 2.0, z=0.0)

                )



        elif self.pressing_down:

            self.control.throttle = 1.0

            self.control.reverse = True

            self.control.brake = 0.0



            if speed < 0.1:

                fwd = self.vehicle.get_transform().get_forward_vector()

                self.vehicle.set_target_velocity(

                    carla.Vector3D(x=-fwd.x * 2.0, y=-fwd.y * 2.0, z=0.0)

                )



        else:

            self.control.throttle = 0.0

            self.control.brake = 0.0



        # Steering

        if self.pressing_left:

            self.control.steer = -0.7

        elif self.pressing_right:

            self.control.steer = 0.7

        else:

            self.control.steer = 0.0



        if self.vehicle is not None:

            self.vehicle.apply_control(self.control)



        # Publish telemetry to ROS /cmd_vel

        twist = Twist()

        direction = -1.0 if self.control.reverse else 1.0

        twist.linear.x = float(self.control.throttle * direction)

        twist.angular.z = float(-self.control.steer)

        self.cmd_vel_pub.publish(twist)



    def run(self):

        while not rospy.is_shutdown():

            if not self.handle_events():

                return



            self.apply_vehicle_control()

            self.update_spectator_camera()



            # Render Camera View

            if self.surface is not None:

                self.display.blit(self.surface, (0, 0))

            else:

                self.display.fill((30, 30, 30))



            # Overlay Driving HUD

            v = self.vehicle.get_velocity()

            speed_kmh = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

           

            hud_bg = pygame.Surface((760, 80))

            hud_bg.set_alpha(180)

            hud_bg.fill((0, 0, 0))

            self.display.blit(hud_bg, (20, 20))



            t1 = self.font.render(

                f'Speed: {speed_kmh:.1f} km/h | UP: Forward | DOWN: Reverse | LEFT/RIGHT: Steer',

                True,

                (0, 255, 0),

            )

            t2 = self.font.render(

                f'SPACE: Handbrake | Inputs -> UP:{self.pressing_up} DOWN:{self.pressing_down} LEFT:{self.pressing_left} RIGHT:{self.pressing_right}',

                True,

                (255, 255, 0),

            )

            self.display.blit(t1, (30, 30))

            self.display.blit(t2, (30, 60))



            pygame.display.flip()

            self.clock.tick(30)



    def cleanup(self):

        rospy.loginfo('Cleaning up actors and shutting down...')

        if hasattr(self, 'camera') and self.camera is not None:

            self.camera.destroy()

        if hasattr(self, 'vehicle') and self.vehicle is not None:

            self.vehicle.destroy()

        pygame.quit()





if __name__ == '__main__':

    controller = CarlaRosKeyboardController()

    try:

        controller.run()

    except rospy.ROSInterruptException:

        pass

    finally:

        controller.cleanup()