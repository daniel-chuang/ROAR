"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)

    F1           : toggle HUD
    ESC          : quit
"""

import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_g
from pygame.locals import K_d
from pygame.locals import K_h
from pygame.locals import K_m
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_w
from pygame.locals import K_l
from pygame.locals import K_i
from pygame.locals import K_z
from pygame.locals import K_x
from typing import Tuple
import logging
import carla
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig


class KeyboardControl(object):
    """Class that handles keyboard input."""

    def __init__(self, world, carla_setting: CarlaConfig):
        self.logger = logging.getLogger(__name__)
        if carla_setting.print_keyboard_hint:
            print(__doc__)
            print()
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        self.use_joystick = False
        try:
            self.joystick = XboxController()
            self.use_joystick = True
            pass
        except Exception as e:
            self.logger.info("No joystick detected. Plz use your keyboard instead")

        self.logger.debug("Keyboard Control initiated")

    def parse_events(self, client, world, clock) -> \
            Tuple[bool, carla.VehicleControl]:
        """
        Parse keyboard press.
        :param client: carla.Client
        :param world: carla.Client
        :param clock: pygame clock
        :return:
            bool - True if should continue, aka no exit key was pressed
            control - carla.VehicleControl
        """
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False, None
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return False, None
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (
                        event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT
                ):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r:
                    if world.recording_enabled:
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")

                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = (
                            not self._control.manual_gear_shift
                        )
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification(
                            "%s Transmission"
                            % (
                                "Manual"
                                if self._control.manual_gear_shift
                                else "Automatic"
                            )
                        )
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker
        if isinstance(self._control, carla.VehicleControl):
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0
            # Set automatic control-related vehicle lights
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else:  # Remove the Brake flag
                current_lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else:  # Remove the Reverse flag
                current_lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Reverse
            if current_lights != self._lights:  # Change the light state only if necessary
                self._lights = current_lights
                world.player.set_light_state(carla.VehicleLightState(self._lights))
        elif isinstance(self._control, carla.WalkerControl):
            self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
        return True, self._control
        # world.player.apply_control(self._control)
        # self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
        # return True, self._control

    def _parse_joystick(self) -> Tuple[float, float]:
        # code to test which axis is your controller using
        vals = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        # print(vals)
        right_trigger_val: float = self.joystick.get_axis(5)
        left_trigger_val: float = self.joystick.get_axis(2)

        right_joystick_horizontal_val = self.joystick.get_axis(3)

        # post processing on raw values
        left_trigger_val = (1 + left_trigger_val) / 2
        right_trigger_val = (1 + right_trigger_val) / 2
        throttle = left_trigger_val + (-1 * right_trigger_val)
        steering = (right_joystick_horizontal_val + self._control.steer * 10) / 11
        return throttle, steering

    def _parse_vehicle_keys(self, keys, milliseconds):

        #print(self.joystick.RightTrigger, self.joystick.LeftJoystickX)
        if self.use_joystick:
            throttle = self.joystick.RightTrigger / 1.5
            steering = self.joystick.LeftJoystickX / 5
            if abs(steering) < 0.02:
                steering = 0
            brake = self.joystick.LeftTrigger
            self._control.manual_gear_shift = True
            self._control.gear = -1 if throttle < 0 else 1
            self._control.throttle = abs(throttle)
            self._control.steer = steering
            self._control.brake = brake
        else:

            if keys[K_UP] or keys[K_w]:
                self._control.throttle = min(self._control.throttle + 0.01, 1)
            else:
                self._control.throttle = 0.0

            if keys[K_DOWN] or keys[K_s]:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._control.brake = 0

            steer_increment = 5e-4 * milliseconds
            if keys[K_LEFT] or keys[K_a]:
                if self._steer_cache > 0:
                    self._steer_cache = 0
                else:
                    self._steer_cache -= steer_increment
            elif keys[K_RIGHT] or keys[K_d]:
                if self._steer_cache < 0:
                    self._steer_cache = 0
                else:
                    self._steer_cache += steer_increment
            else:
                self._steer_cache = 0.0
            self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = 0.01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = 0.01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = (
                world.player_max_speed_fast
                if pygame.key.get_mods() & KMOD_SHIFT
                else world.player_max_speed
            )
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# For Xbox 1 controller
from inputs import get_gamepad
import math
import threading

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        left_x = self.LeftJoystickX
        left_y = self.LeftJoystickY
        right_x = self.RightJoystickX
        right_y = self.RightJoystickY
        a = self.A
        x = self.Y
        y = self.X # b=1, x=2
        b = self.B
        rt = self.RightTrigger
        lt = self.LeftTrigger
        return [left_x, left_y, right_x, right_y, a, x, y, b, rt, lt]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.X = event.state
                elif event.code == 'BTN_WEST':
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state