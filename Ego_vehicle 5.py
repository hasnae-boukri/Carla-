#!/usr/bin/env python
# Copyright (c) 2019 Intel Labs
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.
"""Welcome to CARLA manual control with steering wheel Logitech G29.
To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.
To find out the values of your steering wheel use jstest-gtk in Ubuntu."""

from __future__ import print_function

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
import glob
import os
import sys
#from pyquaternion import Quaternion
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
from carla import ColorConverter as cc
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
from collections import deque 
import xlwt 
from misc import distance_vehicle, get_speed
from enum import Enum 
#from SpeedDistance_controller import SpeedDistance_VehiclePIDController
from controller import VehiclePIDController
#from TrafficDetector import * 
from misc import * 
from misc import is_within_distance_ahead
from SpeedDistance_controller import *
from collections import deque
import networkx as nx 
import time 
if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1 ,  K_F3 , K_F4 ,   K_F5 , K_F6 , K_F7, K_F8 , K_F9 , K_F10 
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name
# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
class World(object):
    def __init__(self, carla_world, hud, actor_filter):
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.gnss_sensor = None
        self.lane_invasion_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        
    def get_player_vehicle(self):
        return self.player

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- SpeedDistance_VehiclePIDController-----------------------------------------
# ==============================================================================
class SpeedDistance_VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, player, front_vehicle,
                 args_lateral={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0},
                 args_longitudinal={'K_Pv': 1.0, 'K_Dv': 0.0, 'K_Iv': 0.0,
                                    'K_Pd': 1.0, 'K_Dd': 0.0, 'K_Id': 0.0}):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID controller using the following
        semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        self._player = player
        self._front_vehicle = front_vehicle
        self._world = self._player.get_world()
        self._lon_controller = SpeedDistance_PIDLongitudinalController(
            self._player, self._front_vehicle,**args_longitudinal)
        self._lat_controller = SpeedDistance_PIDLateralController(
            self._player,  **args_lateral)

    def run_step(self, target_speed, target_distance, target_waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """
        print (self._front_vehicle == None)
        if self._front_vehicle is not None:
            if self._front_vehicle.get_acceleration().x <= 1.0:

                throttle = 0.0
                steering = self._lat_controller.run_step(target_waypoint)
                brake = 1.0
        else:
            throttle = self._lon_controller.run_step(target_speed,target_distance)
            steering = self._lat_controller.run_step(target_waypoint)
            brake = 0.0



        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


# ==============================================================================
# -- TrafficDetector------------------------------------------------------------
# ==============================================================================

class TrafficDetector(object):
    
    def __init__(self, vehicle, target_waypoint, sign_distance, vehicle_distance):
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._last_traffic_light = None
        self._lightslist = self._world.get_actors().filter("*traffic_light*")
        self._vehiclelist = self._world.get_actors().filter("*vehicle*")
        self._speedlimit_list = self._world.get_actors().filter("*speed_limit*")
        self._sign_distance = sign_distance
        self._vehicle_distance = vehicle_distance
        self._target_waypoint = target_waypoint
        self._last_us_traffic_light = None
        self._lane_history = []
        self._speedlimit_history = []
        self._speedlimit_history.append("Init")
    
    def record_lane_id(self):
        vehicle = self._vehicle
        current_map = self._map
        speed_limit_list = self._speedlimit_list
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        self._lane_history.append(ego_vehicle_waypoint.lane_id)
    
    def search_front_vehicle(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,distance):        
                # print (target_vehicle.id == vehicle_list[0])
                return (True,target_vehicle)
        return (False,None)
    
    def search_rear_vehicle_left_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y -3.5
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        180.0 + vehicle.get_transform().rotation.yaw,50.0):        
                # print (target_vehicle.id == vehicle_list[0])
                return (True,target_vehicle)
        return (False,None)  

    def search_front_vehicle_left_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y -3.5
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,50.0):        
                # print (target_vehicle.id == vehicle_list[0])
                return (True,target_vehicle)
        return (False,None)    
        
    def search_rear_vehicle_right_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y +3.5
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        # shadow_location = carla.Location(x= ego_vehicle_location.x, y= ego_vehicle_location.y + 3.5, z= ego_vehicle_location.z)
        # shadow_waypoint = self._map.get_waypoint(shadow_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue
            # print (target_vehicle)
            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        180.0 + vehicle.get_transform().rotation.yaw,50.0):        
                return (True,target_vehicle)
        return (False,None)       
    
    def search_front_vehicle_right_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y +3.5
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        # shadow_location = carla.Location(x= ego_vehicle_location.x, y= ego_vehicle_location.y + 3.5, z= ego_vehicle_location.z)
        # shadow_waypoint = self._map.get_waypoint(shadow_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue
            # print (target_vehicle)
            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        180.0 + vehicle.get_transform().rotation.yaw,50.0):        
                return (True,target_vehicle)
        return (False,None)   

    def define_hazard_vehicle(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,15.0):        
                return (True,target_vehicle)
        return (False,None)

    def get_speed_limit(self):
        """

        """
        vehicle = self._vehicle
        current_map = self._map
        speed_limit_list = self._speedlimit_list
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        # while Ture:

        if len(self._speedlimit_history) > 4000:
            last_value = self._speedlimit_history[-1]
            self._speedlimit_history.pop(0)
            self._speedlimit_history.append(last_value)

        for speed_limit in speed_limit_list:
            # print (len(speed_limit_list))
            # speed_limit_history.append("a")

            object_waypoint = current_map.get_waypoint(speed_limit.get_location())
            if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = speed_limit.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,
                                        20.0):
            # if traffic_light.state == carla.libcarla.TrafficLightState.Red:
                speed_limit_value = (str(speed_limit).split('.'))[-1]
                speed_limit_value = speed_limit_value[0] + speed_limit_value[1]
                # print (type(speed_limit_value))
                self._speedlimit_history.append(speed_limit_value)            
                # print (speed_limit_value + str(len(speed_limit_history)))
                return (self._speedlimit_history[-1])
    
        # print (str(len(speed_limit_history)))
        return (self._speedlimit_history[-1])


    def is_light_red(self):
        """
        Method to check if there is a red light affecting us. This version of
        the method is compatible with both European and US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
        """
        world = self._world
        if world.map_name == 'Town01' or world.map_name == 'Town02':
            return self._is_light_red_europe_style()
        else:
            return self._is_light_red_us_style()



    def _is_light_red_europe_style(self):
        """
        This method is specialized to check European style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
        """
        vehicle = self._vehicle
        lights_list = self._lightslist
        current_map = self._map    
        distance = self._sign_distance
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            object_waypoint = current_map.get_waypoint(traffic_light.get_location())
            if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = traffic_light.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,
                                        distance):
                if traffic_light.state == carla.libcarla.TrafficLightState.Red:
                    return (True, traffic_light)

        return (False, None)

    
    def _is_light_red_us_style(self):
        """
        This method is specialized to check US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
        """
        last_us_traffic_light = self._last_us_traffic_light
        vehicle = self._vehicle
        lights_list = self._lightslist
        current_map = self._map 
        target_waypoint = self._target_waypoint
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)

        if ego_vehicle_waypoint.is_intersection:
            # It is too late. Do not block the intersection! Keep going!
            return (False, None)

        if target_waypoint is not None:            
            suitable_waypoint = optimize_target_waypoint(current_map,target_waypoint,vehicle.get_transform().rotation.yaw)
            if suitable_waypoint.is_intersection:
            # if self._local_planner._target_waypoint.is_intersection:
                # potential_lights = []
                min_angle = 180.0
                sel_magnitude = 0.0
                sel_traffic_light = None
                for traffic_light in lights_list:
                    loc = traffic_light.get_location()
                    magnitude, angle = compute_magnitude_angle(loc,
                                                                ego_vehicle_location,
                                                                vehicle.get_transform().rotation.yaw)
                    if magnitude < 80.0 and angle < min(25.0, min_angle):
                    # if magnitude < 280.0 and angle < 40.0:

                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light
                        min_angle = angle

                if sel_traffic_light is not None:
                    print('=== Magnitude = {} | Angle = {} | ID = {} | Status = {}'.format(sel_magnitude, min_angle, sel_traffic_light.id, sel_traffic_light.state ))

                    if last_us_traffic_light is None:
                        last_us_traffic_light = sel_traffic_light

                    if last_us_traffic_light.state == carla.libcarla.TrafficLightState.Red:
                        return (True, last_us_traffic_light)
                else:
                    last_us_traffic_light = None

        return (False, None)

# ==============================================================================
# -- ACC_Controller -----------------------------------------------------------
# ==============================================================================

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4

class ACC_Controller(object):
    
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self,player, opt_dict={}):
        self._player = player
        self._world = self._player.get_world()
        self._map = self._world.get_map()
        self._last_us_traffic_light = None
        self._dt = None
        self._target_speed = None
        self._sampling_radius = None
        self._min_distance = None
        self._current_waypoint = None
        self._target_road_option = None
        self._next_waypoints = None
        self._target_waypoint = None
        self._SpeedDistance_vehicle_controller = None
        self._Speed_vehicle_controller = None
        self._current_plan = None
        self._global_plan = None
        self._trigger_counter = 0
        self._traffic_detector = TrafficDetector(self._player,self._target_waypoint,12.5,100.0)
        self._lane_id = self._map.get_waypoint(self._player.get_location()).lane_id
        
        # queue with tupltarget_vehicle_waypoints of (waypoint, RoadOption)
        self._hop_resolution = 2.0
        self._waypoints_queue = deque(maxlen=600)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        
        # this is for the traffic detector
        self._lightslist = self._world.get_actors().filter("*traffic_light*")
        self._vehiclelist = self._world.get_actors().filter("*vehicle*")
        
        # SETTING controller
        #self.preset_speed = 160.0
        self.preset_speed = 60.0
        self._target_distance = 10.0
        self.initialize_PID_controller(opt_dict)
        
        # Initialize the overtake parameters
        self._overtake_intention = False
        self._overtake_leftchange = False
        self._overtake_rightchange = False
        self._front_overtake_target = None
        self._overtake_done = False
        self._first_move_done = False

        # Automatic lane change parameters
        self._left_lane_change = False
        self._right_lane_change = False

    def refresh_traffic_detector(self):
        self._traffic_detector._target_waypoint = self._target_waypoint

    
    def emergency_brake_situation(self):
        emergency_1 = False
        # emergency_2 = False
        hazard_vehicle_state, hazard_vehicle = self._traffic_detector.define_hazard_vehicle()               
        # light_state, traffic_light = self._traffic_detector.is_light_red()
        if hazard_vehicle_state:
            if get_speed(hazard_vehicle) <= 0.30 * get_speed(self._player) or get_speed(hazard_vehicle) < 5.0:
                # or hazard_vehicle.get_acceleration().x < -0.17:
                emergency_1 = True
        if emergency_1 is True:
            return True
        return False

    def search_overtake_vehicle(self):
        player = self._player
        vehicle_list = self._vehiclelist
        current_map = self._map  
        ego_vehicle_location = player.get_location()        
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == player.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,player.get_transform().rotation.yaw,80.0):  
                if (get_speed(target_vehicle) < self.preset_speed * 0.55) and  (self.emergency_brake_situation() is False):   
                    return (True,target_vehicle)
        return (False,None)

    def set_target_speed(self):
        vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()
        overtake_vehicle_state, overtake_vehicle = self.search_overtake_vehicle()                
        front_left_vehicle_state, front_vehicle_left_lane = self._traffic_detector.search_front_vehicle_left_lane()
        rear_left_vehicle_state, rear_vehicle_left_lane = self._traffic_detector.search_rear_vehicle_left_lane()

        print (self._overtake_intention)
        
        if overtake_vehicle_state is False:
            if vehicle_state is True:
                return get_speed(front_vehicle)
            else:
                return (self.preset_speed)

        else: 
            if front_left_vehicle_state is False and rear_left_vehicle_state is False:
                self._overtake_intention = True 
                self._front_overtake_target = overtake_vehicle
                return (self.preset_speed)
            else:
                return (get_speed(overtake_vehicle))

    def set_target_distance(self):
        front_vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()
        t0=1.5
        Cv=0.05
        Ca=0.3
        th_max=1.6
        th_min=0.2  
        delta_t = 0.0     
        if  front_vehicle_state:

            # ego_x = self._vehicle.get_location().x
            # ego_y = self._vehicle.get_location().y
            
            # front_x = front_vehicle.get_location().x
            # front_y = front_vehicle.get_location().y
            
            # dx = ego_x - front_x
            # dy = ego_y - front_y
            
            # init_distance = math.sqrt(dx * dx + dy * dy)
            RelaSpeed = (get_speed(front_vehicle) - get_speed(self._player)) / 3.6
            # print("RelaSpeed",RelaSpeed)
            ap = front_vehicle.get_acceleration()
            front_acceleration = math.sqrt(ap.x**2+ap.y**2+ap.z**2)
            th = t0 - Cv * RelaSpeed - Ca * front_acceleration
            if th >= th_max:
                delta_t = th_max
            elif th > th_min and th < th_max:
                delta_t = th
            else:
                delta_t = th_min
        
        return  delta_t * (get_speed(self._player) / 3.6) + 15.0
        # print("self._target_distance",self._target_distance)
        # return self._target_distance

    def left_lane_change(self,inject_distance):
        front_left_vehicle_state, front_vehicle_left_lane = self._traffic_detector.search_front_vehicle_left_lane()
        rear_left_vehicle_state, rear_vehicle_left_lane = self._traffic_detector.search_rear_vehicle_left_lane()
        if rear_left_vehicle_state is False and front_left_vehicle_state is False:
            print ('I begin left change')
            self._waypoints_queue.clear()
            new_waypoint = self._map.get_waypoint(carla.Location(self._player.get_location().x + inject_distance, 
                                                                    self._player.get_location().y - 3.5,
                                                                    self._player.get_location().z)) 
            
            self._waypoints_queue.append((new_waypoint, RoadOption.LANEFOLLOW))
        else:
            pass
    
    def right_lane_change(self,inject_distance):
        rear_right_vehicle_state, rear_vehicle_right_lane = self._traffic_detector.search_rear_vehicle_right_lane()
        front_right_vehicle_state, front_vehicle_right_lane = self._traffic_detector.search_front_vehicle_right_lane()
        if rear_right_vehicle_state is False and front_right_vehicle_state is False:
            print ('I begin right change')
            self._waypoints_queue.clear()
            new_waypoint = self._map.get_waypoint(carla.Location(self._player.get_location().x + inject_distance, 
                                                                    self._player.get_location().y + 3.5,
                                                                    self._player.get_location().z)) 
            
            self._waypoints_queue.append((new_waypoint, RoadOption.LANEFOLLOW))
        else:
            pass

    def decide_on_overtake(self):    
        rear_right_vehicle_state, rear_vehicle_right_lane = self._traffic_detector.search_rear_vehicle_right_lane()
        rear_left_vehicle_state, rear_vehicle_left_lane = self._traffic_detector.search_rear_vehicle_left_lane()
        front_right_vehicle_state, front_vehicle_right_lane = self._traffic_detector.search_front_vehicle_right_lane()
        front_left_vehicle_state, front_vehicle_left_lane = self._traffic_detector.search_front_vehicle_left_lane()
    
        if self._overtake_intention is True:    
            # print (self._front_overtake_target is not None)
            if self._front_overtake_target is not None and self._overtake_leftchange is False and self._overtake_rightchange is False and \
                rear_left_vehicle_state is False and front_left_vehicle_state is False and self._overtake_done is False:
                
                self.left_lane_change(80.0)
                self._overtake_leftchange= True

            if self._overtake_leftchange is True and self._first_move_done is False and self._overtake_rightchange is False and self._overtake_done is False:
                if self._map.get_waypoint(self._player.get_location()).lane_id == self._lane_id + 1:
                    self._front_overtake_target = None
                    self._first_move_done = True
                    print ("I reach frist move")
                    self._lane_id = self._map.get_waypoint(self._player.get_location()).lane_id

            if (self._first_move_done is True and rear_right_vehicle_state is False and self._overtake_leftchange is True and \
                self._overtake_rightchange is False and front_right_vehicle_state is False and \
                rear_right_vehicle_state is False and self._overtake_done is False):
                    self.right_lane_change(115.0) 
                    self._overtake_rightchange = True
                    print ("I reach second move")
        
            if self._first_move_done is True and self._overtake_rightchange is True and self._overtake_done is False:
                if self._map.get_waypoint(self._player.get_location()).lane_id == self._lane_id - 1:
                    self._lane_id = self._map.get_waypoint(self._player.get_location()).lane_id               
                    self._overtake_done = True

            
            if self._first_move_done is True and self._overtake_rightchange is True and self._overtake_done is True:
                self._overtake_intention = False
                self._front_overtake_target = None
                self._overtake_leftchange = False
                self._overtake_rightchange = False
                self._overtake_done = False
                self._first_move_done = False
        pass
    
    def emergency_stop(self):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control                  
    
    def initialize_PID_controller(self, opt_dict):
        self._dt = 1.0 / 20.0
        self._target_speed = self.preset_speed  # Km/h
        self._sampling_radius = self._target_speed * 0.5 / 3.6  # 0.5 seconds horizon
        self._min_distance = self._sampling_radius * self.MIN_DISTANCE_PERCENTAGE
        # vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()     
        args_lateral_dict = {
            'K_P': 1.1,
            'K_D': 0.0001,
            'K_I': 1.2,
            'dt': self._dt}
        
        SpeedDistance_args_longitudinal_dict = {
            'K_Pv': 0.4,
            'K_Dv': 0.01,
            'K_Iv':0.2,
            'K_Pd': -0.05,
            'K_Dd': 0.0,
            'K_Id': -0.02,            
            'dt': self._dt}  
        
        Speed_args_longitudinal_dict = {
            'K_P': 1.1,
            'K_D': 0.01,
            'K_I':1.0,
            'dt': self._dt}
        
        if 'dt' in opt_dict:
            self._dt = opt_dict['dt']
        if 'target_speed' in opt_dict:
            self._target_speed = opt_dict['target_speed']
        if 'sampling_radius' in opt_dict:
            self._sampling_radius = self._target_speed * \
                opt_dict['sampling_radius'] / 3.6
        if 'lateral_control_dict' in opt_dict:
            args_lateral_dict = opt_dict['lateral_control_dict']
        if ' Speed_longitudinal_control_dict' in opt_dict:
             Speed_args_longitudinal_dict = opt_dict['longitudinal_control_dict']
        if ' SpeedDistance_longitudinal_control_dict' in opt_dict:
             SpeedDistance_args_longitudinal_dict = opt_dict['SpeedDistance_longitudinal_control_dict']
        self._current_waypoint = self._map.get_waypoint(
            self._player.get_location())

        self._SpeedDistance_vehicle_controller = SpeedDistance_VehiclePIDController(self._player,None,
                                                        args_lateral=args_lateral_dict,
                                                        args_longitudinal= SpeedDistance_args_longitudinal_dict)

        self._Speed_vehicle_controller = VehiclePIDController(self._player,
                                                              args_lateral=args_lateral_dict,
                                                              args_longitudinal= Speed_args_longitudinal_dict)

        self._global_plan = False

        # compute initial waypoints
        self._waypoints_queue.append( (self._current_waypoint.next(self._sampling_radius)[0], RoadOption.LANEFOLLOW))
        self._target_road_option = RoadOption.LANEFOLLOW
       
        # fill waypoint trajectory queue
        self._compute_next_waypoints(k=240)        

    def _compute_next_waypoints(self, k=1):
        """
        Add new waypoints to the trajectory queue.
        Calculate each waypoint in the path and work out the corresponding operation.

        :param k: how many waypoints to compute
        :return:
        
        """
        # check we do not overflow the queue
        available_entries = self._waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, k)

        for _ in range(k):
            last_waypoint = self._waypoints_queue[-1][0]
            next_waypoints = list(last_waypoint.next(4.0))   
            if len(next_waypoints) == 1:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                road_option = RoadOption.LANEFOLLOW
            else:
                # random choice between the possible options
                road_options_list = retrieve_options(
                    next_waypoints, last_waypoint)
                road_option = random.choice(road_options_list)
                next_waypoint = next_waypoints[road_options_list.index(
                    road_option)]
            
            # print (road_option)
            self._waypoints_queue.append((next_waypoint, road_option))
            # print (str(self._waypoints_queue[1].name))
    
    def run_step(self, debug=True):
        """
        Execute one step of local planning which involves running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

        :param debug: boolean flag to activate waypoints debugging
        :return:
        """
        # self._traffic_detector.search_rear_vehicle_right_lane()
        self.refresh_traffic_detector()
        # print (self._overtake_intention)
        # if len(self._SpeedDistance_vehicle_controller._lon_controller._ed_buffer) > 0:
        #     print (self._SpeedDistance_vehicle_controller._lon_controller._ed_buffer[-1])
        
        # not enough waypoints in the horizon? => add more!
        if len(self._waypoints_queue) < int(self._waypoints_queue.maxlen * 0.5):
            if not self._global_plan:
                self._compute_next_waypoints(k=120)

        if len(self._waypoints_queue) == 0:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False
            control.manual_gear_shift = False

            return control

        #   Buffering the waypoints
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(
                        self._waypoints_queue.popleft())
                else:
                    break

        self._traffic_detector.get_speed_limit()
        
        # current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._player.get_location())
        
        # target waypoint
        vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()                  
        #  refresh the target speed and target distance for PID controller.
        self._target_speed = self.set_target_speed()
        # print (self._target_speed)
        # print ("target distance is " + str(self._target_distance))
        self._target_waypoint, self._target_road_option = self._waypoint_buffer[0]
        # print (self._overtake_intention)
        if self.emergency_brake_situation() is False:
            if self._overtake_intention is False:
                if vehicle_state:
                        self._target_distance = self.set_target_distance() - 20.0
                        self._SpeedDistance_vehicle_controller._lon_controller._front_vehicle = front_vehicle
                        # print ("I am using speed-distance PID.")

                        control = self._SpeedDistance_vehicle_controller.run_step(self._target_speed,self._target_distance, self._target_waypoint)
                else:
                    control = self._Speed_vehicle_controller.run_step(self._target_speed, self._target_waypoint)
                    # print ("I am using speed PID.")
            else:
                self.decide_on_overtake()
                control = self._Speed_vehicle_controller.run_step(self._target_speed, self._target_waypoint)
        else:
            control = self.emergency_stop()
        
        # purge the queue of obsolete waypoints
        vehicle_transform = self._player.get_transform()
        max_index = -1

        for i, (waypoint, _) in enumerate(self._waypoint_buffer):
            if distance_vehicle(
                    waypoint, vehicle_transform) < self._min_distance:
                max_index = i
        
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        if debug:
            draw_waypoints(self._player.get_world(), [self._target_waypoint], self._player.get_location().z + 1.0)

        return control

def retrieve_options(list_waypoints, current_waypoint):
    """
    Compute the type of connection between the current active waypoint and the multiple waypoints present in
    list_waypoints. The result is encoded as a list of RoadOption enums.

    :param list_waypoints: list with the possible target waypoints in case of multiple options
    :param current_waypoint: current active waypoint
    :return: list of RoadOption enums representing the type of connection from the active waypoint to each
            candidate in list_waypoints
    """
    options = []
    for next_waypoint in list_waypoints:
        # this is needed because something we are linking to
        # the beggining of an intersection, therefore the
        # variation in angle is small
        next_next_waypoint = next_waypoint.next(3.0)[0]
        link = compute_connection(current_waypoint, next_next_waypoint)
        options.append(link)

    return options


def compute_connection(current_waypoint, next_waypoint):
    """
    Compute the type of topological connection between an active waypoint (current_waypoint) and a target waypoint
    (next_waypoint).

    :param current_waypoint: active waypoint
    :param next_waypoint: target waypoint
    :return: the type of topological connection encoded as a RoadOption enum:
            RoadOption.STRAIGHT
            RoadOption.LEFT
            RoadOption.RIGHT
    """
    n = next_waypoint.transform.rotation.yaw
    n = n % 360.0

    c = current_waypoint.transform.rotation.yaw
    c = c % 360.0

    diff_angle = (n - c) % 180.0
    if diff_angle < 1.0:
        return RoadOption.STRAIGHT
    elif diff_angle > 90.0:
        return RoadOption.LEFT
    else:
        return RoadOption.RIGHT        

  
       
# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot,controller):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

         # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G923 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G923 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G923 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G923 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G923 Racing Wheel', 'handbrake'))
        self._handbrake_idx = int(self._parser.get('G923 Racing Wheel', 'handbrake'))
        self._clutch_idx = 11

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0 :
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                elif event.button == 2:
                    world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 23:
                    world.camera_manager.next_sensor()
                elif event.button==11 :
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd



        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()



    def check_autopilot_leader(self, leader_vehicle_speed):
            player_speed = self._world.player.get_velocity().length()
            leader_vehicle_speed=self._world.front_vehicle.get_velocity().length()
            if player_speed > leader_vehicle_speed:
                self._autopilot_enabled = True
                self._world.player.set_autopilot(True)

            
    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.world.get_map().name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            'Traffic_lights:% 16d FPS' % self.server_fps,
            '',  
            ]
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:%s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        self._notifications.tick(world, clock)

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.lane_markings = []  # Initialize an empty list for lane markings

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lane_markings = event.crossed_lane_markings  # Update lane_markings attribute
        lane_types = set(x.type for x in self.lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))
        
# ==============================================================================
# -- get_curvature_function --------------------------------------------------------
# ==============================================================================


    '''def get_curvature(self):
        if len(self.lane_markings) < 1:
            return None
        # Extract the lane marking positions as x,y coordinates
        positions = [lm.transform.location.to_carla() for lm in self.lane_markings]
        x = [p.x for p in positions]
        y = [p.y for p in positions]

        # Fit a second-order polynomial to the lane marking positions
        fit = np.polyfit(x, y, 2)

        # Compute the radius of curvature from the polynomial coefficients
        a, b, c = fit
        curvature = ((1 + (2 * a) ** 2) ** 1.5) / (2 * abs(a))

        return curvature

    def destroy(self):
        """
        Destroy the sensor actor and free up resources.
        """
        if self.sensor is not None:
            self.sensor.destroy()
            self.sensor = None'''


# ==============================================================================
# -- IMUSensor------------------------------------------------------------------
# ==============================================================================
class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        self.orientation = Quaternion()  # Initialize with identity quaternion
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))


    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        #print(sensor_data)
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
        self.orientation = sensor_data.transform.rotation

    '''#### added
    def _process_imu(self, imu_data):
        self.angular_velocity = imu_data.gyroscope
        self.orientation = imu_data.rotation'''

    def get_yaw_angle(self):
        # Get the yaw angle from the transform

        yaw = math.degrees(self.sensor.get_transform().rotation.yaw)


        return yaw

    '''def get_yaw_angle(self):
        # Convert the orientation quaternion to a rotation matrix
        R = self.orientation.rotation_matrix

        # Extract the yaw angle from the rotation matrix
        yaw = np.arctan2(R[1, 0], R[0, 0])

        return yaw'''

    def destroy(self):
        """
        Destroy the sensor actor and free up resources.
        """
        if self.sensor is not None:
            self.sensor.destroy()
            self.sensor = None

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


# ==============================================================================
# -- NewCameraManager -------------------------------------------------------------
# ==============================================================================
'''
This is a new version of NewCameraManager which supports adding different view of Supporting Sensors.
Remember to add the following codes in the world.destroy() function.


        for item in self.camera_manager.SupportingSensors:
            actors.append(item)


'''

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.SupportingSensors = []
        self._surface = None
        # self._surface_1 = pygame.Surface((hud.dim[0]/4,hud.dim[1]/4))
        self._parent = parent_actor
        self._hud = hud
        self._SupportSurfaceList = [pygame.Surface((hud.dim[0]/3.5,hud.dim[1]/3.5)) for i in range(1,4)]
        self._recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-10, z=4.8), carla.Rotation(pitch=-17)),
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            carla.Transform(carla.Location(x=0.1,y=-0.3, z=1.2), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=0.2, z=1.6), carla.Rotation(pitch=0)),

            ]
        
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        self._blueprints = self._parent.get_world().get_blueprint_library()
        self._world = self._parent.get_world()
        bp_library = self._world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            item.append(bp)
        self._index = None

    def set_support_sensors(self):
        bp_library = self._blueprints
        bp_SupportingSensors = []
        
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(self._hud.dim[0]/3.5))
                bp.set_attribute('image_size_y', str(self._hud.dim[1]/3.5))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            bp_SupportingSensors.append(bp)
      
        Camera_1 = self._world.spawn_actor(
                            bp_SupportingSensors[5],
                            self._camera_transforms[4],
                            attach_to=self._parent)
        
        Camera_2 = self._world.spawn_actor(
                            bp_SupportingSensors[0],
                            self._camera_transforms[2],
                            attach_to=self._parent)
        
        Camera_3 = self._world.spawn_actor(
                            bp_SupportingSensors[3],
                            self._camera_transforms[4],
                            attach_to=self._parent)

        self.SupportingSensors.append(Camera_1)
        self.SupportingSensors.append(Camera_2)
        self.SupportingSensors.append(Camera_3)

        weak_self = weakref.ref(self)

        Camera_1.listen(lambda image: CameraManager._camera1_display(weak_self,image))      
        Camera_2.listen(lambda image: CameraManager._camera2_display(weak_self,image))      
        Camera_3.listen(lambda image: CameraManager._camera3_display(weak_self,image))      
    
    def toggle_camera(self):
        # This is used to change the installation location of the sensors or cameras.
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        self.set_support_sensors()
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:          

                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
            # display.blit(self._SupportSurfaceList[0],(480,100))
            # display.blit(self._surface_1,(480,100))

            display.blit(self._SupportSurfaceList[0],(10,490))
            display.blit(self._SupportSurfaceList[1],(460,490))
            display.blit(self._SupportSurfaceList[2],(910,490))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

    @staticmethod
    def _camera1_display(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self._sensors[5][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self._SupportSurfaceList[0] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

    @staticmethod
    def _camera2_display(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self._sensors[0][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._SupportSurfaceList[1] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

    @staticmethod
    def _camera3_display(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self._sensors[3][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._SupportSurfaceList[2] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

# ==============================================================================
# -- Recorder() ----------------------------------------------------------------
# ==============================================================================
class Recorder(object):
    def __init__(self,player,controller,workbook):
        self.player = player
        self.world = player.get_world()
        self.controller = controller
        self.front_vehicle_state, self.front_vehicle = controller._traffic_detector.search_front_vehicle()
        self.workbook = workbook
        self.counter = 1
        self.sheetname = "Raw_data"
        self.sheet = workbook.add_sheet(self.sheetname)
        self.vehicle_list = self.world.get_actors().filter("vehicle*")
        self.workbookname =  str(time.strftime('%Y.%m.%d_%H%M%S',time.localtime(time.time()))) \
                                    + '_ID_' + str(player.id) + '.xls'
        
        self.sheet.write(0,0,"Ego_Speed")
        self.sheet.write(0,1,"Ego_TargetSpeed")        
        self.sheet.write(0,2,"Ego_Acceleration")        
        self.sheet.write(0,3,"Ego_Throttle")
        self.sheet.write(0,4,"Ego_Steering")
        self.sheet.write(0,5,"Ego_Brake")
        self.sheet.write(0,6,"Ego_Location_x")
        self.sheet.write(0,7,"Ego_Location_y")
        self.sheet.write(0,8,"Ego_Rotation_Yaw")
        self.sheet.write(0,9,"Ego_Rotation_Pitch")
        self.sheet.write(0,10,"Ego_Rotation_Roll")
        self.sheet.write(0,11,"Front_Speed")
        self.sheet.write(0,12,"Front_Acceleration")
        self.sheet.write(0,13,"Front_Throttle")
        self.sheet.write(0,14,"Front_Steering")
        self.sheet.write(0,15,"Front_Brake")
        self.sheet.write(0,16,"Front_Location_x")
        self.sheet.write(0,17,"Front_Location_y")
        self.sheet.write(0,18,"Front_Rotation_Yaw")
        self.sheet.write(0,19,"Front_Rotation_Pitch")
        self.sheet.write(0,20,"Front_Rotation_Roll")
        self.sheet.write(0,21,"Relative_Distance")
        self.sheet.write(0,22,"Target_Relative_Distance")
        self.sheet.write(0,23,"Target_location_x")
        self.sheet.write(0,24,"Target_location_y")
        self.sheet.write(0,25,"Left_rear_car")
        self.sheet.write(0,26,"Left_front_car")
        self.sheet.write(0,27,"Right_rear_car")
        self.sheet.write(0,28,"Right_front_car")
        self.sheet.write(0,29,"Right_rear_car_x")
        self.sheet.write(0,30,"Right_rear_car_y")
        self.sheet.write(0,31,"Right_rear_car_speed")
        self.sheet.write(0,32,"Right_rear_car_S_critical")
        self.sheet.write(0,33,"Right_rear_car_relative_distance")
        self.sheet.write(0,34,"Left_rear_car_x")
        self.sheet.write(0,35,"Left_rear_car_y")
        self.sheet.write(0,36,"Left_rear_car_speed")
        self.sheet.write(0,37,"Left_rear_car_S_critical")
        self.sheet.write(0,38,"Left_rear_car_relative_distance")
        self.sheet.write(0,39,"Left_front_car_x")
        self.sheet.write(0,40,"Left_front_car_y")
        self.sheet.write(0,41,"Left_front_car_speed")
        self.sheet.write(0,42,"Left_front_car_relative_distance")
        self.sheet.write(0,43,"Right_front_car_x")
        self.sheet.write(0,44,"Right_front_car_y")
        self.sheet.write(0,45,"Right_front_car_speed")
        self.sheet.write(0,46,"Right_front_car_relative_distance")
        self.sheet.write(0,47,"overtake_intention")
        self.sheet.write(0,48,"overtake_target")
        self.sheet.write(0,49,"overtake_target_speed")

    def start_recorder(self):
        wb = self.workbook
        player = self.player
        controller = self.controller
        front_vehicle_state, front_vehicle = controller._traffic_detector.search_front_vehicle()
        rear_right_vehicle_state, rear_vehicle_right_lane = controller._traffic_detector.search_rear_vehicle_right_lane()
        rear_left_vehicle_state, rear_vehicle_left_lane = controller._traffic_detector.search_rear_vehicle_left_lane()
        front_right_vehicle_state, front_vehicle_right_lane = controller._traffic_detector.search_front_vehicle_right_lane()
        front_left_vehicle_state, front_vehicle_left_lane = controller._traffic_detector.search_front_vehicle_left_lane()
        overtake_vehicle_state, overtake_vehicle = controller.search_overtake_vehicle()                
        row = self.counter
        worksheet = self.sheet
        
        # Export the data of Ego car.
        ego_speed = get_speed(player)
        
        ego_target_speed = controller.set_target_speed()
        
        ego_acceleration_vector = player.get_acceleration()
        # ego_acceleration = math.sqrt(ego_acceleration_vector.x**2 + ego_acceleration_vector.y**2 + ego_acceleration_vector.z**2)
        ego_acceleration = ego_acceleration_vector.x   
        ego_control = player.get_control()
        ego_throttle = ego_control.throttle
        ego_steering = ego_control.steer
        ego_brake = ego_control.brake        

        ego_location_x = player.get_location().x
        ego_location_y = player.get_location().y
        
        ego_rotation_yaw = player.get_transform().rotation.yaw
        ego_rotation_pitch = player.get_transform().rotation.pitch
        ego_rotation_roll = player.get_transform().rotation.roll

        target_location_x = controller._target_waypoint.transform.location.x
        target_location_y = controller._target_waypoint.transform.location.y

        worksheet.write(row,0,ego_speed)
        worksheet.write(row,1,ego_target_speed)        
        worksheet.write(row,2,ego_acceleration)
        worksheet.write(row,3,ego_throttle )
        worksheet.write(row,4,ego_steering)
        worksheet.write(row,5,ego_brake)
        worksheet.write(row,6,ego_location_x)
        worksheet.write(row,7,ego_location_y)
        worksheet.write(row,8, ego_rotation_yaw)
        worksheet.write(row,9, ego_rotation_pitch)
        worksheet.write(row,10, ego_rotation_roll)
        worksheet.write(row,23, target_location_x)
        worksheet.write(row,24, target_location_y)
        worksheet.write(row,32,controller._traffic_detector.rr_s_critical)
        worksheet.write(row,37,controller._traffic_detector.rl_s_critical)
        worksheet.write(row,47, controller._overtake_intention)

        # Export the data of Front car.
        if overtake_vehicle_state:
            worksheet.write(row,48,overtake_vehicle.id)
            worksheet.write(row,49,get_speed(overtake_vehicle))

        if front_vehicle_state:
            front_speed = get_speed(front_vehicle)
                        
            front_acceleration_vector = front_vehicle.get_acceleration()
            # front_acceleration = math.sqrt(front_acceleration_vector.x**2 + front_acceleration_vector.y**2 + front_acceleration_vector.z**2)
            front_acceleration = front_acceleration_vector.x
            front_control = front_vehicle.get_vehicle_control()
            front_throttle = front_control.throttle
            front_steering = front_control.steer
            front_brake = front_control.brake        

            front_location_x = front_vehicle.get_location().x
            front_location_y = front_vehicle.get_location().y
            
            front_rotation_yaw = front_vehicle.get_transform().rotation.yaw
            front_rotation_pitch = front_vehicle.get_transform().rotation.pitch
            front_rotation_roll = front_vehicle.get_transform().rotation.roll

            relative_distance = math.sqrt((front_location_x - ego_location_x)**2 + (front_location_y - ego_location_y)**2)
            target_relative_distance = controller.set_target_distance()
            
            worksheet.write(row,11,front_speed)
            worksheet.write(row,12,front_acceleration)        
            worksheet.write(row,13,front_throttle)
            worksheet.write(row,14,front_steering)
            worksheet.write(row,15,front_brake)
            worksheet.write(row,16,front_location_x)
            worksheet.write(row,17,front_location_y)
            worksheet.write(row,18,front_rotation_yaw)
            worksheet.write(row,19,front_rotation_pitch)
            worksheet.write(row,20,front_rotation_roll)
            worksheet.write(row,21,relative_distance)
            worksheet.write(row,22,target_relative_distance)
        
        if rear_right_vehicle_state:
            rr_location_x =  rear_vehicle_right_lane.get_location().x
            rr_location_y =  rear_vehicle_right_lane.get_location().y           
            rr_speed = get_speed(rear_vehicle_right_lane)
            rr_distance = ego_location_x - rr_location_x
            
            worksheet.write(row,27,rear_vehicle_right_lane.id)
            worksheet.write(row,29,rr_location_x)
            worksheet.write(row,30,rr_location_y)
            worksheet.write(row,31,rr_speed)
            worksheet.write(row,33,rr_distance)

        if rear_left_vehicle_state:
            rl_location_x =  rear_vehicle_left_lane.get_location().x
            rl_location_y =  rear_vehicle_left_lane.get_location().y           
            rl_speed = get_speed(rear_vehicle_left_lane)
            rl_distance = ego_location_x - rl_location_x

            worksheet.write(row,25,rear_vehicle_left_lane.id)
            worksheet.write(row,34,rl_location_x)
            worksheet.write(row,35,rl_location_y)
            worksheet.write(row,36,rl_speed)
            worksheet.write(row,38,rl_distance)

        if front_left_vehicle_state:
            fl_location_x = front_vehicle_left_lane.get_location().x
            fl_location_y = front_vehicle_left_lane.get_location().y           
            fl_speed = get_speed(front_vehicle_left_lane)
            fl_distance = - ego_location_x + fl_location_x

            worksheet.write(row,26,front_vehicle_left_lane.id)
            worksheet.write(row,39,fl_location_x)
            worksheet.write(row,40,fl_location_y)
            worksheet.write(row,41,fl_speed)
            worksheet.write(row,42,fl_distance)

        if front_right_vehicle_state:
            fr_location_x =  front_vehicle_right_lane.get_location().x
            fr_location_y =  front_vehicle_right_lane.get_location().y           
            fr_speed = get_speed(front_vehicle_right_lane)
            fr_distance = -ego_location_x + fr_location_x

            worksheet.write(row,28,front_vehicle_right_lane.id)
            worksheet.write(row,43,fr_location_x)
            worksheet.write(row,44,fr_location_y)
            worksheet.write(row,45,fr_speed)
            worksheet.write(row,46,fr_distance)

        self.counter += 1
        wb.save(self.workbookname)

    def finish_recorder(self):
        wb = self.workbook
        wb.save()


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    supporting_actor_list = []

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)

        world = World(client.get_world(), hud, args.filter)
        controller = DualControl(world, args.autopilot,ACC_Controller)
        setting=world.world.get_settings()
        setting.fixed_delta_seconds=0.05
        world.world.apply_settings(setting)
        transform_2 = carla.Transform (carla.Location(x=2400, y=-8.25, z=0.4),carla.Rotation(pitch=0, yaw=180, roll=0))
        transform_3 = carla.Transform (carla.Location(x=-2410, y=5.7, z=0.4),carla.Rotation(pitch=0, yaw=0, roll=0))
        #transform_3 = carla.Transform (carla.Location(x=-2410, y=16, z=0.4),carla.Rotation(pitch=0, yaw=0, roll=0))
        transform_4 = carla.Transform(carla.Location(x=-2405 ,y=12.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0))
        transform_5 = carla.Transform(carla.Location(x=-2430 ,y=15.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0))
        transform_6 = carla.Transform(carla.Location(x=-2420 ,y=8.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0)) # THis is for left front block vehicle test.
        transform_7 = carla.Transform(carla.Location(x=-2420 ,y=15.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0)) # THis is for right lane change vehicle test.

        follow_test_point = world.world.get_map().get_spawn_points()[0]
        
        player = world.get_player_vehicle()
        physics_vehicle = world.player.get_physics_control()
        car_mass = physics_vehicle.mass
        print(car_mass)
        spawn_number = 0
        bp_stream = world.world.get_blueprint_library().filter('vehicle*')
        bp_stream = [x for x in bp_stream if int(x.get_attribute('number_of_wheels')) == 4]
        bp_stream = [x for x in bp_stream if not x.id.endswith('isetta')]
        
        
        for vehicle_index in range(0,spawn_number):
            vehicle_model = random.choice(bp_stream)
            vehicle_model.set_attribute('role_name','autopilot')
            vehicle_actor = world.world.try_spawn_actor(vehicle_model,random.choice(world.world.get_map().get_spawn_points()))
            if vehicle_actor is not None:
                supporting_actor_list.append(vehicle_actor)
                # vehicle_actor.apply_control (carla.VehicleControl (throttle = 0.5, steer=0.0, brake =0.0) )
                vehicle_actor.set_autopilot(True)
               
            # print (vehicle_actor.get_transform())
        
        hero = world.player
        ACC_controller =ACC_Controller(hero)
        clock = pygame.time.Clock()
        wb = xlwt.Workbook()
        ego_recorder = Recorder(hero,ACC_controller,wb)
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            #control = ACC_controller.run_step()
            #world.player.apply_control(control)
            '''ego_recorder.start_recorder()'''
           # lane_invasion_sensor = LaneInvasionSensor(vehicle, hud)
            #imu_sensor = IMUSensor(vehicle)
            #Call the get_curvature() method on the created object
           # curvature = lane_invasion_sensor.get_curvature()
            #yaw_angle = imu_sensor.get_yaw_angle()
            #print(yaw_angle)

            #print(curvature)
           #imu_sensor.destroy()

    finally:

        if world is not None:
            world.destroy()
        print('\ndestroying %d actors' % len(supporting_actor_list))
        for actor in supporting_actor_list:
            actor.destroy() 
        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-ap', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Roaming", "Basic"],
                           help="select which agent to run",
                           default="Basic")
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default=' 1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)
    actor_list = []
    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)

    finally:
        print('\ndestroying %d actors' % len(actor_list))
        for actor in actor_list:
            actor.destroy()

if __name__ == '__main__':

    main()
