#!/usr/bin/env python

# Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import gym
import gym_carla

import os
import sys
import glob

import pygame
from pygame.locals import K_w
from pygame.locals import K_a
from pygame.locals import K_s
from pygame.locals import K_d

try:
    # /home/agardille/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg

    print('/home/agardille/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))

    sys.path.append(glob.glob('/home/agardille/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print('CARLA not imported')
    pass

import carla

def main():
  # parameters for the gym_carla environment
  params = {
    'number_of_vehicles': 100,
    'number_of_walkers': 0,
    'display_size': 300,  # screen size of bird-eye render
    'max_past_step': 1,  # the number of past steps to draw
    'dt': 0.1,  # time interval between two frames
    'discrete': False,  # whether to use discrete control space
    'discrete_acc': [-3.0, 0.0, 3.0],  # discrete value of accelerations
    'discrete_steer': [-0.2, 0.0, 0.2],  # discrete value of steering angles
    'continuous_accel_range': [-3.0, 3.0],  # continuous acceleration range
    'continuous_steer_range': [-0.3, 0.3],  # continuous steering angle range
    'ego_vehicle_filter': 'vehicle.lincoln*',  # filter for defining ego vehicle
    'port': 2000,  # connection port
    'town': 'Town03',  # which town to simulate
    'task_mode': 'random',  # mode of the task, [random, roundabout (only for Town03)]
    'max_time_episode': 1000,  # maximum timesteps per episode
    'max_waypt': 12,  # maximum number of waypoints
    'obs_range': 32,  # observation range (meter)
    'lidar_bin': 0.125,  # bin size of lidar sensor (meter)
    'd_behind': 12,  # distance behind the ego vehicle (meter)
    'out_lane_thres': 2.0,  # threshold for out of lane
    'desired_speed': 8,  # desired speed (m/s)
    'max_ego_spawn_times': 200,  # maximum times to spawn ego vehicle
    'display_route': True,  # whether to render the desired route
    'pixor_size': 64,  # size of the pixor labels
    'pixor': False,  # whether to output PIXOR observation
  }

  # Set gym-carla environment
  env = gym.make('carla-v0', params=params)
  obs = env.reset()

  action = [0.0, 0.0]
  while True:
    

    '''keys = pygame.key.get_pressed()
    print(keys[K_w])
    if keys[K_w]:
      print('w')
      action[0] = 1.0
    if keys[K_a]:
      action[1] = -0.1
    if keys[K_s]:
      action[0] = -1.0
    if keys[K_d]:
      action[1] = 0.1'''

    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        return True
      elif event.type == pygame.KEYDOWN: #KEYUP:
        print('key')
        if event.key == K_w:
          print('W')
          action[0] += 1.0
        elif event.key == K_a:
          action[1] += 0.1
        elif event.key == K_s:
          action[0] += -1.0
        elif event.key == K_d:
          action[1] += -0.1
          

    obs,r,done,info = env.step(action)
    print(r)
    #print('state: ', obs['state'])

    if done:
      action = [0.0, 0.0]
      print('state: ', obs['state'].shape)
      print('camera: ', obs['camera'].shape)
      print('birdeye: ', obs['birdeye'].shape)
      print('lidar: ', obs['lidar'].shape)
      obs = env.reset()


if __name__ == '__main__':
  main()