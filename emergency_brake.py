#!/usr/bin/env python

# Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import gym
import gym_carla

import stable_baselines as sb

import os
import sys
import glob

from agents.navigation.basic_agent import BasicAgent

import random

from policies import *

# parameters for the gym_carla environment
params_standard = {
  'number_of_vehicles': 100,
  'number_of_walkers': 0,
  'display_size': 600,  # screen size of bird-eye render
  'max_past_step': 1,  # the number of past steps to draw
  'dt': 0.1,  # time interval between two frames
  'discrete': False,  # whether to use discrete control space
  'discrete_acc': [-3.0, 0.0, 3.0],  # discrete value of accelerations
  'discrete_steer': [-0.3, 0.0, 0.3],  # discrete value of steering angles
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

params_emergency_break = {
  'number_of_vehicles': 0,
  'number_of_walkers': 0,
  'display_size': 256,  # screen size of bird-eye render
  'max_past_step': 1,  # the number of past steps to draw
  'dt': 0.1,  # time interval between two frames
  'discrete': False,  # whether to use discrete control space
  'discrete_acc': [-3.0, 0.0, 3.0],  # discrete value of accelerations
  'discrete_steer': [-0.3, 0.0, 0.3],  # discrete value of steering angles
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

try:
    # /opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg
    # /home/agardille/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg      

    sys.path.append(glob.glob('/home/agardille/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print('CARLA not imported')
    pass

import carla

def main():
  # Set gym-carla environment
  env = gym.make('carla-v0', params=params_emergency_break)
  obs = env.reset()

  no_obstacle = True

  #model = ForwardAgent()
  model = ControlAgent()
  total_reward = 0.0

  basic_agent = env.get_basic_agent()

  while True:
    
    action, _state = model.predict(obs, deterministic=True)
    #print('action: ', action)

    control = basic_agent.run_step()
    print("steer: ", control.steer)

    obs,r,done,info = env.step(action)
    total_reward += r

    if no_obstacle and random.randint(0, 100)==0 :
      no_obstacle = env.add_obstacle()
    #print('state: ', obs['state'])

    if done:
      print('total reward: ', total_reward)
      total_reward = 0.0
      no_obstacle = True
      action = [0.0, 0.0]
      print('state: ', obs['state'].shape)
      print('camera: ', obs['camera'].shape)
      print('birdeye: ', obs['birdeye'].shape)
      print('lidar: ', obs['lidar'].shape)
      obs = env.reset()


if __name__ == '__main__':
  main()