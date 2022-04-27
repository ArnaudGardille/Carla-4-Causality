
import stable_baselines as sb
from stable_baselines.common.policies import BasePolicy


import collections
import copy
import warnings
from abc import ABC, abstractmethod
from functools import partial
from typing import Any, Dict, List, Optional, Tuple, Type, Union

import gym
import numpy as np
import torch as th
from torch import nn

import pygame
from pygame.locals import K_w
from pygame.locals import K_a
from pygame.locals import K_s
from pygame.locals import K_d

class ControlAgent():
    def predict(self, obs, deterministic=True):
        keys=pygame.key.get_pressed()
        action = [ 0.0, 0.0]
        if keys[K_w]:
            action[0] = 1.0
        elif keys[K_s]:
            action[0] = -1.0
        if keys[K_a]:
            action[1] = 0.3
        elif keys[K_d]:
            action[1] = -0.3
        pygame.event.pump() # process event queue
        return (action, 0)

class ForwardAgent():
    def predict(self, obs, deterministic=True):
        return ([1.0, 0.0], 0)
    