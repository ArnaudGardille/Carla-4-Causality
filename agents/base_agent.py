
import stable_baselines3 
from stable_baselines3.common.policies import BasePolicy
from stable_baselines3.common.base_class import BaseAlgorithm


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


class ForwardPolicy(BasePolicy):
    def _predict(self, observation: th.Tensor, deterministic: bool = False) -> th.Tensor:
        return th.Tensor([1.0, 0.0])

if __name__ == '__main__':
    env = gym.make('CartPole-v1')
    model = BaseAlgorithm


class Agent:
    def __init__(self) -> None:
        pass

    def __call__(self, s : np.ndarray):
        pass

    def predict(
        self,
        observation: np.ndarray):
        pass

class ForwardAgent(Agent):
    def predict(
        self,
        observation: np.ndarray):
        pass


    