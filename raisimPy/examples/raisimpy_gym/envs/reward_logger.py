#!/usr/bin/env python
"""Reward Logger

This is the Python version of the reward logger class as described in [1].

References:
    - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/RewardLogger.hpp
"""

import raisimpy as raisim

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)"]
__license__ = "MIT"


class RewardLogger(object):

    class RewardTerm(object):

        def __init__(self):
            self.sum = 0.
            self.count = 0
            self.values = []

        def clean(self):
            self.sum = 0.
            self.count = 0
            self.values = []

        def log(self, value):
            self.values.append(value)
            self.sum += value
            self.count += 1

    def __init__(self):
        self.reward_terms = dict()  # {str: RewardTerm}

    def init(self, reward_term_names):
        for item in reward_term_names:
            self.reward_terms[item] = self.RewardTerm()

    def log(self, term_name, value):
        self.reward_terms[term_name].log(value)

    def get_reward_terms(self):
        return self.reward_terms

    def clean(self):
        for item in self.reward_terms.values():
            item.clean()


# global reward logger
raisim.gui.reward_logger = RewardLogger()
