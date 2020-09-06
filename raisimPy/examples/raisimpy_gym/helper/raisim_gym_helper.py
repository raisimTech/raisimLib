#!/usr/bin/env python
"""Helper classes and methods.

This is the same code as given in [1].

References:
    - [1] https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/helper/raisim_gym_helper.py
"""

__author__ = ["Jemin Hwangbo (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)"]
__license__ = "MIT"

from shutil import copyfile
import datetime
import os
import ntpath


class ConfigurationSaver:
    def __init__(self, log_dir, save_items):
        self._data_dir = log_dir + '/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        os.makedirs(self._data_dir)

        if save_items is not None:
            for save_item in save_items:
                base_file_name = ntpath.basename(save_item)
                copyfile(save_item, self._data_dir + '/' + base_file_name)

    @property
    def data_dir(self):
        return self._data_dir
        

def TensorboardLauncher(directory_path):
    from tensorboard import program
    import webbrowser
    # learning visualizer
    tb = program.TensorBoard()
    tb.configure(argv=[None, '--logdir', directory_path])
    url = tb.launch()
    webbrowser.open_new(url)
