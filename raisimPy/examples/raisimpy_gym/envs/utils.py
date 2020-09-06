#!/usr/bin/env python
"""This contains utils functions to load a YAML file, or detect a keyboard interrupt exception.
"""

import sys
import yaml


def keyboard_interrupt(func):
    """Decorator to be used on a method to check if there was a keyboard interrupt error that was raised."""
    def wrap(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except KeyboardInterrupt:
            self.close()  # this will close the visualizer if necessary
            sys.exit(0)
    return wrap


def load_yaml(filename):
    with open(filename, "rb") as f:
        try:
            node = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)
    return node


# Test
if __name__ == '__main__':
    import os

    config = load_yaml(os.path.abspath('anymal/default_cfg.yaml'))

    print("Config: {}, {}".format(type(config), config))