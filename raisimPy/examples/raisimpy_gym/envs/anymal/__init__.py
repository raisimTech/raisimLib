
import os

ANYMAL_RESOURCE_DIRECTORY = os.path.dirname(os.path.realpath(__file__)) + '/../../../rsc/ANYmal/'
ANYMAL_CONFIG_DIRECTORY = os.path.dirname(os.path.abspath(__file__)) + '/'

from .env import AnymalEnv
