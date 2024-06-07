import time;

from ..core import *;
from ..provider import *;

class Climb(Mode) :
    def __init__(self, mode: Mode) :
        # sync
        self.controller = mode.controller;

        self.upper_initial_position = mode.upper_initial_position;
        self.middle_initial_position = mode.middle_initial_position;
        self.lower_initial_position = mode.lower_initial_position;

        self.boost_speed = mode.boost_speed;
        self.safety_speed = mode.safety_speed;
        self.walking_speed = mode.walking_speed;
        self.driving_speed = mode.driving_speed;