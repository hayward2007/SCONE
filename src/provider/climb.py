import time;

from ..core import *;
from .. import core, provider;
# from .mode import Mode;
# from .walk import Walk;

class Climb(provider.Mode) :
    def __init__(self, mode: provider.Mode) :
        # sync
        self.controller = mode.controller;

        self.upper_initial_position = mode.upper_initial_position;
        self.middle_initial_position = mode.middle_initial_position;
        self.lower_initial_position = mode.lower_initial_position;

        self.boost_speed = mode.boost_speed;
        self.safety_speed = mode.safety_speed;
        self.walking_speed = mode.walking_speed;
        self.driving_speed = mode.driving_speed;
        self.climbing_speed = mode.climbing_speed;

    def __del__(self) :
        pass;

    def change_mode(self) :
        return provider.Walk(self);