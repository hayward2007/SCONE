import time;

from ..core import *;
from .. import provider;

class Drive(provider.Mode) :
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

        self.controller.set_all_mode(Actuator.model.XM.operating_mode.velocity);

    def __del__(self) :
        pass;
    
    def left(self) :
        for i in Actuator.lower_index :
            self.controller.set_speed(i, - self.driving_speed, Actuator.model.XM.operating_mode.velocity);
        time.sleep(0.5);
    
        for i in Actuator.lower_index :
            self.controller.set_speed(i, 0, Actuator.model.XM.operating_mode.velocity);
    
    def right(self) :
        for i in Actuator.lower_index :
            self.controller.set_speed(i, self.driving_speed, address = Actuator.model.XM.address.goal_velocity);
        time.sleep(0.5);
    
        for i in Actuator.lower_index :
            self.controller.set_speed(i, 0, address = Actuator.model.XM.address.goal_velocity);

    def change_mode(self) :
        return provider.Climb(self);