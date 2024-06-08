import time;

from ..core import *;
from .. import provider;

class Climb(provider.Mode) :
    def __init__(self, mode: provider.Mode) :
        self.controller = mode.controller;

        self.upper_initial_position = mode.upper_initial_position;
        self.middle_initial_position = mode.middle_initial_position;
        self.lower_initial_position = mode.lower_initial_position;

        self.boost_speed = mode.boost_speed;
        self.safety_speed = mode.safety_speed;
        self.walking_speed = mode.walking_speed;
        self.driving_speed = mode.driving_speed;
        self.climbing_speed = mode.climbing_speed;
    
        self.controller.set_all_mode(Actuator.model.XM.operating_mode.position);
        self.controller.set_all_speed(self.safety_speed);

        for i in Actuator.middle_diagonal_left_index :
            self.controller.set_position(i, self.middle_initial_position - 20);
        time.sleep(0.5);

        for i in Actuator.lower_diagonal_left_index :
            self.controller.set_speed(i, self.boost_speed);
            self.controller.set_raw_position(i, Actuator.position.center);
        time.sleep(0.5);
    
        for i in Actuator.middle_diagonal_left_index :
            self.controller.set_raw_position(i, Actuator.position.center);
        time.sleep(0.5);

        for i in Actuator.middle_diagonal_right_index :
            self.controller.set_position(i, self.middle_initial_position - 20);
        time.sleep(0.5);
    
        for i in Actuator.lower_diagonal_right_index :
            self.controller.set_speed(i, self.boost_speed);
            self.controller.set_raw_position(i, Actuator.position.center);
        time.sleep(0.5);
    
        for i in Actuator.middle_diagonal_right_index :
            self.controller.set_raw_position(i, Actuator.position.center);
        time.sleep(0.5);
    
    def __left_stance(self) :
        for i in Actuator.lower_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, 270);
        for i in Actuator.middle_right_index :
            self.controller.set_position(i, 270);
        time.sleep(1);

        self.controller.set_all_mode(Actuator.model.XM.operating_mode.velocity);
    
    def __right_stance(self) :
        for i in Actuator.lower_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, 90);
        for i in Actuator.middle_left_index :
            self.controller.set_position(i, 90);
        time.sleep(1);

        self.controller.set_all_mode(Actuator.model.XM.operating_mode.velocity);
    
    def left(self) :
        self.__left_stance();

        for i in Actuator.lower_index :
            self.controller.set_speed(i, self.climbing_speed);
        time.sleep(1);
    
        for i in Actuator.lower_index :
            self.controller.set_speed(i, 0);
    
    def right(self) :
        self.__right_stance();

        for i in Actuator.middle_right_index :
            self.controller.set_speed(i, self.climbing_speed);
        time.sleep(1);
    
        for i in Actuator.middle_right_index :
            self.controller.set_speed(i, 0);

    def change_mode(self) :
        return provider.Walk(self);