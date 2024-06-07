import time;

from ..core import *;
from ..provider import *;

class Drive(Mode) :
    def __init__(self, mode: Mode) :
        # sync
        self.controller = mode.controller;

        self.upper_initial_position = mode.upper_initial_position;
        self.middle_initial_position = mode.middle_initial_position;
        self.lower_initial_position = mode.lower_initial_position;

        self.boost_speed = mode.boost_speed;
        self.safety_speed = mode.safety_speed;
        self.walking_speed = mode.walking_speed;

        # initialize operating mode
        for i in Actuator.index :
            self.controller.set_torque(i, Actuator.torque.off);
            self.controller.set_mode(i, Actuator.model.XM.operating_mode.position);
            self.controller.set_acceleration(i, 20);
        time.sleep(0.1);

        # initialize position
        self.controller.enable_torque();
        self.controller.set_all_speed(self.safety_speed);

        for i in Actuator.middle_index :
            self.controller.set_position(i, self.__starting_middle_position);
        time.sleep(0.5);

        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        for i in Actuator.lower_index :
            self.controller.set_speed(i, self.boost_speed);
            self.controller.set_position(i, self.lower_initial_position);
        time.sleep(0.7);

        self.controller.set_all_speed(self.safety_speed);
        for i in Actuator.middle_index :
            self.controller.set_position(i, self.middle_initial_position);
        time.sleep(1);

        self.controller.set_all_speed(self.walking_speed);