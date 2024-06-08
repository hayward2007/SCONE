import time;

from ..core import *;
from .. import provider;

class Walk(provider.Mode) :
    # the moving degree of each step
    __moving_degree = 15;

    def __init__(self, mode: provider.Mode, is_initial: bool = False) :
        self.controller = mode.controller;

        self.upper_initial_position = mode.upper_initial_position;
        self.middle_initial_position = mode.middle_initial_position;
        self.lower_initial_position = mode.lower_initial_position;

        self.boost_speed = mode.boost_speed;
        self.safety_speed = mode.safety_speed;
        self.walking_speed = mode.walking_speed;
        self.driving_speed = mode.driving_speed;
        self.climbing_speed = mode.climbing_speed;
    
        if not is_initial :
            self.controller.set_all_mode(Actuator.model.XM.operating_mode.position);

            for i in Actuator.middle_index :
                self.controller.set_position(i, self.middle_initial_position);
            time.sleep(0.05);

            self.__hold_dignoal_left();
            time.sleep(0.05);

            for i in Actuator.lower_diagonal_left_index :
                self.controller.set_speed(i, self.boost_speed);
                self.controller.set_position(i, self.lower_initial_position);
            time.sleep(0.3);

            for i in Actuator.upper_diagonal_left_index :
                self.controller.set_position(i, self.upper_initial_position[i - 1]);
            time.sleep(0.5);
            
            self.__release_dignoal_left();
            self.__hold_dignoal_right();
            time.sleep(0.05);

            for i in Actuator.lower_diagonal_right_index :
                self.controller.set_speed(i, self.boost_speed);
                self.controller.set_position(i, self.lower_initial_position);
            time.sleep(0.3);

            for i in Actuator.upper_diagonal_right_index :
                self.controller.set_position(i, self.upper_initial_position[i - 1]);
            time.sleep(0.5);
            
            self.__release_dignoal_right();
            time.sleep(0.05);
    
    def __del__(self) :
        pass;

    
    def __hold_dignoal_left(self) :
        for i in Actuator.middle_diagonal_left_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, self.middle_initial_position - self.__moving_degree);
        time.sleep(0.5);
    
    def __hold_dignoal_right(self) :
        for i in Actuator.middle_diagonal_right_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, self.middle_initial_position - self.__moving_degree);
        time.sleep(0.5);

    def __release_dignoal_left(self) :
        for i in Actuator.middle_diagonal_left_index :
            self.controller.set_position(i, self.middle_initial_position);
        time.sleep(0.5);

    def __release_dignoal_right(self) :
        for i in Actuator.middle_diagonal_right_index :
            self.controller.set_position(i, self.middle_initial_position);
        time.sleep(0.5);

    def forward(self) :
        self.__hold_dignoal_left();
        time.sleep(0.1);
    
        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] - self.__moving_degree);
        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] + self.__moving_degree);
        time.sleep(0.5);
    
        self.__release_dignoal_left();
        self.__hold_dignoal_right();
        time.sleep(0.1);
    
        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        time.sleep(0.5);
    
        self.__release_dignoal_right();

    def backward(self) :
        pass;

    def right(self) :
        self.__hold_dignoal_left();
        time.sleep(0.05);

        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] - self.__moving_degree * ( 1 if i % 2 == 1 else -1 ));
        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] + self.__moving_degree * ( 1 if i % 2 == 1 else -1 ));
        time.sleep(0.05);
    
        self.__release_dignoal_left();
        self.__hold_dignoal_right();
        time.sleep(0.05);
    
        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        time.sleep(0.05);
    
        self.__release_dignoal_right();

    def left(self) :
        self.__hold_dignoal_right();
        time.sleep(0.05);

        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] + self.__moving_degree * ( 1 if i % 2 == 1 else -1 ));
        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] - self.__moving_degree * ( 1 if i % 2 == 1 else -1 ));
        time.sleep(0.05);
    
        self.__release_dignoal_right();
        self.__hold_dignoal_left();
        time.sleep(0.05);
    
        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        time.sleep(0.05);
    
        self.__release_dignoal_left();

    def change_mode(self) :
        self.__hold_dignoal_left();
        time.sleep(0.05);

        for i in Actuator.lower_diagonal_left_index :
            self.controller.set_speed(i, self.boost_speed);
            self.controller.set_raw_position(i, Actuator.position.center if i % 2 == 1 else 0);
        time.sleep(0.3);

        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_raw_position(i, Actuator.position.center);
        time.sleep(0.5);
        
        self.__release_dignoal_left();
        self.__hold_dignoal_right();
        time.sleep(0.05);

        for i in Actuator.lower_diagonal_right_index :
            self.controller.set_speed(i, self.boost_speed);
            self.controller.set_raw_position(i, Actuator.position.center if i % 2 == 0 else 0);
        time.sleep(0.3);

        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_raw_position(i, Actuator.position.center);
        time.sleep(0.5);

        self.__release_dignoal_right();
        time.sleep(0.05);

        for i in Actuator.middle_index :
            self.controller.set_raw_position(i, Actuator.position.center);

        return provider.Drive(self);