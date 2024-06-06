import time;

from ..core import *;
from ..provider import *;

class Walk(Mode) :
    __starting_middle_position = 105;
    __ending_middle_position = 150;

    # the moving degree of each step
    __moving_degree = 15;

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
    
    def __del__(self) :
        for i in Actuator.middle_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, self.__ending_middle_position);
        time.sleep(3);
        self.controller.disable_torque();
    
    def __hold_dignoal_left(self) :
        for i in Actuator.middle_diagonal_left_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, self.middle_initial_position - 15);
        time.sleep(0.5);
    
    def __hold_dignoal_right(self) :
        for i in Actuator.middle_diagonal_right_index :
            self.controller.set_speed(i, self.safety_speed);
            self.controller.set_position(i, self.middle_initial_position - 15);
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