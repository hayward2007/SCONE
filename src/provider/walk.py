from time import *;
from core import *;
from provider import *;

class Walk(Mode) :
    # the moving degree of each step
    __moving_degree = 20;

    def __init__(self) :
        super().__init__();

        # initialize position
        self.controller.enable_torque();
        self.controller.set_all_speed(50);

        for i in Actuator.middle_index :
            self.controller.set_position(i, self.middle_initial_position - 105);
        time.sleep(0.5);

        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        for i in Actuator.lower_index :
            self.controller.set_speed(i, 150);
            self.controller.set_position(i, 195);
        time.sleep(0.7);

        self.controller.set_all_speed(50);
        for i in Actuator.middle_index :
            self.controller.set_position(i, self.middle_initial_position);
        time.sleep(1);

        self.controller.set_all_speed(self.walking_speed);
    
    def __hold_dignoal_left(self) :
        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.middle_initial_position - 15);
        time.sleep(0.5);
    
    def __hold_dignoal_right(self) :
        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.middle_initial_position - 15);
        time.sleep(0.5);

    def __release_dignoal_left(self) :
        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.middle_initial_position);
        time.sleep(0.5);

    def __release_dignoal_right(self) :
        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.middle_initial_position);
        time.sleep(0.5);

    def forward(self) :
        self.__hold_dignoal_left();
        time.sleep(0.3);
    
        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] - self.__moving_degree);
        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] + self.__moving_degree);
        time.sleep(0.5);
    
        self.__release_dignoal_left();
        self.__hold_dignoal_right();
        time.sleep(0.3);
    
        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        time.sleep(0.5);
    
        self.__release_dignoal_right();

    def backward(self) :
        pass;

    def left(self) :
        self.__hold_dignoal_left();
        time.sleep(0.3);

        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] - self.__moving_degree);
        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] + self.__moving_degree);
        time.sleep(0.5);
    
        self.__release_dignoal_left();
        self.__hold_dignoal_right();
        time.sleep(0.3);
    
        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        time.sleep(0.5);
    
        self.__release_dignoal_right();

    def right(self) :
        self.__hold_dignoal_right();
        time.sleep(0.3);

        for i in Actuator.upper_diagonal_right_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] - self.__moving_degree);
        for i in Actuator.upper_diagonal_left_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1] + self.__moving_degree);
        time.sleep(0.5);
    
        self.__release_dignoal_right();
        self.__hold_dignoal_left();
        time.sleep(0.3);
    
        for i in Actuator.upper_index :
            self.controller.set_position(i, self.upper_initial_position[i - 1]);
        time.sleep(0.5);

        self.__release_dignoal_left();