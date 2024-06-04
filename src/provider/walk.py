from time import *;
from core import *;
from provider import *;

class Walk(Mode) :
    def __init__(self, controller: Controller) :
        super().__init__();
        self.controller = controller;
    
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
        asdf

    def backward(self) :
        asdf

    def left(self) :
        asdf

    def right(self) :
        asdf