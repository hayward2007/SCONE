from provider import *;

class Sport(Mode) :

    def __init__(self) :
        super().__init__();
        self.upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
        self.middle_initial_position = 240;
        self.lower_initial_position = 255;

        self.safety_speed = 50;
        self.walk_speed = 100;
        