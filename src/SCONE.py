import time;

from .core import *;
from .provider import *;

class SCONE :
    # operating modes
    class Standard(Mode) :
        def __init__(self, controller: Controller) :
            self.upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
            self.middle_initial_position = 240;
            self.lower_initial_position = 255;

            self.safety_speed = 50;
            self.walking_speed = 100;
            self.driving_speed = 150;
            self.climbing_speed = 200;

            self.controller = controller;
            self.walk = Walk(self, self.controller);

    class Sport(Mode) :
        def __init__(self, controller: Controller) :
            self.upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
            self.middle_initial_position = 165;
            self.lower_initial_position = 195;

            self.boost_speed = 150;
            self.safety_speed = 50;
            self.walking_speed = 100;
            self.driving_speed = 150;
            self.climbing_speed = 200;

            self.controller = controller;
            self.walk = Walk(self);

            # self.walk.forward();
            self.walk.left();
            self.walk.left();
            self.walk.left();
        
            time.sleep(4);
    
        def __del__(self) :
            for i in Actuator.middle_index :
                self.controller.set_speed(i, 30);
                self.controller.set_position(i, 150);
            time.sleep(1);
            self.controller.disable_torque();

    def __init__(self) :
        self.controller = Controller();
        self.sport = SCONE.Sport(self.controller);