import time;
from InquirerPy import prompt;
# from enum import Enum

from .core import *;
from .provider import *;

class SCONE :
    # class __Status(Enum) :
        # Walk_Standard = 0;
        
    # operating modes
    class Standard(Mode) :
        def __init__(self, controller: Controller) :
            self.upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
            self.middle_initial_position = 240;
            self.lower_initial_position = 255;

            self.boost_speed = 150;
            self.safety_speed = 50;
            self.walking_speed = 100;
            self.driving_speed = 150;
            self.climbing_speed = 200;

            self.controller = controller;
            self.walk = Walk(self);
        
            time.sleep(4);

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
        
            time.sleep(3);

    def __init__(self) :
        # initialize controller
        self.controller = Controller();

        questions = [
        # {"type": "input", "message": "What's your name:", "name": "name"},
            {
                "type": "list",
                "message": "What mode do you want to use?",
                "choices": ["Standard", "Sport"],
            },
            # {"type": "confirm", "message": "Confirm?"},
        ]
        result = prompt(questions);

        if result[0] == "Standard" :
            self.standard = SCONE.Standard(self.controller);
        elif result[0] == "Sport" :
            self.sport = SCONE.Sport(self.controller);