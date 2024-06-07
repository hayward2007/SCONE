import time;
from enum import Enum
from InquirerPy import prompt;

from .core import *;
from .provider import *;

class SCONE :
    class Cli :
        class __Command :
            commands = [{
                "type": "list",
                "message": "What task you want to execute?",
                "choices": ["Remote", "Change Mode", "Actuator Settings", "System Settings", "Shutdown"],
            }];

            mode = [{
                "type": "list",
                "message": "What mode do you want to use?",
                "choices": ["Standard", "Sport"],
            }];

            system_settings = [{
                "type": "list",
                "message": "What task you want to execute?",
                "choices": ["System Information", "Return"],
            }];

            operating_mode = [{
                "type": "list",
                "message": "What operating mode do you want to use?",
                "choices": ["Walk", "Drive", "Climb"],
            }];

        class __Status(Enum) :
            INITIALIZING = 0;

            WALKING_STANCE = 11;
            WALKING_FORWARD = 12;
            WALKING_BACKWARD = 13;
            WALKING_LEFT = 14;
            WALKING_RIGHT = 15;

            DRIVING_STANCE = 21;
            DRIVING_FORWARD = 22;
            DRIVING_BACKWARD = 23;
            DRIVING_LEFT = 24;
            DRIVING_RIGHT = 25;

            CLIMBING_STANCE = 31;
            CLIMBING_FORWARD = 32;
            CLIMBING_BACKWARD = 33;
            CLIMBING_LEFT = 34;
            CLIMBING_RIGHT = 35;
        
        class __Mode :
            STANDARD = "Standard";
            SPORT = "Sport";
        
        class __Operating_Mode(Enum) :
            WALK = 1;
            DRIVE = 2;
            CLIMB = 3;

        def __init__(self) :
            self.status = self.__Status.INITIALIZING;

            self.controller = Controller();
            # mode = prompt(self.__Command.mode);
            self.__set_mode(prompt(self.__Command.mode)[0]);
        
            task = prompt(self.__Command.commands);
        
            while task[0] != "Shutdown" :
                if task[0] == "Remote" :
                    print("[SCONE] Remote Control".ljust(35, " "));
                elif task[0] == "Change Mode" :
                    self.__set_mode(prompt(self.__Command.mode)[0]);
                
                task = prompt(self.__Command.commands);
        
        def __set_mode(self, mode) :
            if mode == self.__Mode.STANDARD :
                self.mode = self.__Mode.STANDARD;
                self.operate = SCONE.Standard(self.controller);
            
            elif mode == self.__Mode.SPORT :
                self.mode = self.__Mode.SPORT;
                self.operate = SCONE.Sport(self.controller);
            
            print("[SCONE] Mode".ljust(35, " ") + f"[SET] {str(mode)}".ljust(35, " "));
            self.__set_operating_mode(self.__Operating_Mode.WALK);
            self.__set_status(self.__Status.WALKING_STANCE);
        
        def __set_status(self, status: __Status) :
            self.status = status;
            print("[SCONE] Status".ljust(35, " ") + f"[SET] {str(status)}".ljust(35, " "));
    
        def __set_operating_mode(self, operating_mode: __Operating_Mode) :
            self.operating_mode = operating_mode;
            print("[SCONE] Operating Mode".ljust(35, " ") + f"[SET] {str(operating_mode)}".ljust(35, " "));
    
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
        self.Cli();
