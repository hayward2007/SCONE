import time
from enum import Enum
from getch import getch;
from InquirerPy import prompt;

from .core import *;
from .provider import *;

# You can use SCONE class as a SCONE api however SCONE.Cli is for my personal use
class SCONE :
    class Cli :
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
            self.__cli();
        
        def __set_status(self, status: __Status) :
            self.status = status;
            print("[SCONE] Status".ljust(35, " ") + f"[SET] {str(status)}".ljust(35, " "));
    
        def __set_operating_mode(self, operating_mode: __Operating_Mode) :
            self.operating_mode = operating_mode;
            print("[SCONE] Operating Mode".ljust(35, " ") + f"[SET] {str(operating_mode)}".ljust(35, " "));
        
        def __cli(self) :
            questions = [{
                "type": "list",
                "message": "What task you want to execute?",
                "choices": ["Remote", "Change Mode", "Actuator Settings", "System Settings", "Shutdown"],
            }];

            task = "Change Mode";
        
            while task != "Shutdown" :
                if task == "Remote" :
                    self.__remote();
                elif task == "Change Mode" :
                    self.__change_mode();
                elif task == "Actuator Settings" :
                    self.__actuator_settings();
                elif task == "System Settings" :
                    self.__system_settings();
                
                task = prompt(questions)[0];
        
        def __remote(self) :
            # changing operating mode ( pressing r ) has a sequence
            # Walk -> Drive -> Climb -> Walk -> Drive -> Climb ...
            print("[SCONE] Remote Control".ljust(35, " "));

            key = getch();

            while key != 'q' :
                key = getch();
                print(key);

                # if key == 'w' :
                #     self.__set_status(self.__Status.WALKING_FORWARD);
                #     self.operate.walk.forward();
                # elif key == 's' :
                #     self.__set_status(self.__Status.WALKING_BACKWARD);
                #     self.operate.walk.backward();
                # elif key == 'a' :
                #     if self.status == self.__Status.WALKING_LEFT :
                #         continue;
                    
                #     self.__set_status(self.__Status.WALKING_LEFT);
                #     self.operate.walk.left();
                #     self.__set_status(self.__Status.WALKING_STANCE);
                
                # elif key == 'd' :
                #     if self.status == self.__Status.WALKING_RIGHT :
                #         continue;
                    
                #     self.__set_status(self.__Status.WALKING_RIGHT);
                #     self.operate.walk.right();
                #     self.__set_status(self.__Status.WALKING_STANCE);
                
                # elif key = 'r' :
                    

                time.sleep(0.1);
        
        def __change_mode(self) :
            questions = [{
                "type": "list",
                "message": "What mode do you want to use?",
                "choices": ["Standard", "Sport"],
            }];

            task = prompt(questions)[0];

            if task == self.__Mode.STANDARD :
                self.mode = self.__Mode.STANDARD;
                self.operate = SCONE.Standard(self.controller);
            
            elif task == self.__Mode.SPORT :
                self.mode = self.__Mode.SPORT;
                self.operate = SCONE.Sport(self.controller);
            
            print("[SCONE] Mode".ljust(35, " ") + f"[SET] {str(mode)}".ljust(35, " "));
            self.__set_operating_mode(self.__Operating_Mode.WALK);
            self.__set_status(self.__Status.WALKING_STANCE);

        def __actuator_settings(self) :
            questions = [{
                "type": "list",
                "message": "What task you want to execute?",
                "choices": ["System Information", "Return"],
            }];
    
            task = prompt(questions);

            while task[0] != "Return" :
                if task[0] == "System Information" :
                    print("[SCONE] System Information".ljust(35, " "));
                
                task = prompt(questions);
    
        def __system_settings(self) :
            questions = [{
                "type": "list",
                "message": "What task you want to execute?",
                "choices": ["System Information", "Return"],
            }];
    
            task = prompt(questions);

            while task[0] != "Return" :
                if task[0] == "System Information" :
                    print("[SCONE] System Information".ljust(35, " "));
                
                task = prompt(questions);
    
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
            self.mode = Walk(self);

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
            self.mode = Walk(self);

    def __init__(self) :
        self.Cli();
